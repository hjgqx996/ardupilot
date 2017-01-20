/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *   AP_Landing_Deepstall.cpp - Landing logic handler for ArduPlane for deepstall landings
 */

#include "AP_Landing.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

void AP_Landing::type_deepstall_do_land(const AP_Mission::Mission_Command& cmd, const float relative_altitude)
{
    type_deepstall_stage = DEEPSTALL_STAGE_APPROACH_TARGET;

    // for now all deepstalls are into the wind
    Vector3f wind = ahrs.wind_estimate();
    type_deepstall_set_target_heading(degrees(atan2(-wind.y, -wind.x)), false);

    // compute the approach path with the expected height over the stall point
    type_deepstall_build_approach_path(wind, cmd.content.location.alt / 100.0f, cmd.content.location);
}

// currently identical to the slope aborts
void AP_Landing::type_deepstall_verify_abort_landing(const Location &prev_WP_loc, Location &next_WP_loc, bool &throttle_suppressed)
{
    // when aborting a landing, mimic the verify_takeoff with steering hold. Once
    // the altitude has been reached, restart the landing sequence
    throttle_suppressed = false;
    nav_controller->update_heading_hold(get_bearing_cd(prev_WP_loc, next_WP_loc));
}


/*
  update navigation for landing
 */
bool AP_Landing::type_deepstall_verify_land(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
        const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms,
        const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range)
{
        Location entry_point;
        SRV_Channel* elevator;
        float travel_distance;

        switch (type_deepstall_stage) {

        //deepstall is 

        case DEEPSTALL_STAGE_APPROACH_TARGET:
            if (get_distance(current_loc, type_deepstall_landing_point) > 500.0) {
                nav_controller->update_waypoint(current_loc, type_deepstall_landing_point);
                return false;
            }
            type_deepstall_stage = DEEPSTALL_STAGE_FLY_TO_LOITER;
            //fallthrough
        case DEEPSTALL_STAGE_FLY_TO_LOITER:
            if (get_distance(current_loc, type_deepstall_loiter) > 2 * aparm.loiter_radius) {
                nav_controller->update_waypoint(current_loc, type_deepstall_loiter);
                return false;
            }
            type_deepstall_stage = DEEPSTALL_STAGE_LOITER;
            //fallthrough
        case DEEPSTALL_STAGE_LOITER:
            if (!nav_controller->reached_loiter_target()) {
                nav_controller->update_loiter(type_deepstall_loiter, aparm.loiter_radius, 1);
                return false;
            } else {
                if (!type_deepstall_verify_loiter_breakout(current_loc, height)) {
                    nav_controller->update_loiter(type_deepstall_loiter, aparm.loiter_radius, 1);
                    return false;
                }
            }
            type_deepstall_stage = DEEPSTALL_STAGE_APPROACH;
            // FIXME: recompute target heading and approach
            //fallthrough
        case DEEPSTALL_STAGE_APPROACH:
            nav_controller->update_waypoint(type_deepstall_loiter_exit, type_deepstall_extended_approach);

            travel_distance = type_deepstall_predict_travel_distance(ahrs.wind_estimate(), height);

            memcpy(&entry_point, &type_deepstall_landing_point, sizeof(Location));
            location_update(entry_point, type_deepstall_target_heading_deg + 180.0, travel_distance);

            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "nav approach: %f %d", travel_distance,
            location_passed_point(current_loc, type_deepstall_loiter_exit, entry_point)
                            );

            if (!location_passed_point(current_loc, type_deepstall_loiter_exit, entry_point)) {
                if (location_passed_point(current_loc, type_deepstall_loiter_exit, type_deepstall_extended_approach)) {
                    // this should never happen, but prevent against an indefinite fly away
                    type_deepstall_stage = DEEPSTALL_STAGE_APPROACH_TARGET;
                }
                return false;
            }
            type_deepstall_stage = DEEPSTALL_STAGE_LAND;
            type_deepstall_stall_entry_time = AP_HAL::millis();
            elevator = SRV_Channels::get_channel_for(SRV_Channel::k_elevator);
            if (elevator != nullptr) {
                // take the last used elevator angle as the starting deflection
                // don't worry about bailing here if the elevator channel can't be found
                // that will be handled within control_servos
                type_deepstall_initial_elevator_pwm = elevator->get_output_pwm();
            }
            type_deepstall_l1_xtrack_i = 0; // reset the integrators
            //fallthrough
        case DEEPSTALL_STAGE_LAND:
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "landing");
            // while in deepstall the only thing verify needs to keep the extended approach point sufficently far away
            nav_controller->update_waypoint(current_loc, type_deepstall_extended_approach);
            return false;
        }
}

bool AP_Landing::type_deepstall_control_servos(void)
{
    if (!(type_deepstall_stage == DEEPSTALL_STAGE_LAND)) {
        return false;
    }

    SRV_Channel* elevator = SRV_Channels::get_channel_for(SRV_Channel::k_elevator);
    SRV_Channel* aileron = SRV_Channels::get_channel_for(SRV_Channel::k_aileron);

    if (elevator == nullptr || aileron == nullptr) {
        // deepstalls are impossible without these channels, abort the process
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Deepstall: Unable to find both a aileron and elevator channel");
        type_deepstall_request_go_around();
        return false;
    }

    // calculate the progress on slewing the elevator
    float slew_progress = 1.0f;
    if (type_deepstall_slew_speed > 0) {
        slew_progress  = constrain_float((AP_HAL::millis() - type_deepstall_stall_entry_time) /
                                         (100.0f * type_deepstall_slew_speed), 0.0f, 1.0f);
    }

    // mix the elevator to the correct value
    elevator->set_output_pwm(linear_interpolate(type_deepstall_initial_elevator_pwm, type_deepstall_elevator_pwm,
                             slew_progress, 0.0f, 1.0f));

    // use the current airspeed to dictate the travel limits
    float airspeed;
    ahrs.airspeed_estimate(&airspeed);

    // only allow the the deepstall steering controller to run below the handoff airspeed
    if (slew_progress >= 1.0f || airspeed <= type_deepstall_handoff_airspeed) {
        // run the steering conntroller
        float pid = type_deepstall_update_steering();


        float travel_limit = constrain_float((type_deepstall_handoff_airspeed - airspeed) /
                                             (type_deepstall_handoff_airspeed - type_deepstall_handoff_lower_limit_airspeed) *
                                             0.5f + 0.5f,
                                             0.5f, 1.0f);

        float output = constrain_float(pid, -travel_limit, travel_limit);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, output*4500);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron_with_input, output*4500);

    } else {
        // allow the normal servo control of the channel
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron_with_input,
                                        SRV_Channels::get_output_scaled(SRV_Channel::k_aileron));
    }

    // hand off rudder control to deepstall controlled
}

bool AP_Landing::type_deepstall_request_go_around(void)
{
    commanded_go_around = true;
    return true;
}

bool AP_Landing::type_deepstall_is_throttle_suppressed(void) const
{
    return type_deepstall_stage == DEEPSTALL_STAGE_LAND;
}

bool AP_Landing::type_deepstall_get_target_altitude_location(Location &location)
{
    memcpy(&location, &type_deepstall_landing_point, sizeof(Location));
    return true;
}

const DataFlash_Class::PID_Info& AP_Landing::type_deepstall_get_pid_info(void) const
{
    return type_deepstall_PID.get_pid_info();
}

void AP_Landing::type_deepstall_set_target_heading(const float heading_deg, const bool constrain)
{
    if (constrain) {
        // FIXME: What is happening here???
        float delta = wrap_PI(radians(heading_deg - type_deepstall_target_heading_deg));
        delta = atan2(sinf(delta), cosf(delta));
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "FIXME: Deepstall constrain heading not yet implemented");
//        targetHeading = wrap(CLAMP(delta, -15.0f, 15.0f) + targetHeading, -180.0f, 180.0f);
    } else {
        type_deepstall_target_heading_deg = heading_deg;
    }
}

void AP_Landing::type_deepstall_build_approach_path(const Vector3f wind, const float height, const Location &landing_point)
{
    memcpy(&type_deepstall_landing_point, &landing_point, sizeof(Location));
    memcpy(&type_deepstall_extended_approach, &landing_point, sizeof(Location));
    memcpy(&type_deepstall_loiter_exit, &landing_point, sizeof(Location));

    //extend the approach point to 1km away so that there is always a navigational target
    location_update(type_deepstall_extended_approach, type_deepstall_target_heading_deg, 1000.0);

    float expected_travel_distance = type_deepstall_predict_travel_distance(wind, height);
    float approach_extension = expected_travel_distance + type_deepstall_approach_extension + aparm.loiter_radius;
    // an approach extension of 0 can result in a divide by 0
    if (fabsf(approach_extension) < 1.0f) {
        approach_extension = 1.0f;
    }

    location_update(type_deepstall_loiter_exit, type_deepstall_target_heading_deg + 180, approach_extension);
    memcpy(&type_deepstall_loiter, &type_deepstall_loiter_exit, sizeof(Location));
    // TODO: Support loitering on either side of the approach path
    location_update(type_deepstall_loiter, type_deepstall_target_heading_deg + 90.0, aparm.loiter_radius);

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Loiter: %3.8f %3.8f\n",
                                     type_deepstall_loiter.lat / 1e7, type_deepstall_loiter.lng / 1e7);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Loiter ex: %3.8f %3.8f\n",
                                     type_deepstall_loiter_exit.lat / 1e7, type_deepstall_loiter_exit.lng / 1e7);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Landing: %3.8f %3.8f\n",
                                     type_deepstall_landing_point.lat / 1e7, type_deepstall_landing_point.lng / 1e7);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Extended: %3.8f %3.8f\n",
                                     type_deepstall_extended_approach.lat / 1e7, type_deepstall_extended_approach.lng / 1e7);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Extended by: %f (%f)\n", approach_extension,
                                     expected_travel_distance);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Target Heading: %3.1f\n\n", type_deepstall_target_heading_deg);

}

float AP_Landing::type_deepstall_predict_travel_distance(const Vector3f wind, const float height) const
{
    bool reverse = false;

    float course = radians(type_deepstall_target_heading_deg);

    // a forward speed of 0 will result in a divide by 0
    float forward_speed = MAX(type_deepstall_forward_speed, 0.1f);

    Vector2f wind_vec(wind.x, wind.y); // work with the 2D component of wind
    float wind_length = MAX(wind_vec.length(), 0.05f); // always assume a slight wind to avoid divide by 0
    Vector2f course_vec(cosf(course), sinf(course));

    float offset = course + atan2(-wind.y, -wind.x) + M_PI;

    float stall_distance = type_deepstall_slope_a * wind_length * cos(offset) + type_deepstall_slope_b;

    float theta = constrain_float((wind_vec * course_vec) / wind_length, -1.0f, 1.0f);
    if ((course_vec % wind_vec) > 0) {
        reverse = true;
        theta *= -1;
    }

    float cross_component = sinf(theta) * wind_length;
    float estimated_crab_angle = asinf(constrain_float(cross_component / forward_speed, -1.0f, 1.0f));
    if (reverse) {
        estimated_crab_angle *= -1;
    }

    float estimated_forward = cosf(estimated_crab_angle) * forward_speed + cosf(theta) * wind_length;

    return estimated_forward * height / type_deepstall_down_speed + stall_distance;
}

bool AP_Landing::type_deepstall_verify_loiter_breakout(const Location &current_loc, const float height_error) const
{
    Vector2f location_delta = location_diff(current_loc, type_deepstall_extended_approach);
    float heading_error = degrees(ahrs.groundspeed_vector().angle(location_delta));

    // FIXME: incorporate airspeed error

    /* Check to see if the the plane is heading toward the land waypoint. We use 20 degrees (+/-10 deg)
       of margin so that the altitude to be within 5 meters of desired
     */

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Breakout: %f %f", heading_error, height_error);

    if (heading_error <= 10.0  && fabsf(height_error) < DEEPSTALL_LOITER_ALT_TOLERANCE) {
            // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp
            return true;
    }   
    return false;
}

float AP_Landing::type_deepstall_update_steering()
{
    Location current_loc;
    if (!ahrs.get_position(current_loc)) {
        // panic if no position source is available
        // continue the deepstall. but target just holding the wings held level as deepstall should be a minimal energy
        // configuration on the aircraft, and if a position isn't available aborting would be worse
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_CRITICAL, "Deepstall: No position available. Attempting to hold level");
        memcpy(&current_loc, &type_deepstall_landing_point, sizeof(Location));
    }
    uint32_t time = AP_HAL::millis();
    float dt = constrain_float(time - type_deepstall_last_time, (uint32_t)10UL, (uint32_t)200UL) / 1000.0;
    type_deepstall_last_time = time;


    Vector2f ab = location_diff(type_deepstall_loiter_exit, type_deepstall_extended_approach);
    ab.normalize();
    Vector2f a_air = location_diff(type_deepstall_loiter_exit, current_loc);

    float crosstrack_error = a_air % ab;
    float sine_nu1 = constrain_float(crosstrack_error / MAX(type_deepstall_l1_period, 0.1f), -0.7071f, 0.7107f);
    float nu1 = asinf(sine_nu1);

    if (type_deepstall_l1_i > 0) {
        type_deepstall_l1_xtrack_i += nu1 * type_deepstall_l1_i / dt;
        type_deepstall_l1_xtrack_i = constrain_float(type_deepstall_l1_xtrack_i, -0.5f, 0.5f);
        nu1 += type_deepstall_l1_xtrack_i;
    }

    float desired_change = wrap_PI(radians(type_deepstall_target_heading_deg) + nu1 - ahrs.yaw);

    float yaw_rate = ahrs.get_gyro().z;
    float yaw_rate_limit = radians(type_deepstall_yaw_rate_limit);
    float error = wrap_PI(constrain_float(desired_change / type_deepstall_time_constant,
                                          -yaw_rate_limit, yaw_rate_limit));

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "x: %f e: %f r: %f d: %f",
                                     crosstrack_error,
                                     error,
                                     degrees(yaw_rate),
                                     location_diff(current_loc, type_deepstall_landing_point).length());

    float pid = type_deepstall_PID.get_pid(error);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "c: %f p: %f", desired_change, pid);

    return pid;
}
