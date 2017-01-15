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

void AP_Landing::type_deepstall_do_land(const AP_Mission::Mission_Command& cmd, const float relative_altitude)
{
    // for now all deepstalls are into the wind
    Vector3f wind = ahrs.wind_estimate();
    type_deepstall_set_target_heading(degrees(atan2(-wind.y, -wind.x)), false);

    // compute the approach path with the expected height over the stall point
    type_deepstall_build_approach_path(wind, cmd.content.location.alt / 100.0f, cmd.content.location);

    //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Landing approach start at %.1fm", (double)relative_altitude);
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
                if (!type_deepstall_verify_loiter_breakout(current_loc)) {
                    nav_controller->update_loiter(type_deepstall_loiter, aparm.loiter_radius, 1);
                    return false;
                }
            }
            type_deepstall_stage = DEEPSTALL_STAGE_APPROACH;
            // FIXME: recompute target heading and approach
            //fallthrough
        case DEEPSTALL_STAGE_APPROACH:
            nav_controller->update_waypoint(type_deepstall_loiter_exit, type_deepstall_extended_approach);

            Location entry_point;
            memcpy(&entry_point, &type_deepstall_landing_point, sizeof(Location));
            location_update(entry_point, type_deepstall_target_heading_deg + 180.0,
                            type_deepstall_predict_travel_distance(ahrs.wind_estimate(), height));

            if (!location_passed_point(current_loc, type_deepstall_loiter_exit, entry_point)) {
                if (location_passed_point(current_loc, type_deepstall_loiter_exit, type_deepstall_extended_approach)) {
                    // this should never happen, but prevent against an indefinite fly away
                    type_deepstall_stage = DEEPSTALL_STAGE_APPROACH_TARGET;
                }
                return false;
            }
            type_deepstall_stage = DEEPSTALL_STAGE_LAND;
            //fallthrough
        case DEEPSTALL_STAGE_LAND:
            // while in deepstall the only thing verify needs to keep the extended approach point sufficently far away
            memcpy(&type_deepstall_extended_approach, &type_deepstall_landing_point, sizeof(Location));
            location_update(type_deepstall_extended_approach, type_deepstall_target_heading_deg, 1000.0);
            return false;
        }
}

bool AP_Landing::type_deepstall_request_go_around(void)
{
    commanded_go_around = true;
    return true;
}

bool AP_Landing::type_deepstall_get_target_altitude_location(Location &location)
{
    memcpy(&location, &type_deepstall_landing_point, sizeof(Location));
    return true;
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

    location_update(type_deepstall_loiter_exit, type_deepstall_target_heading_deg + 180,
                    expected_travel_distance + type_deepstall_approach_extension);
    memcpy(&type_deepstall_loiter, &type_deepstall_loiter_exit, sizeof(Location));
    // TODO: Support loitering on either side of the approach path
    location_update(type_deepstall_loiter, type_deepstall_target_heading_deg + 90.0, aparm.loiter_radius);

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Loiter: %3.8f %3.8f\n",
                                     type_deepstall_loiter.lat / 1e7, type_deepstall_loiter.lng / 1e7);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Loiter exit: %3.8f %3.8f\n",
                                     type_deepstall_loiter_exit.lat / 1e7, type_deepstall_loiter_exit.lng / 1e7);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Landing: %3.8f %3.8f\n",
                                     type_deepstall_landing_point.lat / 1e7, type_deepstall_landing_point.lng / 1e7);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Extended: %3.8f %3.8f\n",
                                     type_deepstall_extended_approach.lat / 1e7, type_deepstall_extended_approach.lng / 1e7);
    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Extended by: %f (%f)\n",
                                     expected_travel_distance + aparm.loiter_radius + type_deepstall_approach_extension,
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

bool AP_Landing::type_deepstall_verify_loiter_breakout(const Location &current_loc) const
{
    int32_t heading_cd = (int32_t)(ahrs.groundspeed_vector().length() * 100.0);
    int32_t bearing_cd = get_bearing_cd(current_loc, type_deepstall_extended_approach);

    int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    // FIXME: incorporate airspeed error

    /* Check to see if the the plane is heading toward the land
       waypoint. We use 20 degrees (+/-10 deg) of margin so that
       we can handle 200 degrees/second of yaw. We also require
       the altitude to be within 0.5 meters of desired, and
       enforce a minimum of one turn
     */
    if (labs(heading_err_cd) <= 1000  &&  
        labs(type_deepstall_loiter.alt - current_loc.alt) < DEEPSTALL_LOITER_ALT_TOLERANCE) {
            // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp
            return true;
    }   
    return false;

}
