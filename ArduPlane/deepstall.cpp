#include "deepstall.h"
#include <math.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Navigation/AP_Navigation.h>

#define BOOL_TO_SIGN(bvalue) ((bvalue) ? -1 : 1)
extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo DeepStall::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable QuadPlane
    // @Description: This enables QuadPlane functionality, assuming quad motors on outputs 5 to 8
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, DeepStall, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: VF_B
    // @DisplayName: 2nd GPS type
    // @Description: GPS type of 2nd GPS
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
    // @RebootRequired: True
    AP_GROUPINFO("TCON",   2, DeepStall, tcon, 2.647),

    // @Param: DS_A
    // @DisplayName: GPS type
    // @Description: GPS type
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
    // @RebootRequired: True
    AP_GROUPINFO("DS_A",    3, DeepStall, ds_a, -1.486634076),

    // @Param: DS_B
    // @DisplayName: 2nd GPS type
    // @Description: GPS type of 2nd GPS
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
    // @RebootRequired: True
    AP_GROUPINFO("DS_B",   4, DeepStall, ds_b, 16.28549267),

    // @Param: DS_B
    // @DisplayName: 2nd GPS type
    // @Description: GPS type of 2nd GPS
    // @Values: 0:None,1:AUTO,2:uBlox,3:MTK,4:MTK19,5:NMEA,6:SiRF,7:HIL,8:SwiftNav,9:PX4-UAVCAN,10:SBF,11:GSOF
    // @RebootRequired: True
    AP_GROUPINFO("L1_I",   5, DeepStall, l1_i, 0.05f),
    
    AP_GROUPINFO("SLEW",   6, DeepStall, slew_speed, 250),
    AP_GROUPINFO("EXTD",   7, DeepStall, approach_extension, 100.0f),
    AP_GROUPINFO("L1_P",   8, DeepStall, l1_period, 15.0f),
    AP_GROUPINFO("AIRS",   9, DeepStall, approach_airspeed_cm, 1400),
    AP_GROUPINFO("ELEV",  10, DeepStall, elevator, 1200),
    AP_GROUPINFO("KP",    11, DeepStall, kp, 4.0f),
    AP_GROUPINFO("KI",    12, DeepStall, ki, 0.5f),
    AP_GROUPINFO("KD",    13, DeepStall, kd, 0.01f),
    AP_GROUPINFO("ILIM",  14, DeepStall, ilim, 0.2f),
    AP_GROUPINFO("YLIM",  15, DeepStall, yaw_rate_limit, 0.3f),
    AP_GROUPINFO("CTRL",  16, DeepStall, controller_handoff_airspeed_cm, 900),
    AP_GROUPINFO("VDWN",  17, DeepStall, descent_speed, 6.0f),
    AP_GROUPINFO("VFWD",  18, DeepStall, forward_speed, 10.0f),

    AP_GROUPEND
};


DeepStall::DeepStall() {
    YawRateController = new PIDController(0,0,0);
    YawRateController->setIntegralLimit(0);
    rCmd = 0;
    targetHeading = 0;
    _last_t = 0;
    stage = DEEPSTALL_FLY_TO_LOITER;
    ready = false;
    AP_Param::setup_object_defaults(this, var_info);
}

void DeepStall::abort() {
    YawRateController->setGains(kp, ki, kd);
    YawRateController->setIntegralLimit(ilim);
    YawRateController->resetIntegrator();
    stage = DEEPSTALL_FLY_TO_LOITER; // Reset deepstall stage in case of abort
    ready = false;
    _last_t = 0;
    loiter_sum_cd = 0;
    l1_xtrack_i = 0.0f;
}

float DeepStall::predictDistanceTraveled(Vector3f wind, float altitude) {
    float course = radians(targetHeading);

/*    Vector3f course_comp(cos(course), sin(course), 0.0f);
    float wind_comp = -1 * (course_comp * wind);

    float forward_distance = vf_a * wind_comp + vf_b;
    float stall_distance = ds_a * wind_comp + ds_b;
    Vector3f aligned_component(sin(course), cos(course), 0.0f);
    float v_e = forward_distance;// * (aligned_component * wind);
    return v_e * altitude / vspeed + stall_distance;
*/
// alternate comp version

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                                    "fsp %f, w %f, l1_i %f\n", 1.0f * forward_speed, wind.length(), 1.0f * l1_i);
    forward_speed = MAX(forward_speed, 0.1f);
    Vector2f wind_vec(wind.x, wind.y);
    Vector2f course_vec(cos(course), sin(course));

    float stall_distance = ds_a * wind_vec.length() + ds_b;

    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                  "theta = acos(%f) %f\n", acos((wind_vec * course_vec) / (MAX(wind_vec.length(), 0.05f) * course_vec.length())),
                  (wind_vec * course_vec) / (MAX(wind_vec.length(), 0.05f) * course_vec.length()));
    float theta = acos(CLAMP((wind_vec * course_vec) / (MAX(wind_vec.length(), 0.05f) * course_vec.length()), -1.0f, 1.0f));
    theta *= (course_vec % wind_vec) > 0 ? -1 : 1;
//    float forward_component = cosf(theta) * wind_vec.length();
    float cross_component = sinf(theta) * wind_vec.length();
    float estimated_crab_angle = asinf(CLAMP(cross_component / forward_speed, -1.0f,1.0f)); 
    estimated_crab_angle *= (course_vec % wind_vec) > 0 ? -1 : 1;
    float estimated_forward = cosf(estimated_crab_angle) * forward_speed + cosf(theta) * wind_vec.length();
//    hal.console->printf("estimated %f %f %f %f\n", estimated_forward, cosf(estimated_crab_angle) * forward_speed + cosf(theta) * wind_vec.length(), degrees(estimated_crab_angle), degrees(theta));    
// end alternate version

    return estimated_forward * altitude / descent_speed + stall_distance;
}

void DeepStall::computeApproachPath(Vector3f _wind, float loiterRadius, float deltah, Location &landing, float heading) {

        memcpy(&landing_point, &landing, sizeof(Location));
        memcpy(&extended_approach, &landing, sizeof(Location));
        memcpy(&loiter_exit, &landing, sizeof(Location));


        // extended approach point is 1km away so that there is a navigational target
        location_update(extended_approach, targetHeading, 1000.0);

	float d_predict = predictDistanceTraveled(_wind, deltah);
	
        location_update(loiter_exit, targetHeading + 180.0, d_predict + approach_extension);
        memcpy(&loiter, &loiter_exit, sizeof(Location));
        location_update(loiter, targetHeading + 90.0, loiterRadius);

        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
	                                 "Loiter: %3.8f %3.8f\n", loiter.lat / 1e7, loiter.lng / 1e7);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
	                                 "Loiter exit: %3.8f %3.8f\n", loiter_exit.lat / 1e7, loiter_exit.lng / 1e7);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
	                                 "Landing: %3.8f %3.8f\n", landing.lat / 1e7, landing.lng / 1e7);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
	                                 "Extended: %3.8f %3.8f\n", extended_approach.lat / 1e7, extended_approach.lng / 1e7);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                                         "Extended by: %f (%f)\n", d_predict + loiterRadius + approach_extension, d_predict);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
	                                 "Wind Heading: %3.1f\n\n", targetHeading);
	
}

bool DeepStall::verify_loiter_breakout(Location &current_loc, int32_t heading_cd) {
    // Bearing in degrees
    int32_t bearing_cd = get_bearing_cd(current_loc, extended_approach);

    int32_t heading_err_cd = wrap_180_cd(bearing_cd - heading_cd);

    /*
      Check to see if the the plane is heading toward the land
      waypoint. We use 20 degrees (+/-10 deg) of margin so that
      we can handle 200 degrees/second of yaw. We also require
      the altitude to be within 0.5 meters of desired, and
      enforce a minimum of one turn
    */
    if (loiter_sum_cd > 18000 &&
        labs(heading_err_cd) <= 1000  &&
        labs(loiter.alt - current_loc.alt) < 500) {
        // Want to head in a straight line from _here_ to the next waypoint instead of center of loiter wp
        return true;
    }
    return false;
}

STAGE DeepStall::getApproachWaypoint(Location &target, Location &land_loc, Location &current_loc, Vector3f _wind, float deltah, int32_t heading_cd, AP_Navigation *nav_controller, float loiter_radius, float heading) {

    // fly to the loiter point if we are to far away
    if (stage == DEEPSTALL_FLY_TO_LOITER && get_distance(current_loc, land_loc) > 500) {
        memcpy(&target, &loiter, sizeof(Location));
    } else {

        switch (stage) {
            case DEEPSTALL_FLY_TO_LOITER:
                if (get_distance(current_loc, loiter) > 2 * loiter_radius) {
                    memcpy(&target, &loiter, sizeof(Location));
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                                                     "Fly to loiter: d: %f\n", get_distance(current_loc, loiter));
                    break;
                } else {
                    // when within twice the loiter radius, fall through to loiter
                    stage = DEEPSTALL_LOITER;
                }
            case DEEPSTALL_LOITER:
                // fly at the point until it's been reached
                if (!nav_controller->reached_loiter_target()) {
                    memcpy(&target, &loiter, sizeof(Location));
                    old_target_bearing_cd = nav_controller->target_bearing_cd();
                    loiter_sum_cd = 0;
                    break;
                } else {
                    // update the loiter progress
                    loiter_sum_cd += wrap_180_cd(nav_controller->target_bearing_cd() - old_target_bearing_cd);
                    GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                                                     "Loiter: cd: %d\n", loiter_sum_cd);
                    old_target_bearing_cd = nav_controller->target_bearing_cd();
                    if (verify_loiter_breakout(current_loc, heading_cd)) {
                        // breakout of the loiter
                        stage = DEEPSTALL_APPROACH;
                        if (heading == 0.0) {
                            setTargetHeading(atan2(-_wind.y, -_wind.x) * 180 / M_PI, true);
                        }
                        computeApproachPath(_wind, loiter_radius, deltah, land_loc, heading);
                    } else {
                        memcpy(&target, &loiter, sizeof(Location));
                        break;
                    }
                }
            case DEEPSTALL_APPROACH: {
                // always fly at the extended approach point
                memcpy(&target, &extended_approach, sizeof(Location));
                // check if we should enter the stall
                Location entry_loc;
                memcpy(&entry_loc, &landing_point, sizeof(Location));
                float d_predict = predictDistanceTraveled(_wind, deltah);
                location_update(entry_loc, targetHeading + 180.0f, d_predict);
                GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "Approach: l: %f p: %f d: %f\n",
                                                 get_distance(current_loc, land_loc),
                                                 d_predict,
                                                 get_distance(current_loc, entry_loc));
                if (location_passed_point(current_loc, loiter_exit, entry_loc)) {
                    stage = DEEPSTALL_LAND;
                } else {
                    if (location_passed_point(current_loc, loiter, extended_approach)){
                        stage = DEEPSTALL_FLY_TO_LOITER;
                        loiter_sum_cd = 0;
                    }
                    break;
                }}
            case DEEPSTALL_LAND:
                memcpy(&target, &extended_approach, sizeof(Location));
                break;
        }
    }
    return stage;
}

void DeepStall::land(float track, float yawrate, Location current_loc) {

	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 10; // Default to 100 Hz
	}
	_last_t = tnow;

	// Target position controller
	// Generate equation of the tracking line parameters
	float course = radians(targetHeading);
	
        Vector2f ab = location_diff(loiter_exit, extended_approach);
        ab.normalize();
        Vector2f a_air = location_diff(loiter_exit, current_loc);

	float _crosstrack_error = a_air % ab;
        float sine_nu1 = _crosstrack_error / MAX(l1_period, 0.1f);
        sine_nu1 = constrain_float(sine_nu1, -0.7071f, 0.7107f);
        float nu1 = asinf(sine_nu1);

        if (l1_i > 0) {
            l1_xtrack_i += nu1 * l1_i * (1.0 / dt);
            l1_xtrack_i = constrain_float(l1_xtrack_i, -0.5f, 0.5f);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                                             "applied %f to %f %f\n", degrees(l1_xtrack_i), degrees(nu1), 1 * l1_i);
        }
        nu1 += l1_xtrack_i;

	targetTrack = course + nu1;

        float desiredChange = atan2(sin(wrap_PI(targetTrack) - track), cos(wrap_PI(targetTrack) - track));
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
            "delta %f %f %f %f\n",
            degrees(desiredChange),
            degrees(targetTrack),
            degrees(track));

        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO,
                   "%f %f %f %f %f\n",
                    _crosstrack_error,
                    CLAMP(desiredChange / tcon, -yaw_rate_limit, yaw_rate_limit), 
                    180 / M_PI * nu1,
                    yawrate * 180 / M_PI,
                    location_diff(current_loc, landing_point).length());
		
	rCmd = PIDController::saturate(
                 YawRateController->run(((float) dt)/1000.0,
                                        PIDController::wrap(CLAMP(desiredChange / tcon, -yaw_rate_limit, yaw_rate_limit) - yawrate,  -M_PI, M_PI)),
                 -1, 1);
}

float DeepStall::getRudderNorm() {
	return rCmd;
}

void DeepStall::setTargetHeading(float hdg, bool constrain) {
        if (constrain) {
            float delta = atan2(sin(wrap_PI(radians(hdg)) - radians(targetHeading)), cos(wrap_PI(radians(hdg)) - radians(targetHeading)));
            targetHeading = PIDController::wrap(CLAMP(delta, -15.0f, 15.0f) + targetHeading, -180.0f, 180.0f);
        } else {
            targetHeading = hdg;
        }
}
