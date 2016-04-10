#ifndef DEEPSTALL_H
#define DEEPSTALL_H

#include "pidcontroller.h"
#include <stdint.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Navigation/AP_Navigation.h>
#include <AP_Param/AP_Param.h>

enum STAGE {
    DEEPSTALL_FLY_TO_LOITER = 0,
    DEEPSTALL_LOITER,
    DEEPSTALL_APPROACH,
    DEEPSTALL_LAND,
};


class DeepStall
{
    public:
        DeepStall();
        void land(float track, float yawrate, Location current_loc);
        STAGE getApproachWaypoint(Location &target, Location &land_loc, Location &current_loc, Vector3f _wind, float deltah, int32_t heading_cd, AP_Navigation *nav_controller, float loiter_radius, float heading);
        float getRudderNorm();

        void computeApproachPath(Vector3f wind, float loiterRadius, float deltah, Location &landing, float heading);

        void abort();

        void setTargetHeading(float hdg, bool constrain);
        PIDController *YawRateController;

        float targetTrack;
        STAGE stage;
        bool ready;
        int32_t deepstall_start_time;
        Location loiter;
        Location loiter_exit;

        static const struct AP_Param::GroupInfo var_info[];
        AP_Float tcon;
        AP_Float ds_a;
        AP_Float ds_b;
        AP_Float l1_period;
        AP_Float l1_i;
        AP_Float kp;
        AP_Float ki;
        AP_Float kd;
        AP_Float ilim;
        AP_Float yaw_rate_limit;
        AP_Float approach_extension;
        AP_Float descent_speed;
        AP_Float forward_speed;
        AP_Int16 elevator; // PWM for completely stalled aircraft
        AP_Int16 approach_airspeed_cm;
        AP_Int16 controller_handoff_airspeed_cm;
        AP_Int16 slew_speed;
        AP_Int8 enable;

    private:
        float predictDistanceTraveled(Vector3f wind, float altitude);
        bool verify_loiter_breakout(Location &current_loc, int32_t heading_cd);

        Location extended_approach;
        Location landing_point;

        int32_t loiter_sum_cd;
        int32_t old_target_bearing_cd;

        float rCmd;
        float targetHeading;
        uint32_t _last_t;

        float l1_xtrack_i; // L1 integrator accumulator
};

#endif
