#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/vector2.h>
#include <AP_Math/polygon.h>
#include <AP_Math/location.h>

#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AP_AHRS/AP_AHRS.h>     // AHRS library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude controller library for sqrt controller
#include <AC_PID/AC_P.h>               // P library
#include <AC_Fence/AC_Fence.h>         // Failsafe fence library

#define BREAKING_ACCEL_XY_CMSS 250.0f

// bit masks for enabled fence types.
#define AC_AVOID_TYPE_NONE                          0       // fence disabled
#define AC_AVOID_TYPE_POLY                          1       // horizontal polygon fence
#define AC_AVOID_TYPE_CIRCLE                        2       // circular horizontal fence

/*
 * This class prevents the vehicle from leaving a polygon fence in
 * 2 dimensions by limiting velocity (adjust_velocity).
 */
class AC_Avoid {
public:

    /// Constructor
    AC_Avoid(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_P& P, const AC_Fence& fence);

    /*
     * Adjusts the desired velocity so that the vehicle can stop
     * before the fence/object.
     */
    void adjust_velocity(Vector2f &desired_vel, const float accel_cmss);

    static const struct AP_Param::GroupInfo var_info[];

private:

    /*
     * Adjusts the desired velocity for the polygon fence.
     */
    void adjust_velocity_poly(Vector2f &desired_vel);

    /*
     * Adjusts the desired velocity for the circular fence.
     */
    void adjust_velocity_circle(Vector2f &desired_vel);

    /*
     * Limits the component of desired_vel in the direction of the unit vector
     * limit_direction to be at most the maximum speed permitted by the limit_distance.
     *
     * Uses velocity adjustment idea from Randy's second email on this thread:
     * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
     */
    void limit_velocity(Vector2f &desired_vel, const Vector2f limit_direction, const float limit_distance);

    /*
     * Gets the current position, relative to home (not relative to EKF origin)
     */
    Vector2f get_position();

    /*
     * Computes the speed such that the stopping distance
     * of the vehicle will be exactly the input distance.
     */
    float get_max_speed(const float distance);

    /*
     * Computes distance required to stop, given current speed.
     */
    float get_stopping_distance(const float speed);

    /*
     * Gets the fence radius in cm
     */
    float get_radius() { return _fence.get_radius() * 100.0f; }

    /*
     * Gets the fence margin in cm
     */
    float get_margin() { return _fence.get_margin() * 100.0f; }

    const AP_InertialNav& _inav;
    const AP_AHRS& _ahrs;
    /* Vector2f _boundary[5] = { */
    /*   Vector2f(-1000, -1000), */
    /*   Vector2f(1000, -1000), */
    /*   Vector2f(1000, 1000), */
    /*   Vector2f(-1000, 1000), */
    /*   Vector2f(-1000, -1000) */
    /* }; */
    Vector2f _boundary[8] = {
        Vector2f(-1000, -1000),
        Vector2f(1000, -1000),
        Vector2f(1000, 1000),
        Vector2f(500, 1000),
        Vector2f(500, 500),
        Vector2f(-500, 500),
        Vector2f(-1000, 1000),
        Vector2f(-1000, -1000)
    };
    unsigned _nvert;
    float _kP;
    float _accel_cmss;
    AP_Int8 _enabled_fences;
    const AC_Fence& _fence;
};
