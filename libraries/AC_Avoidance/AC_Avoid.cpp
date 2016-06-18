#include "AC_Avoid.h"

const AP_Param::GroupInfo AC_Avoid::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Fence Type
    // @Description: Enabled fence types held as bitmask
    // @Values: 0:None,1:Polygon,2:Circle
    // @User: Standard
    AP_GROUPINFO("TYPE",        0,  AC_Avoid,   _enabled_fences,  AC_AVOID_TYPE_CIRCLE),

    AP_GROUPEND

};

/// Constructor
AC_Avoid::AC_Avoid(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_P& P, const AC_Fence& fence)
    : _inav(inav),
      _ahrs(ahrs),
      _nvert(sizeof(_boundary)/sizeof(*_boundary)),
      _kP(P.kP()),
      _accel_cmss(BREAKING_ACCEL_XY_CMSS),
      _fence(fence)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
 * Returns the point closest to p on the line segment (v,w).
 *
 * Comments and implementation taken from
 * http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
 */
static Vector2f closest_point(Vector2f p, Vector2f v, Vector2f w)
{
    // length squared of line segment
    const float l2 = (v - w).length_squared();
    if (is_zero(l2)) {
        // v == w case
        return v;
    }
    // Consider the line extending the segment, parameterized as v + t (w - v).
    // We find projection of point p onto the line.
    // It falls where t = [(p-v) . (w-v)] / |w-v|^2
    // We clamp t from [0,1] to handle points outside the segment vw.
    const float t = ((p - v) * (w - v)) / l2;
    if (t <= 0) {
        return v;
    } else if (t >= 1) {
        return w;
    } else {
        return v + (w - v)*t;
    }
}

void AC_Avoid::adjust_velocity(Vector2f &desired_vel, const float accel_cmss)
{
    _accel_cmss = accel_cmss;

    if (_enabled_fences == AC_AVOID_TYPE_POLY) {
        adjust_velocity_poly(desired_vel);
    } else if (_enabled_fences == AC_AVOID_TYPE_CIRCLE) {
        adjust_velocity_circle(desired_vel);
    }
}

/*
 * Adjusts the desired velocity for the circular fence.
 */
void AC_Avoid::adjust_velocity_circle(Vector2f &desired_vel)
{
    const Vector2f position_xy = get_position();

    float speed = desired_vel.length();
    // get the fence radius in cm
    const float fence_radius = get_radius();
    // get the margin to the fence in cm
    const float margin = get_margin();

    if (!is_zero(speed) && position_xy.length() <= fence_radius) {
        // Currently inside circular fence
        Vector2f stopping_point = position_xy + desired_vel*(get_stopping_distance(speed)/speed);
        float stopping_point_length = stopping_point.length();
        if (stopping_point_length > fence_radius - margin) {
            // Unsafe desired velocity - will not be able to stop before fence breach
            // Project stopping point radially onto fence boundary
            // Adjusted velocity will point towards this projected point at a safe speed
            Vector2f target = stopping_point * ((fence_radius - margin) / stopping_point_length);
            Vector2f target_direction = target - position_xy;
            float distance_to_target = target_direction.length();
            float max_speed = get_max_speed(distance_to_target);
            desired_vel = target_direction * (MIN(speed,max_speed) / distance_to_target);
        }
    }
}

/*
 * Adjusts the desired velocity for the polygon fence.
 */
void AC_Avoid::adjust_velocity_poly(Vector2f &desired_vel)
{
    const Vector2f position_xy = get_position();
    // Safe_vel will be adjusted to remain within fence.
    // We need a separate vector in case adjustment fails,
    // e.g. if we are exactly on the boundary.
    Vector2f safe_vel(desired_vel);

    if (!Polygon_outside(position_xy, _boundary, _nvert)) {
        unsigned i, j;
        for (i = 0, j = _nvert-1; i < _nvert; j = i++) {
            // end points of current edge
            Vector2f start = _boundary[j];
            Vector2f end = _boundary[i];
            // vector from current position to closest point on current edge
            Vector2f limit_direction = closest_point(position_xy, start, end) - position_xy;
            // distance to closest point
            const float limit_distance = limit_direction.length();
            if (!is_zero(limit_distance)) {
                // We are strictly inside the given edge.
                // Adjust velocity to not violate this edge.
                limit_direction /= limit_distance;
                limit_velocity(safe_vel, limit_direction, limit_distance);
            } else {
                // We are exactly on the edge - treat this as a fence breach.
                // i.e. do not adjust velocity.
                return;
            }
        }

        desired_vel = safe_vel;
    }
}

/*
 * Limits the component of desired_vel in the direction of the unit vector
 * limit_direction to be at most the maximum speed permitted by the limit_distance.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
void AC_Avoid::limit_velocity(Vector2f &desired_vel, const Vector2f limit_direction, const float limit_distance)
{
    const float max_speed = get_max_speed(limit_distance - get_margin());
    // project onto limit direction
    const float speed = desired_vel * limit_direction;
    if (speed > max_speed) {
        // subtract difference between desired speed and maximum acceptable speed
        desired_vel += limit_direction*(max_speed - speed);
    }
}

/*
 * Gets the current xy-position, relative to home (not relative to EKF origin)
 */
Vector2f AC_Avoid::get_position()
{
    const Vector3f position_xyz = _inav.get_position();
    const Vector2f position_xy(position_xyz.x,position_xyz.y);
    const Vector2f diff = location_diff(_inav.get_origin(),_ahrs.get_home());
    return position_xy + diff;
}

/*
 * Computes the speed such that the stopping distance
 * of the vehicle will be exactly the input distance.
 */
float AC_Avoid::get_max_speed(const float distance)
{
    return AC_AttitudeControl::sqrt_controller(distance,_kP,_accel_cmss);
}

/*
 * Computes distance required to stop, given current speed.
 *
 * Implementation copied from AC_PosControl.
 */
float AC_Avoid::get_stopping_distance(const float speed)
{
    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (_kP <= 0.0f || _accel_cmss <= 0.0f || is_zero(speed)) {
        return 0.0f;
    }

    // calculate point at which velocity switches from linear to sqrt
    float linear_speed = _accel_cmss/_kP;

    // calculate distance within which we can stop
    if (speed < linear_speed) {
        return speed/_kP;
    } else {
        float linear_distance = _accel_cmss/(2.0f*_kP*_kP);
        return linear_distance + (speed*speed)/(2.0f*_accel_cmss);
    }
}
