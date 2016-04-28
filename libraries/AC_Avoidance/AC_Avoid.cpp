#include "AC_Avoid.h"

/// Constructor
AC_Avoid::AC_Avoid(const AP_InertialNav& inav)
  : _inav(inav),
    _nvert(sizeof(_boundary)/sizeof(*_boundary)),
    _inside_position(Vector2f(0,0)),
    _accel_cms(250.0f),
    _kP(1.0f)
{}

/*
 * Adjusts the desired velocity so that the vehicle can stop
 * before the fence/object.
 */
void AC_Avoid::adjust_velocity(Vector2f &desired_vel) {
  Vector3f position_xyz = _inav.get_position();
  Vector2f position_xy(position_xyz.x,position_xyz.y);
  Vector2f intersect;
  unsigned num_intersects = poly_intersection(position_xy, desired_vel, _boundary, _nvert, intersect);
  if (num_intersects % 2 == 1) {
    // Inside the fence
    float max_speed = get_max_speed((position_xy - intersect).length());
    float desired_speed = desired_vel.length();
    if (desired_speed >= max_speed && desired_speed > 0) {
      desired_vel *= max_speed/desired_speed;
    }
    _inside_position = position_xy;
  } else {
    // Outside the fence
    // Head towards last known inside position using sqrt controller.
    Vector2f error = (_inside_position - position_xy);
    float distance = error.length();
    desired_vel = error * get_max_speed(distance) / distance;
  }
}

/*
 * Computes the speed such that the stopping distance
 * of the vehicle will be exactly the input distance.
 */
float AC_Avoid::get_max_speed(float distance) {
  return AC_AttitudeControl::sqrt_controller(distance,_kP,_accel_cms);
}
