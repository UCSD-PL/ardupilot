#include "AC_Avoid.h"

/// Constructor
AC_Avoid::AC_Avoid(const AP_InertialNav& inav)
  : _inav(inav),
    _nvert(4),
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
  if (num_intersects > 0 && num_intersects % 2 == 1) {
    // Inside the fence
    float max_speed = get_max_speed((position_xy - intersect).length());
    float desired_speed = desired_vel.length();
    if (desired_speed >= max_speed && desired_speed > 0) {
      desired_vel *= max_speed/desired_speed;
    }
    _inside_position = position_xy;
  } else {
    // Outside the fence
    // Head towards last known inside position at some fixed velocity.
    desired_vel = (_inside_position - position_xy).normalized()*RECOVERY_SPEED;
  }
}

/*
 * Computes the speed such that the stopping distance
 * of the vehicle will be exactly the input distance.
 */
float AC_Avoid::get_max_speed(float distance) {
  float linear_distance = _accel_cms/(2.0f*_kP*_kP);
  if (distance < linear_distance) {
    return distance*_kP;
  } else {
    return safe_sqrt((distance - linear_distance)*2.0f*_accel_cms);
  }
}
