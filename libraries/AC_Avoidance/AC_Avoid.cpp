#include "AC_Avoid.h"

/*
 * Adjusts the desired velocity so that the vehicle can stop
 * before the fence/object.
 */
void AC_Avoid::adjust_velocity(Vector2f &desired_vel, const Vector2f position) {
  Vector2f intersect;
  unsigned num_intersects = boundary_intersection(position, desired_vel, _boundary, _nvert, intersect);
  if (num_intersects > 0 && num_intersects % 2 == 0) {
    // Inside the fence
    float max_speed = get_max_speed((position - intersect).length());
    float desired_speed = desired_vel.length();
    if (desired_speed >= max_speed && desired_speed > 0) {
      desired_vel *= max_speed/desired_speed;
    }
    _inside_position = position;
  } else {
    // Outside the fence
    // Head towards last known inside position at some fixed velocity.
    desired_vel = (_inside_position - position).normalized()*RECOVERY_SPEED;
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
