#include "AC_Avoid.h"

/// Constructor
AC_Avoid::AC_Avoid(const AP_InertialNav& inav, AC_P& P)
  : _inav(inav),
    _nvert(sizeof(_boundary)/sizeof(*_boundary)),
    _inside_position(Vector2f(0,0)),
    _kP(P.kP()),
    _accel_cmss(BREAKING_ACCEL_XY_CMSS),
    _enabled(false)
{}

/*
 * Adjusts the desired velocity so that the vehicle can stop
 * before the fence/object.
 */
void AC_Avoid::adjust_velocity(Vector2f &desired_vel) {
  if (!_enabled) {
    return;
  }

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
  } else if (num_intersects > 0 && (intersect - _inside_position).length() < EPSILON) {
    // Outside the fence but trying to get back in near last known inside position
    // Head inside at least as quickly as sqrt controller
    float min_speed = get_max_speed((position_xy - intersect).length());
    float desired_speed = desired_vel.length();
    if (desired_speed < min_speed && desired_speed > 0) {
      desired_vel *= min_speed/desired_speed;
    }
  } else {
    // Outside the fence
    // Head towards last known inside position using sqrt controller.
    Vector2f error = _inside_position - position_xy;
    float error_distance = error.length();
    if (error_distance != 0) {
      desired_vel = error * get_max_speed(error_distance) / error_distance;
    } else {
      desired_vel.zero();
    }
  }
}

/*
 * Tries to enable the geo-fence. Returns true if successful, false otherwise.
 */
bool AC_Avoid::enable() {
  Vector3f position_xyz = _inav.get_position();
  Vector2f position_xy(position_xyz.x,position_xyz.y);
  Vector2f intersect;
  unsigned num_intersects = poly_intersection(position_xy, Vector2f(1,0), _boundary, _nvert, intersect);
  if (num_intersects % 2 == 1) {
    // Inside the fence
    _enabled = true;
    _inside_position = position_xy;
  } else {
    _enabled = false;
  }
  return _enabled;
}

/*
 * Disables the geo-fence
 */
void AC_Avoid::disable() {
  _enabled = false;
}

/*
 * Sets the maximum x-y breaking acceleration.
 */
void AC_Avoid::set_breaking_accel_xy_cmss(float accel_cmss) {
  _accel_cmss = accel_cmss;
}


/*
 * Computes the speed such that the stopping distance
 * of the vehicle will be exactly the input distance.
 */
float AC_Avoid::get_max_speed(float distance) {
  return AC_AttitudeControl::sqrt_controller(distance,_kP,_accel_cmss);
}
