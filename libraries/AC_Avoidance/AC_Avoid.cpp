#include "AC_Avoid.h"

/// Constructor
AC_Avoid::AC_Avoid(const AP_InertialNav& inav, AC_P& P)
  : _inav(inav),
    _nvert(sizeof(_boundary)/sizeof(*_boundary)),
    _inside_position(Vector2f(0,0)),
    _kP(P.kP()),
    _accel_cmss(BREAKING_ACCEL_XY_CMSS),
    _buffer(100.0f),
    _enabled(false)
{}

/*
 * Returns the point closest to p on the line segment (v,w).
 *
 * Comments and implementation taken from
 * http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
 */
static Vector2f closest_point(Vector2f p, Vector2f v, Vector2f w) {
  // length squared of line segment
  const float l2 = (v - w).length_squared();
  if (l2 == 0.0) {
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

  // TODO: Only do the following if we are strictly inside the boundary

  unsigned i, j;
  for (i = 0, j = _nvert-1; i < _nvert; j = i++) {
    Vector2f start = _boundary[j];
    Vector2f end = _boundary[i];
    Vector2f limit_direction = closest_point(position_xy, start, end) - position_xy;
    const float limit_distance = limit_direction.length();
    if (limit_distance != 0) {
      limit_direction /= limit_distance;
      limit_velocity(desired_vel, limit_direction, limit_distance);
    }
    // If the limit distance is equal to the 0, then we are exactly on the
    // boundary and should probably initiate failsafe
  }

}

/*
 * Limits the component of desired_vel in the direction of the unit vector
 * limit_direction to be at most the maximum speed permitted by the limit_distance.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
void AC_Avoid::limit_velocity(Vector2f &desired_vel, const Vector2f limit_direction, const float limit_distance) {
  const float max_speed = get_max_speed(limit_distance);
  // project onto limit direction
  const float speed = desired_vel * limit_direction;
  if (speed > max_speed) {
    // subtract difference between desired speed and maximum acceptable speed
    desired_vel += limit_direction*(max_speed - speed);
  }
}

/*
 * Tries to enable the geo-fence. Returns true if successful, false otherwise.
 */
bool AC_Avoid::enable() {
  // Vector3f position_xyz = _inav.get_position();
  // Vector2f position_xy(position_xyz.x,position_xyz.y);
  // Vector2f intersect;
  // unsigned num_intersects = poly_intersection(position_xy, Vector2f(1,0), _boundary, _nvert, intersect);
  // if (num_intersects % 2 == 1) {
  //   // Inside the fence
  //   _enabled = true;
  //   _inside_position = position_xy;
  // } else {
  //   _enabled = false;
  // }
  // TODO: Check if actually inside fence
  _enabled = true;
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
  return AC_AttitudeControl::sqrt_controller(distance - _buffer,_kP,_accel_cmss);
}
