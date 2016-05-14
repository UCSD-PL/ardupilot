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

  const Vector3f position_xyz = _inav.get_position();
  const Vector2f position_xy(position_xyz.x,position_xyz.y);
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
      if (limit_distance != 0) {
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
  const Vector3f position_xyz = _inav.get_position();
  const Vector2f position_xy(position_xyz.x,position_xyz.y);
  if (Polygon_outside(position_xy, _boundary, _nvert)) {
    _enabled = false;
  } else {
    _enabled = true;
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
  return AC_AttitudeControl::sqrt_controller(distance - _buffer,_kP,_accel_cmss);
}
