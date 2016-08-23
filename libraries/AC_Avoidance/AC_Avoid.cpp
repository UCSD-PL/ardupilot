#include "AC_Avoid.h"

const AP_Param::GroupInfo AC_Avoid::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Avoidance control enable/disable
    // @Description: Enabled/disable stopping at fence
    // @Values: 0:None,1:StopAtFence
    // @User: Standard
    AP_GROUPINFO("ENABLE", 1,  AC_Avoid, _enabled, AC_AVOID_STOP_AT_FENCE),

    AP_GROUPEND
};

/// Constructor
AC_Avoid::AC_Avoid(const AP_AHRS& ahrs, const AP_InertialNav& inav, const AC_Fence& fence)
    : _ahrs(ahrs),
      _inav(inav),
      _fence(fence)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AC_Avoid::adjust_velocity(const float kP, const float accel_cmss, Vector2f &desired_vel)
{
    // exit immediately if disabled
    if (_enabled == AC_AVOID_DISABLED) {
        return;
    }

    // limit acceleration
    float accel_cmss_limited = MIN(accel_cmss, AC_AVOID_ACCEL_CMSS_MAX);

    if (_enabled == AC_AVOID_STOP_AT_FENCE) {
        adjust_velocity_circle(kP, accel_cmss_limited, desired_vel);
        adjust_velocity_poly(kP, accel_cmss_limited, desired_vel);
    }
}

// convenience function to accept Vector3f.  Only x and y are adjusted
void AC_Avoid::adjust_velocity(const float kP, const float accel_cmss, Vector3f &desired_vel)
{
    Vector2f des_vel_xy(desired_vel.x, desired_vel.y);
    adjust_velocity(kP, accel_cmss, des_vel_xy);
    desired_vel.x = des_vel_xy.x;
    desired_vel.y = des_vel_xy.y;
}

/*
 * Adjusts the desired velocity for the circular fence.
 */
void AC_Avoid::adjust_velocity_circle(const float kP, const float accel_cmss, Vector2f &desired_vel)
{
    // exit if circular fence is not enabled
    if ((_fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) == 0) {
        return;
    }

    // exit if the circular fence has already been breached
    if ((_fence.get_breaches() & AC_FENCE_TYPE_CIRCLE) != 0) {
        return;
    }

    // get position as a 2D offset in cm from ahrs home
    const Vector2f position_xy = get_position();

    float speed = desired_vel.length();
    // get the fence radius in cm
    const float fence_radius = _fence.get_radius() * 100.0f;
    // get the margin to the fence in cm
    const float margin = get_margin();

    if (!is_zero(speed) && position_xy.length() <= fence_radius) {
        // Currently inside circular fence
        Vector2f stopping_point = position_xy + desired_vel*(get_stopping_distance(kP, accel_cmss, speed)/speed);
        float stopping_point_length = stopping_point.length();
        if (stopping_point_length > fence_radius - margin) {
            // Unsafe desired velocity - will not be able to stop before fence breach
            // Project stopping point radially onto fence boundary
            // Adjusted velocity will point towards this projected point at a safe speed
            Vector2f target = stopping_point * ((fence_radius - margin) / stopping_point_length);
            Vector2f target_direction = target - position_xy;
            float distance_to_target = target_direction.length();
            float max_speed = get_max_speed(kP, accel_cmss, distance_to_target);
            desired_vel = target_direction * (MIN(speed,max_speed) / distance_to_target);
        }
    }
}

/*
 * Adjusts the desired velocity for the polygon fence.
 */

void AC_Avoid::adjust_velocity_poly(const float kP, const float accel_cmss, Vector2f &desired_vel)
{
    // exit if the polygon fence is not enabled
    if ((_fence.get_enabled_fences() & AC_FENCE_TYPE_POLYGON) == 0) {
        return;
    }

    // exit if the polygon fence has already been breached
    if ((_fence.get_breaches() & AC_FENCE_TYPE_POLYGON) != 0) {
        return;
    }

    // get polygon boundary
    // Note: first point in list is the return-point (which copter does not use)
    uint16_t num_points;
    Vector2f* boundary = _fence.get_polygon_points(num_points);

    // exit if there are no points
    if (boundary == NULL || num_points == 0) {
        return;
    }

    // do not adjust velocity if vehicle is outside the polygon fence
    const Vector3f& position = _inav.get_position();
    Vector2f position_xy(position.x, position.y);
    if (_fence.boundary_breached(position_xy, num_points, boundary)) {
        return;
    }

    // Safe_vel will be adjusted to remain within fence.
    // We need a separate vector in case adjustment fails,
    // e.g. if we are exactly on the boundary.
    Vector2f safe_vel(desired_vel);

    uint16_t i, j;
    for (i = 1, j = num_points-1; i < num_points; j = i++) {
        // end points of current edge
        Vector2f start = boundary[j];
        Vector2f end = boundary[i];
        // vector from current position to closest point on current edge
        Vector2f limit_direction = Vector2f::closest_point(position_xy, start, end) - position_xy;
        // distance to closest point
        const float limit_distance = limit_direction.length();
        if (!is_zero(limit_distance)) {
            // We are strictly inside the given edge.
            // Adjust velocity to not violate this edge.
            limit_direction /= limit_distance;
            limit_velocity(kP, accel_cmss, safe_vel, limit_direction, limit_distance);
        } else {
            // We are exactly on the edge - treat this as a fence breach.
            // i.e. do not adjust velocity.
            return;
        }
    }

    desired_vel = safe_vel;
}

/*
 * Returns a stopping point that is visible from position within boundary.
 * This function should only be called if position is within boundary.
 * The stopping point returned is the closest point to desired_stopping_point
 * on the visible region bounded by the convex chain of edges that includes
 * the edge intersected by the segment from position to desired_stopping_point.
 * If these segments do not intersect, then desired_stopping_point is returned.
 */
Vector2f AC_Avoid::get_visible_stopping_point(const Vector2f* boundary, const uint16_t num_points, const Vector2f position, const Vector2f desired_stopping_point)
{
  // The edge intersected by the vector from the current position to the desired
  // stopping point.
  // If the value of this variable is i, then the edge is from i to (i + 1).
  // This is the convention followed for all subsequent edge variables.
  const uint16_t fst_edge;
  if (poly_intersection(_boundary, num_points, position, desired_stopping_point, fst_edge)) {
    // The head of the stack of visible edges.
    uint16_t head = fst_edge;
    Vector2f head_start, head_end;
    get_edge(boundary, num_points, head, head_start, head_end);
    // Whether to go around the boundary clockwise or counter-clockwise
    const uint8_t incr = ((head_end - head_start)*(desired_stopping_point - position) > 0 ) ? 1 : -1;
    // The "optimal" visible point on the head of the stack.
    Vector2f visible = closest_point_line(desired_stopping_point, head_start, head_end);;
    float error_sq = (visible - desired_stopping_point).length_squared();
    // The start and end of the current edge being processed in the following loop.
    Vector2f start, end;
    // Iterate through all remaining edges on the boundary in the order determined by incr
    for (uint16_t i = fst_edge + incr; i != fst_edge; i = circular_increment(i, num_points, incr)) {
      // get the current edge
      get_edge(boundary, num_points, i, start, end);
      
      // We don't care what the intersection point is, just whether it intersects.
      if (segment_segment_intersection(position, visible, start, end, Vector2f(0,0))) {
	// If edge intersects segment from position to visible point,
	// find where ray from position to end intersects stack.
	// This intersection point is the new visible point, and the
	// corresponding edge is the new head of the stack.
	while (!ray_segment_intersection(position, end, head_start, head_end, visible) && head_start != fst_edge) {
	  head = circular_increment(head, num_points, -incr);
	  get_edge(boundary, num_points, head, head_start, head_end);
	}
      } else if ((incr > 0 && head_end == start && !reflex_angle(end, head_end, head_start)) ||
		 (incr < 0 && head_start == end && !reflex_angle(head_end, head_start, start))) {
	Vector2f closest_point = closest_point_line(desired_stopping_point, start, end);
	float distance_sq = (closest_point - desired_stopping_point).length_squared();
	if (distance_sq <= error_sq) {
	  // If edge is adjacent to stack head and forms a non-reflex interior angle
	  // and edge is closer to desired stopping point, push edge onto stack.
	  head = start;
	  get_edge(boundary, num_points, head, head_start, head_end);
	  visible = closest_point;
	  error_sq = distance_sq;
	}
      }
    }

    // TODO: scale point towards position until it is at least margin distance
    // from boundary. This does not quite work. Perhaps margin computation should
    // be in main loop.
    return visible;
  } else {
    // desired stopping point is visible
    return desired_stopping_point;
  }
}

/*
 * Returns the point closest to p on the line (v,w).
 */
static Vector2f closest_point_line(const Vector2f &p, const Vector2f &v, const Vector2f &w)
{
  // length squared of line segment
  const Vector2f w_v = w - v;
  const float l2 = w_v.length_squared();
  if (l2 < FLT_EPSILON) {
    // v == w case
    return v;
  }
  return v + (w_v * (((p - v) * w_v)) / l2));
}

/*
 * Returns true if the counterclockwise angle from A to C (with vertex B)
 * has magnitude > 180.
 */
bool AC_Avoid::reflex_angle(Vector2f A, Vector2f B, Vector2f C)
{
  return (A - B) % (C - B) < 0;
}

/*
 * Compute the edge of the polygon intersected by
 * the line segment (p1,p2).
 * Returns true if there is such an intersection.
 */
bool poly_intersection(const Vector2f *poly, const uint16_t num_points, const Vector2f p1, const Vector2f p2, const uint16_t intersect)  {
  unsigned i, j;
  float distance = FLT_MAX;
  bool has_intersection;
  for (i = 1, j = num_points-1; i < num_points; j = i++) {
    Vector2f start = poly[j];
    Vector2f end = poly[i];
    Vector2f next_intersect;
    if (segment_segment_intersection(p1, p2, start, end, next_intersect)) {
      const float next_distance = (next_intersect - position).length();
      if (next_distance <= distance) {
	intersect = i;
	distance = next_distance;
	has_intersection = true;
      }
    }
  }

  return has_intersection;
}

/*
 * Computes the intersection between the ray (p1,p2) and the line segment (q1,q2),
 * excluding the point q2.
 * Returns false if they do not intersect. Returns false if they are collinear.
 * Returns true otherwise.
 *
 * Sets intersect to the intersecting point.
 *
 * Implementation adapted from
 * http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
 */
bool AC_Avoid::ray_segment_intersection(Vector2f p1, Vector2f p2, Vector2f q1, Vector2f q2, Vector2f &intersect) {
  float t, u;
  if (line_line_intersection(p1, p2, q1, q2, t, u)) {
    if (0 <= u && u < 1 && 0 <= t) {
      // lines intersect
      // t can be any non-negative value because (p, r) is a ray
      // u must be between 0 and 1 because (q, s) is a line segment
      intersect = p1 + (r*t);
      return true;
    } else {
      // non-parallel and non-intersecting
      return false;
    }
  } else {
    return false;
  }
}

/*
 * Computes the intersection between the line segment (p1,p2) and the line segment (q1,q2),
 * excluding the point q2.
 * Returns false if they do not intersect. Returns false if they are collinear.
 * Returns true otherwise.
 *
 * Sets intersect to the intersecting point.
 *
 * Implementation adapted from
 * http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
 */
bool AC_Avoid::segment_segment_intersection(Vector2f p1, Vector2f p2, Vector2f q1, Vector2f q2, Vector2f &intersect) {
  float t, u;
  if (line_line_intersection(p1, p2, q1, q2, t, u)) {
    if (0 <= u && u < 1 && 0 <= t && t <= 1) {
      // lines intersect
      // t can be any non-negative value because (p, r) is a ray
      // u must be between 0 and 1 because (q, s) is a line segment
      intersect = p1 + (r*t);
      return true;
    } else {
      // non-parallel and non-intersecting
      return false;
    }
  } else {
    return false;
  }
}

/*
 * Computes the intersection between the line (p1,p2) and the line (q1,q2).
 * Returns false if they are collinear or parallel.
 * Returns true otherwise.
 *
 * If the function returns true, t and u are set such that
 *     p1 + (p2 - p1) * t = q1 + (q2 - q1) * u
 * 
 *
 * Implementation adapted from
 * http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
 */
bool AC_Avoid::line_line_intersection(Vector2f p1, Vector2f p2, Vector2f q1, Vector2f q2, float& t, float& u) {
  Vector2f r = p2 - p1;
  Vector2f s = q2 - q1;
  Vector2f q_p = q1 - p1;
  float rxs = r%s;
  float q_pxr = q_p % r;
  if (rxs == 0) {
    // either collinear or parallel and non-intersecting
    return false;
  } else {
    // t = (q − p) × s / (r × s)
    // u = (q − p) × r / (r × s)
    t = (q_p % s) / rxs;
    u = q_pxr / rxs;
    return true;
  }
}

/*
 * Increments the index i for a circular array of size N using the increment incr.
 * incr must be -1 or 1.
 */
uint16_t AC_Avoid::circular_increment(const uint16_t i, const uint16_t N, const uint8_t incr)
{
  // The zeroth point in the boundary is the return point,
  // which the copter doesn't use.
  if (incr < 0 && i == 1) {
    return N - 1;
  } else if (incr > 0 && i == N - 1) {
    return 1;
  } else {
    return i + incr;
  }
}

/*
 * Sets start and end to the start and end points of the ith edge of boundary.
 */
void AC_Avoid::get_edge(const Vector2f* boundary, const uint16_t num_points, const uint16_t i, Vector2f& start, Vector2f& end)
{
  start = boundary[i];
  end = boundary[circular_increment(i, num_points, 1)];
}

/*
 * Limits the component of desired_vel in the direction of the unit vector
 * limit_direction to be at most the maximum speed permitted by the limit_distance.
 *
 * Uses velocity adjustment idea from Randy's second email on this thread:
 * https://groups.google.com/forum/#!searchin/drones-discuss/obstacle/drones-discuss/QwUXz__WuqY/qo3G8iTLSJAJ
 */
void AC_Avoid::limit_velocity(const float kP, const float accel_cmss, Vector2f &desired_vel, const Vector2f limit_direction, const float limit_distance) const
{
    const float max_speed = get_max_speed(kP, accel_cmss, limit_distance - get_margin());
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
    const Vector2f diff = location_diff(_inav.get_origin(),_ahrs.get_home()) * 100.0f;
    return position_xy - diff;
}

/*
 * Computes the speed such that the stopping distance
 * of the vehicle will be exactly the input distance.
 */
float AC_Avoid::get_max_speed(const float kP, const float accel_cmss, const float distance) const
{
    return AC_AttitudeControl::sqrt_controller(distance, kP, accel_cmss);
}

/*
 * Computes distance required to stop, given current speed.
 *
 * Implementation copied from AC_PosControl.
 */
float AC_Avoid::get_stopping_distance(const float kP, const float accel_cmss, const float speed) const
{
    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (kP <= 0.0f || accel_cmss <= 0.0f || is_zero(speed)) {
        return 0.0f;
    }

    // calculate point at which velocity switches from linear to sqrt
    float linear_speed = accel_cmss/kP;

    // calculate distance within which we can stop
    if (speed < linear_speed) {
        return speed/kP;
    } else {
        float linear_distance = accel_cmss/(2.0f*kP*kP);
        return linear_distance + (speed*speed)/(2.0f*accel_cmss);
    }
}
