#include "AC_Avoid.h"

/*
 * Adjusts the desired velocity so that the vehicle can stop
 * before the fence/object.
 */
void AC_Avoid::adjust_velocity(Vector2f &desired_vel, const Vector2f position) {
  Vector2f intersect;
  unsigned num_intersects = boundary_intersection(position, desired_vel, intersect);
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
    desired_vel = (_inside_position - position).normalized()*RECOVERY_VELOCITY;
  }
}

/*
 * Compute the nearest intersection point of the
 * boundary with the ray starting from the input position
 * and extending in the direction of the input direction.
 */
unsigned AC_Avoid::boundary_intersection(Vector2f position, Vector2f direction, Vector2f &intersect) {
  unsigned i, j;
  unsigned num_intersects = 0;
  float distance = FLT_MAX;
  for (i = 0, j = _nvert-1; i < _nvert; j = i++) {
    Vector2f start = _boundary[j];
    Vector2f end = _boundary[i];
    if ((end - start) % (position - start) > 0) {
      // lies to the inside plane of the current boundary segment
      Vector2f next_intersect;
      if (intersection(position, direction, start, end - start, next_intersect)) {
	float next_distance = (next_intersect - position).length();
	num_intersects++;
	if (next_distance <= distance) {
	  intersect = next_intersect;
	  distance = next_distance;
	}
      }
    }
  }

  return num_intersects;
}

/*
 * Compute the intersection between a ray (p,p + r) and a line segment (q, q + s).
 * Returns false if they do not intersect. Returns false if they are collinear.
 * Returns true otherwise.
 *
 * Implementation taken from
 * http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
 */
static bool intersection(Vector2f p, Vector2f r, Vector2f q, Vector2f s, Vector2f &intersect) {
  // if input vectors are Vector2l, will they be cast to floats? Need to rectify Vector2l and Vector2f
  Vector2f q_p = q - p;
  float rxs = r%s;
  float q_pxr = q_p % r;
  if (rxs == 0) {
    // either collinear or parallel and non-intersecting
    return false;
    // if (q_pxr == 0) {
    //   // collinear
      
    // } else {
    //   // parallel and non-intersecting
    //   return null;
    // }
  } else {
    // t = (q − p) × s / (r × s)
    // u = (q − p) × r / (r × s)
    float t = (q_p % s) / rxs;
    float u = q_pxr / rxs;
    if (0 <= u && u <= 1 && 0 <= t) {
      // lines intersect
      // t can be any non-negative value because (p, p + r) is a ray
      // u must be between 0 and 1 because (q, q + s) is a line segment
      intersect = p + (r*t);
      return true;
    } else {
      // non-parallel and non-intersecting
      return false;
    }
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
