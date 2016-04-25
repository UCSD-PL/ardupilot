#include "intersection.h"

/*
 * Compute the nearest intersection point of the
 * polygon with the ray starting from the input position
 * and extending in the direction of the input direction.
 *
 * Returns the number of intersections.
 */
unsigned poly_intersection(Vector2f position, Vector2f direction, Vector2f *poly, unsigned nvert, Vector2f &intersect) {
  unsigned i, j;
  unsigned num_intersects = 0;
  float distance = FLT_MAX;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    Vector2f start = poly[j];
    Vector2f end = poly[i];
    if ((end - start) % (position - start) > 0) {
      // lies to the inside plane of the current poly segment
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
bool intersection(Vector2f p, Vector2f r, Vector2f q, Vector2f s, Vector2f &intersect) {
  // if input vectors are Vector2l, will they be cast to floats? Need to rectify Vector2l and Vector2f
  Vector2f q_p = q - p;
  float rxs = r%s;
  float q_pxr = q_p % r;
  if (rxs == 0) {
    // either collinear or parallel and non-intersecting
    return false;
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
