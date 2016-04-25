#include "AP_Math.h"
#include "vector2.h"

/*
 * Compute the nearest intersection point of the
 * boundary with the ray starting from the input position
 * and extending in the direction of the input direction.
 *
 * Returns the number of intersections.
 */
unsigned boundary_intersection(Vector2f position, Vector2f direction,
			       Vector2f *boundary, unsigned nvert, Vector2f &intersect);

/*
 * Compute the intersection between a ray (p,p + r) and a line segment (q, q + s).
 * Returns false if they do not intersect. Returns false if they are collinear.
 * Returns true otherwise.
 *
 * Implementation taken from
 * http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
 */
bool intersection(Vector2f p, Vector2f r, Vector2f q, Vector2f s, Vector2f &intersect);
