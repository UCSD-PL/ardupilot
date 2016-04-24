#include <AP_Math/AP_Math.h>
#include <AP_Math/vector2.h>

#define RECOVERY_VELOCITY 500

class AC_Avoid
{
 public:

  /*
   * Adjusts the desired velocity so that the vehicle can stop
   * before the fence/object.
   */
  void adjust_velocity(Vector2f &desired_vel, const Vector2f position);

 private:

  /*
   * Compute the nearest intersection point of the
   * boundary with the ray starting from the input position
   * and extending in the direction of the input direction.
   */
  unsigned boundary_intersection(Vector2f position, Vector2f direction, Vector2f &intersect);

  /*
   * Compute the intersection between a ray (p,p + r) and a line segment (q, q + s).
   * Returns false if they do not intersect. Returns false if they are collinear.
   * Returns true otherwise.
   *
   * Implementation taken from
   * http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
   */
  static bool intersection(Vector2f p, Vector2f r, Vector2f q, Vector2f s, Vector2f &intersect);

  /*
   * Computes the speed such that the stopping distance
   * of the vehicle will be exactly the input distance.
   */
  float get_max_speed(float distance);

  Vector2f* _boundary;
  unsigned _nvert;
  Vector2f _inside_position;
  float _accel_cms;
  float _kP;

};
