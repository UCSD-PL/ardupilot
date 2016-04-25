#include <AP_Math/AP_Math.h>
#include <AP_Math/vector2.h>
#include <AP_Math/intersection.h>

#define RECOVERY_SPEED 500

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
