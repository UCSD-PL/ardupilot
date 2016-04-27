#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Math/vector2.h>
#include <AP_Math/intersection.h>

#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library

#define RECOVERY_SPEED 50

class AC_Avoid
{
 public:

  /// Constructor
  AC_Avoid(const AP_InertialNav& inav);

  /*
   * Adjusts the desired velocity so that the vehicle can stop
   * before the fence/object.
   */
  void adjust_velocity(Vector2f &desired_vel);

 private:

  /*
   * Computes the speed such that the stopping distance
   * of the vehicle will be exactly the input distance.
   */
  float get_max_speed(float distance);

  const AP_InertialNav& _inav;
  Vector2f _boundary[4] = {
    Vector2f(-1000, -1000),
    Vector2f(1000, -1000),
    Vector2f(1000, 1000),
    Vector2f(-1000, 1000)
  };
  unsigned _nvert;
  Vector2f _inside_position;
  float _accel_cms;
  float _kP;

};
