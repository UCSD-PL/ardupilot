class AC_Avoid
{
 public:

  /*
   * Adjusts the desired velocity so that the vehicle can stop
   * before the fence/object.
   */
  void adjust_velocity(Vector2f &desired_vel);

 private:

  /*
   * Compute the nearest intersection point of the
   * boundary with the ray starting from the input position
   * and extending in the direction of the input direction.
   */
  Vector2f calc_intersection(Vector2f position, Vector2f direction);

  /*
   * Computes the speed such that the stopping distance
   * of the vehicle will be exactly the input distance.
   */
  static float get_max_speed(float distance);

}
