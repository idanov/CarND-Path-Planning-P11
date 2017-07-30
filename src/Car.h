#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <math.h>

class Car {
public:
  int id;
  double x;
  double y;

  double s;
  double d;
  int lane;
  double speed;
  double yaw;

  Car();
  explicit Car(int id);
  void updatePos(double s, double d, double x, double y);
  void updateSpeed(double vx, double vy);
  void updateYawAndSpeed(double yaw, double speed);
};


#endif //PATH_PLANNING_CAR_H
