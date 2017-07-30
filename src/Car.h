#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <math.h>

const double LANE_WIDTH = 4;

class Car {
public:
  int id;
  double x;
  double y;

  double s;
  double s_dot;
  double s_ddot;
  double d;
  double d_dot;
  double d_ddot;

  double speed;
  double yaw;

  Car();
  explicit Car(int id);
  void updatePos(double s, double d, double x, double y);
  void updateSpeed(double vx, double vy);
  void updateYawAndSpeed(double yaw, double speed);

  int getLane();
  double getLaneD();
};

#endif //PATH_PLANNING_CAR_H
