#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <math.h>
#include <iostream>

using namespace std;

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

  Car();
  explicit Car(int id);
  void updatePos(double s, double d, double x, double y);
  void updateVelocity(double s_dot, double d_dot);
  Car stateAt(double dt);

  int getLane();
  void display();
  double getLaneD();
};

#endif //PATH_PLANNING_CAR_H
