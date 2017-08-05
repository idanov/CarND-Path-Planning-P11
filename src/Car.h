#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <math.h>
#include <sstream>
#include <iostream>
#include <iomanip>

using namespace std;

const double LANE_WIDTH = 4;

class Car {
public:
  int id;

  double s;
  double s_dot;
  double s_ddot;
  double d;
  double d_dot;
  double d_ddot;

  Car();
  explicit Car(int id);
  void updatePos(double s, double d);
  void updateVelocity(double s_dot, double d_dot);
  Car stateAt(double dt);

  int getLane() const;
  string display() const;
  double getLaneD() const;
};

#endif //PATH_PLANNING_CAR_H
