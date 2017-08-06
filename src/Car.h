#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <math.h>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <vector>

using namespace std;

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
  void followTrajectory(const vector<double>& path_s, const vector<double>& path_d, size_t steps);
  Car stateAt(double dt) const;
  size_t getLane() const;
  friend std::ostream& operator<<(std::ostream& stream, const Car& matrix);
  void display() const;
  double getLaneD() const;
};

#endif //PATH_PLANNING_CAR_H
