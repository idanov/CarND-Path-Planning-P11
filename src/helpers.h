#ifndef PATH_PLANNING_HELPERS_H
#define PATH_PLANNING_HELPERS_H

#include <vector>
#include <math.h>

using namespace std;

// Max speed in m/s
const double max_speed = 22.;
const double dt = .02;
const double max_speed_dt = max_speed * dt;
const int n_steps = 50;
const double mph2ms = 0.44704;
const double car_buffer = 15;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

inline double deg2rad(double x) { return x * pi() / 180; }

inline double rad2deg(double x) { return x * 180 / pi(); }

inline double euclidean(double dx, double dy) {
  return sqrt(dx * dx + dy * dy);
}

inline double distance(double x1, double y1, double x2, double y2);

inline double distance(double x1, double y1, double x2, double y2) {
  return euclidean(x2 - x1, y2 - y1);
}

inline vector<double> cartesian2polar(double vx, double vy) {
  double speed = euclidean(vx, vy);
  double theta = atan2(vy, vx);
  if(theta < 0) theta += 2 * M_PI;
  return {speed, theta};
}

#endif //PATH_PLANNING_HELPERS_H
