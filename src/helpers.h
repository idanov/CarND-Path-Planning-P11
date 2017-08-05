#ifndef PATH_PLANNING_HELPERS_H
#define PATH_PLANNING_HELPERS_H

#include <vector>
#include <math.h>

using namespace std;

// Units are meters, seconds and steps (one step is dt time)
const double max_speed = 22.;
const double min_speed = 2.;
const double max_acc = 10.;
const double dt = .02;
const double reaction_time = 0.4;

const size_t n_steps = 100;
const size_t n_steps_react = static_cast<const size_t>(reaction_time / dt);
const double mph2ms = 0.44704;
// a delay of 0.2 at max speed is ~5m, cars are ~5m in length, ~15m buffer should be fine
const double car_buffer = 15;
const double time_horizon = n_steps * dt;

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

inline double circuit(double s) {
  return ((s >= 0 && s < max_s) ? s : fmod(max_s + s, max_s));
}

inline double circuitDiff(double s1, double s2) {
  auto a = circuit(s1) + max_s;
  auto b = circuit(s2);
  auto diff = circuit(a - b);
  return diff;
}

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
