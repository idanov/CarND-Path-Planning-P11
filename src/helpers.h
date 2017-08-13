#ifndef PATH_PLANNING_HELPERS_H
#define PATH_PLANNING_HELPERS_H

#include <vector>
#include <math.h>
#include <iterator>

using namespace std;

const double LANE_WIDTH = 4;
const size_t n_lanes = 3;
const auto lane_d = [](size_t lane) {
  return lane * LANE_WIDTH + LANE_WIDTH / 2;
};

// Units are meters, seconds and steps (one step is dt time)
const double max_speed = 21.;
const double max_acc = 10.;
const double dt = .02;
const double reaction_time = 0.1;

const size_t n_steps = 75;
const size_t n_steps_react = static_cast<const size_t>(reaction_time / dt);
const double mph2ms = 0.44704;
// The desired car buffer depends on the speed
// In general, we want 2 car lengths (cars are crashing at 1 car length) +
// the distance our car will travel through the reaction time if
// the lead car is breaking with 3/4 of max deceleration and we are speeding up at 3/4 max acceleration
const double car_length = 5.4;
const double car_width = 3.6;
const auto fn_car_buffer = [](double v) {
  return 2 * car_length + v * reaction_time + 0.5 * 2 * max_acc * pow(reaction_time, 2);
};
const double time_horizon = n_steps * dt;

// The max s value before wrapping around the track back to 0
const double max_s = 6945.554;

inline double circuit(double s) {
  return ((s >= 0 && s < max_s) ? s : fmod(max_s + s, max_s));
}

inline double circuitDiff(double s1, double s2) {
  auto a = circuit(s1);
  auto b = circuit(s2);
  auto diff = a - b;
  if(diff < -max_s / 2) diff += max_s;
  if(diff > max_s / 2) diff -= max_s;
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

template<class T> ostream& operator<<(ostream& stream, const std::vector<T>& values) {
  streamsize init_precision = stream.precision();
  stream.precision(2);
  stream << "[ ";
  copy(std::begin(values), std::end(values), ostream_iterator<T>(stream<<fixed, " ") );
  stream << ']';
  stream.precision(init_precision);
  return stream;
}

#endif //PATH_PLANNING_HELPERS_H
