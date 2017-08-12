#include "TrajectoryGenerator.h"

vector<vector<double>>
TrajectoryGenerator::generate(Car currState, Car goalState, size_t n_future_steps) const {
  // New path
  vector<double> next_path_s;
  vector<double> next_path_d;

  // Correction for the reset of s coordinates
  if(fabs(currState.s - goalState.s) > max_s / 2) {
    goalState.s += max_s;
  }

  // Speed correction for the road curvature
  auto start = world.getXY(currState.s - 25, currState.d);
  auto end = world.getXY(currState.s + 25, currState.d);
  double dist = distance(start[0], start[1], end[0], end[1]);
  double s_dist = circuitDiff(currState.s + 25, currState.s - 25);
  double ratio = s_dist / dist;
  goalState.s_dot *= ratio;
  goalState.s = (goalState.s - currState.s) * ratio + currState.s;

  // Generate curves for the future
  const function<double(double)> &fn_s = JMT({currState.s, currState.s_dot, currState.s_ddot}, {goalState.s, goalState.s_dot, goalState.s_ddot}, n_future_steps * dt);
  const function<double(double)> &fn_d = JMT({currState.d, currState.d_dot, currState.d_ddot}, {goalState.d, goalState.d_dot, goalState.d_ddot}, n_future_steps * dt);

  // Append the future path with the delayed in Frenet
  double prev_s = currState.s;
  for(int i = 1; i <= n_future_steps; i++) {
    // This is required to ensure we are advancing at each step to prevent buggy behaviour
    double next_s = max(fn_s(i * dt), prev_s + 0.02);
    next_path_s.push_back(next_s);
    next_path_d.push_back(fn_d(i * dt));
    prev_s = next_s;
  }

  return {next_path_s, next_path_d};
}

std::function<double (double)> TrajectoryGenerator::JMT(vector<double> start, vector<double> end, double T) const {
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
      3*T*T, 4*T*T*T,5*T*T*T*T,
      6*T, 12*T*T, 20*T*T*T;

  MatrixXd B = MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
      end[1]-(start[1]+start[2]*T),
      end[2]-start[2];

  VectorXd C = A.inverse() * B;

  vector <double> coeff = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++) {
    coeff.push_back(C.data()[i]);
  }

  return [coeff](double t) {
    double res = 0;
    for(int i = 0; i < coeff.size(); i++) {
      res += coeff[i] * pow(t, i);
    }
    return res;
  };
}

void TrajectoryGenerator::refreshPreviousPath(size_t prev_path_length) {
  size_t past = old_path_s.size() - prev_path_length;
  if(prev_path_length > 0 && past > 0) {
    old_path_s.erase(old_path_s.begin(), old_path_s.begin() + past);
    old_path_d.erase(old_path_d.begin(), old_path_d.begin() + past);
  } else if(prev_path_length == 0) {
    old_path_s.clear();
    old_path_d.clear();
  }
}

size_t TrajectoryGenerator::getTrajectoryLength() const {
  return old_path_s.size();
}

vector<vector<double>> TrajectoryGenerator::getTrajectory() const {
  return {old_path_s, old_path_d};
}

vector<vector<double>> TrajectoryGenerator::updateTrajectory(const vector<Car>& plan) {
  // Recreate the paths
  old_path_s.clear();
  old_path_d.clear();

  for(const Car& step : plan) {
    old_path_s.push_back(step.s);
    old_path_d.push_back(step.d);
  }

  // Convert to Cartesian
  vector<double> next_path_x;
  vector<double> next_path_y;
  for(int i = 0; i < getTrajectoryLength(); i++) {
    const vector<double> &pos = world.getXY(old_path_s[i], old_path_d[i]);
    next_path_x.push_back(pos[0]);
    next_path_y.push_back(pos[1]);
  }

  return {next_path_x, next_path_y};
}
