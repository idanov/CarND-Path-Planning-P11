#include "TrajectoryGenerator.h"

vector<vector<double>>
TrajectoryGenerator::generate(Car currState, Car goalState) const {
  // Add a reaction delay
  size_t delay = min(old_path_s.size(), n_steps_react);

  // Copy the path for the delay
  vector<double> next_path_s(old_path_s.begin(), old_path_s.begin() + delay);
  vector<double> next_path_d(old_path_d.begin(), old_path_d.begin() + delay);
  // Move the car along the path to simulate the delay
  currState.followTrajectory(next_path_s, next_path_d, delay);

  // Generate curves for the future
  size_t n_future_steps = n_steps - delay;
  const function<double(double)> &fn_s = JMT({currState.s, currState.s_dot, currState.s_ddot}, {goalState.s, goalState.s_dot, 0}, n_future_steps * dt);
  const function<double(double)> &fn_d = JMT({currState.d, currState.d_dot, currState.d_ddot}, {goalState.d, 0, 0}, n_future_steps * dt);

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

const vector<vector<double>> TrajectoryGenerator::getTrajectory() const {
  return {old_path_s, old_path_d};
}

vector<vector<double>> TrajectoryGenerator::updateTrajectory(Car currState, Car goalState) {
  // Generate the final path
  vector<vector<double>> path = generate(currState, goalState);
  // Save it for the next cycle
  old_path_s = path[0];
  old_path_d = path[1];

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
