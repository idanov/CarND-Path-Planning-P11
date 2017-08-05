#include "TrajectoryGenerator.h"

vector<vector<double>>
TrajectoryGenerator::generate(Car currState, Car goalState) {

  size_t delay = 0;
  if(!old_path_s.empty()) {
    delay = min(old_path_s.size(), n_steps_react);
    old_path_s.resize(delay);
    old_path_d.resize(delay);
    old_path_x.resize(delay);
    old_path_y.resize(delay);
    updateCar(currState, currState.s, currState.d, currState.s_dot, currState.d_dot, delay);
  }

  size_t n_future_steps = n_steps - delay;
  const function<double(double)> &fn_s = JMT({currState.s, currState.s_dot, currState.s_ddot}, {goalState.s, goalState.s_dot, 0}, n_future_steps * dt);
  const function<double(double)> &fn_d = JMT({currState.d, currState.d_dot, currState.d_ddot}, {goalState.d, 0, 0}, n_future_steps * dt);
  double prev_s = fn_s(0);
  for(int i = 1; i <= n_future_steps; i++) {
    // This is required to ensure we are advancing at each step to prevent buggy behaviour
    double next_s = max(fn_s(i * dt), prev_s + 0.002);
    old_path_s.push_back(next_s);
    old_path_d.push_back(fn_d(i * dt));
    prev_s = next_s;
  }

  for(int i = 0; i < n_future_steps; i++) {
    const vector<double> &pos = world.getXY(old_path_s[i], old_path_d[i]);
    old_path_x.push_back(pos[0]);
    old_path_y.push_back(pos[1]);
  }

  return {old_path_x, old_path_y};
}

std::function<double (double)> TrajectoryGenerator::JMT(vector<double> start, vector<double> end, double T) {
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

void TrajectoryGenerator::updateCar(Car& ego, double default_s, double default_d, double default_s_dot, double default_d_dot, size_t delay) {
  long idx = delay - 1;
  if(idx > 0) {
    double s_dot = circuitDiff(old_path_s[idx], old_path_s[idx - 1]) / dt;
    double d_dot = (old_path_d[idx] - old_path_d[idx - 1]) / dt;
    ego.s_ddot = (s_dot - ego.s_dot) / ((idx + 1) * dt);
    ego.d_ddot = (d_dot - ego.d_dot) / ((idx + 1) * dt);
    ego.updatePos(old_path_s[idx], old_path_d[idx]);
    ego.updateVelocity(s_dot, d_dot);
  } else if(idx == 0) {
    double s_dot = circuitDiff(old_path_s[idx], ego.s) / dt;
    double d_dot = (old_path_d[idx] - ego.d) / dt;
    ego.s_ddot = (s_dot - ego.s_dot) / dt;
    ego.d_ddot = (d_dot - ego.d_dot) / dt;
    ego.updatePos(old_path_s[idx], old_path_d[idx]);
    ego.updateVelocity(s_dot, d_dot);
  } else {
    // If we don't have any historical data, we fallback to default values
    ego.updatePos(default_s, default_d);
    ego.updateVelocity(default_s_dot, default_d_dot);
  }
}

void TrajectoryGenerator::refreshPreviousPath(vector<double> previous_path_x, vector<double> previous_path_y) {
  old_path_x.clear();
  old_path_y.clear();
  old_path_x = std::move(previous_path_x);
  old_path_y = std::move(previous_path_y);
  size_t past = old_path_s.size() - old_path_x.size();
  if(!old_path_x.empty() && past > 0) {
    old_path_s.erase(old_path_s.begin(), min(old_path_s.begin() + past, old_path_s.end()));
    old_path_d.erase(old_path_d.begin(), min(old_path_d.begin() + past, old_path_d.end()));
  } else if(old_path_x.empty()) {
    old_path_s.clear();
    old_path_d.clear();
  }
}

size_t TrajectoryGenerator::getHistoryLen() const {
  return old_path_s.size();
}
