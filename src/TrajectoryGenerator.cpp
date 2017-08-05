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
    followTrajectory(currState, delay);
  }

  size_t n_future_steps = n_steps - delay;
  const function<double(double)> &fn_s = JMT({currState.s, currState.s_dot, 0}, {goalState.s, goalState.s_dot, 0}, n_future_steps * dt);
  const function<double(double)> &fn_d = JMT({currState.d, currState.d_dot, 0}, {goalState.d, 0, 0}, n_future_steps * dt);
  for(int i = 1; i <= n_future_steps; i++) {
    old_path_s.push_back(fn_s(i * dt));
    old_path_d.push_back(fn_d(i * dt));
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

void TrajectoryGenerator::updateCar(Car& ego, double sim_s, double sim_d, double sim_speed, double sim_yaw, size_t path_len) {
  long min_idx = old_path_s.size() - path_len - 1;

  if(min_idx > 0) {
    double s_dot = circuitDiff(old_path_s[min_idx], old_path_s[min_idx - 1]) / dt;
    double d_dot = old_path_d[min_idx] - old_path_d[min_idx - 1];
    ego.updatePos(old_path_s[min_idx], old_path_d[min_idx]);
    ego.updateVelocity(s_dot, d_dot);
  } if(min_idx == 0) {
    double s_dot = circuitDiff(old_path_s[min_idx], ego.s) / dt;
    double d_dot = old_path_d[min_idx] - ego.d;
    ego.updatePos(old_path_s[min_idx], old_path_d[min_idx]);
    ego.updateVelocity(s_dot, d_dot);
  } else {
    // If we don't have any historical data, we fallback to simulator data
    const vector<double> &frenetVel = world.getFrenetVelocity(sim_s, sim_d, mph2ms * sim_speed, deg2rad(sim_yaw));
    ego.updatePos(sim_s, sim_d);
    ego.updateVelocity(frenetVel[0], frenetVel[1]);
  }
}

void TrajectoryGenerator::followTrajectory(Car& ego, size_t horizon) {
  horizon = min(old_path_s.size(), horizon) - 1;
  if(horizon > 0) {
    double s_dot = circuitDiff(old_path_s[horizon], old_path_s[horizon - 1]) / dt;
    double d_dot = old_path_d[horizon] - old_path_d[horizon - 1];
    ego.updatePos(old_path_s[horizon], old_path_d[horizon]);
    ego.updateVelocity(s_dot, d_dot);
  } else if(horizon == 0) {
    double s_dot = circuitDiff(old_path_s[horizon], ego.s) / dt;
    double d_dot = old_path_d[horizon] - ego.d;
    ego.updatePos(old_path_s[horizon], old_path_d[horizon]);
    ego.updateVelocity(s_dot, d_dot);
  }
}

void TrajectoryGenerator::refreshPreviousPath(vector<double> previous_path_x, vector<double> previous_path_y) {
  old_path_x.clear();
  old_path_y.clear();
  old_path_x = std::move(previous_path_x);
  old_path_y = std::move(previous_path_y);
  size_t past = old_path_s.size() - old_path_x.size();
  if(!old_path_x.empty() && past > 0) {
    old_path_s.erase(old_path_s.begin(), old_path_s.begin() + past);
    old_path_d.erase(old_path_d.begin(), old_path_d.begin() + past);
  } else {
    old_path_s.clear();
    old_path_d.clear();
  }
}
