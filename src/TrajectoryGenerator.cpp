#include "TrajectoryGenerator.h"

vector<vector<double>>
TrajectoryGenerator::generate(Car currState, Car goalState, vector<double> previous_path_x, vector<double> previous_path_y,
                              double end_path_s, double end_path_d) {
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double dist_inc = max_speed_dt;
  for(int i = 0; i < n_steps; i++) {
    vector<double> pos = world.getXY(currState.s + i * dist_inc, 6);
    next_x_vals.push_back(pos[0]);
    next_y_vals.push_back(pos[1]);
  }

  return {next_x_vals, next_y_vals};
}

vector<double> TrajectoryGenerator::JMT(vector<double> start, vector<double> end, double T) {
  double T2 = T * T;
  double T3 = T * T2;
  double T4 = T2 * T2;
  double T5 = T * T4;

  double a0 = start[0];
  double a1 = start[1];
  double a2 = start[2] / 2.;

  MatrixXd A(3, 3);
  A << T3, T4, T5,
      3*T2, 4*T3, 5*T4,
      6 * T, 12*T2, 20*T3;

  VectorXd b(3);
  b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T2),
      end[1] - (start[1] + start[2] * T),
      end[2] - start[2];


  VectorXd x = A.colPivHouseholderQr().solve(b);

  return {a0, a1, a2, x[0], x[1], x[2]};
}
