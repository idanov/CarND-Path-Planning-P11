#include "TrajectoryGenerator.h"
#include "helpers.h"

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
