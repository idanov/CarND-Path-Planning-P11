#include "TrajectoryGenerator.h"
#include "helpers.h"

vector<vector<double>>
TrajectoryGenerator::generate(Car ego, vector<double> previous_path_x, vector<double> previous_path_y,
                              double end_path_s, double end_path_d) {
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double pos_x;
  double pos_y;
  double angle;
  int path_size = previous_path_x.size();

  for(int i = 0; i < path_size; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  if(path_size == 0)
  {
    pos_x = ego.x;
    pos_y = ego.y;
    angle = deg2rad(ego.yaw);
  }
  else
  {
    pos_x = previous_path_x[path_size-1];
    pos_y = previous_path_y[path_size-1];

    double pos_x2 = previous_path_x[path_size-2];
    double pos_y2 = previous_path_y[path_size-2];
    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
  }

  double dist_inc = max_speed_dt;
  for(int i = 0; i < n_steps - path_size; i++)
  {
    next_x_vals.push_back(pos_x+(dist_inc)*cos(angle));
    next_y_vals.push_back(pos_y+(dist_inc)*sin(angle));
    pos_x += (dist_inc)*cos(angle);
    pos_y += (dist_inc)*sin(angle);
  }

  return {next_x_vals, next_y_vals};
}
