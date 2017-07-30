#ifndef PATH_PLANNING_TRAJECTORYGENERATOR_H
#define PATH_PLANNING_TRAJECTORYGENERATOR_H

#include <iostream>
#include <string>
#include <vector>
#include "Car.h"
#include "Map.h"

using namespace std;

// Max speed in m/s
const double max_speed = 22.;
const double dt = .02;
const double max_speed_dt = max_speed * dt;
const int n_steps = 50;

class TrajectoryGenerator {
  const Map& world;
public:
  explicit TrajectoryGenerator(const Map& world_): world(world_) {}
  vector<vector<double>> generate(Car ego, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);
};


#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
