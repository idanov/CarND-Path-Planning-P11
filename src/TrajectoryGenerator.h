#ifndef PATH_PLANNING_TRAJECTORYGENERATOR_H
#define PATH_PLANNING_TRAJECTORYGENERATOR_H

#include <iostream>
#include <string>
#include <vector>
#include "Car.h"
#include "Map.h"

using namespace std;

class TrajectoryGenerator {
  Map& world;
public:
  explicit TrajectoryGenerator(Map& world_): world(world_) {}
  vector<vector<double>> generate(Car currState, Car goalState, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);
};


#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
