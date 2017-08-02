#ifndef PATH_PLANNING_TRAJECTORYGENERATOR_H
#define PATH_PLANNING_TRAJECTORYGENERATOR_H

#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "Car.h"
#include "Map.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class TrajectoryGenerator {
  Map& world;
  vector<double> old_path_x;
  vector<double> old_path_y;
  vector<double> old_path_s;
  vector<double> old_path_d;
  std::function<double (double)> JMT(vector< double> start, vector <double> end, double T);
  Car addDelay(Car currState, size_t past, size_t steps = 3);
public:
  explicit TrajectoryGenerator(Map& world_): world(world_) {}
  vector<double> getLastVelocity(size_t prev_path_size);
  vector<vector<double>> generate(Car currState, Car goalState, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d);
};


#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
