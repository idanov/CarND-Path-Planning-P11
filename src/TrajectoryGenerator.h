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
  vector<double> old_path_s;
  vector<double> old_path_d;
  std::function<double (double)> JMT(vector< double> start, vector <double> end, double T) const;
public:
  explicit TrajectoryGenerator(Map& world_): world(world_) {}
  void refreshPreviousPath(size_t prev_path_length);
  vector<vector<double>> generate(Car currState, Car goalState) const;
  vector<vector<double>> updateTrajectory(Car currState, Car goalState);
  size_t getTrajectoryLength() const;
  const vector<vector<double>> getTrajectory() const;
};


#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
