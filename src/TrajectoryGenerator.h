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
public:
  explicit TrajectoryGenerator(Map& world_): world(world_) {}
  void refreshPreviousPath(vector<double> previous_path_x, vector<double> previous_path_y);
  void updateCar(Car& ego, double default_s, double default_d, double default_s_dot, double default_d_dot, size_t delay);
  vector<vector<double>> generate(Car currState, Car goalState);
  size_t getHistoryLen() const;
};


#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
