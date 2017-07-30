#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "helpers.h"
#include "Car.h"

using namespace std;

class BehaviourPlanner {
  string state;
public:
  explicit BehaviourPlanner(string initial_state);
  Car updatePlan(Car ego, const vector<vector<Car>>& predictions);
};

#endif //PATH_PLANNING_PLANNER_H
