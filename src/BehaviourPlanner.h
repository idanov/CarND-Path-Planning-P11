#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "Car.h"

using namespace std;

class BehaviourPlanner {
  string state;
public:
  explicit BehaviourPlanner(string initial_state);
  void update_state(Car ego, map<int, vector<Car>> predictions);
  Car getGoal(Car ego, map<int, vector<Car>> predictions);
};

#endif //PATH_PLANNING_PLANNER_H
