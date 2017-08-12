#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <map>
#include "helpers.h"
#include "Car.h"
#include "TrajectoryGenerator.h"

using namespace std;

class BehaviourPlanner {
  size_t target_lane;
  const TrajectoryGenerator& traj;
  Car generateGoal(size_t goal_lane, Car start, const vector<vector<Car>> &predictions) const;
  vector<Car> findLeaderInLane(size_t lane, double s, const vector<vector<Car>> &predictions) const;
  vector<Car> generateTrajectory(Car start, Car goal, size_t delay) const;
  vector<Car> followTrajectory(Car start, size_t steps) const;
  double calculateCost(Car start, Car goal, const vector<Car>& trajectory, const vector<vector<Car>> &predictions) const;
public:
  explicit BehaviourPlanner(const TrajectoryGenerator& traj_, size_t initial_lane): traj(traj_), target_lane(initial_lane) {};
  vector<Car> updatePlan(const Car& ego, const vector<vector<Car>>& predictions);
};

#endif //PATH_PLANNING_PLANNER_H
