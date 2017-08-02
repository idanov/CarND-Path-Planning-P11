#include "BehaviourPlanner.h"

BehaviourPlanner::BehaviourPlanner(string initial_state) {
  state = initial_state;
}

Car BehaviourPlanner::updatePlan(Car ego, const vector<vector<Car>>& predictions) {
  Car goal(ego);

  goal.s += max_speed;
  goal.s_dot = max_speed;
  goal.s_ddot = 0;
  goal.d = 6;
  goal.d_dot = 0;
  goal.d_ddot = 0;

  return goal;
}
