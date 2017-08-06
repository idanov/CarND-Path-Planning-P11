#include "BehaviourPlanner.h"

BehaviourPlanner::BehaviourPlanner(string initial_state) {
  state = initial_state;
}

Car BehaviourPlanner::updatePlan(const Car& ego, const vector<vector<Car>>& predictions) {
  Car goal(ego);

  // Find leader car
  double leader_dist = max_s;
  double leader_idx = -1;
  for(int i = 0; i < predictions.size(); i++) {
    const vector<Car>& pred = predictions[i];
    if(ego.getLane() == pred[0].getLane()) {
      double dist = circuitDiff(pred[0].s, ego.s);
      if(dist < leader_dist && dist >= 0) {
        leader_dist = dist;
        leader_idx = i;
      }
    }
  }

  double car_max_speed = min(max_speed, ego.s_dot + max_acc * time_horizon);
  double s_move = min(max_speed * time_horizon, ego.s_dot * time_horizon + 0.5 * max_acc * pow(time_horizon, 2));
  if(leader_idx != -1) {
    const vector<Car>& pred = predictions[leader_idx];
    double goal_dist = circuitDiff(pred[pred.size() - 1].s - fn_car_buffer(car_max_speed), ego.s);
    if(goal_dist > 0 && goal_dist < s_move) {
      car_max_speed = pred[pred.size() - 1].s_dot;
      s_move = goal_dist;
    }
  }

  goal.s = circuit(ego.s + s_move);
  goal.s_dot = car_max_speed;
  goal.s_ddot = 0;
  goal.d = 6;
  goal.d_dot = 0;
  goal.d_ddot = 0;

  return goal;
}
