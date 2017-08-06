#include "BehaviourPlanner.h"

Car BehaviourPlanner::updatePlan(const Car& ego, const vector<vector<Car>>& predictions) {
  Car best_goal = generateGoal(target_lane, ego, predictions);
  double best_cost = calculateCost(ego, best_goal, predictions);

  for(size_t goal_lane = 0; goal_lane < n_lanes; goal_lane++) {
    long lanes_to_switch = abs(static_cast<long>(goal_lane) - static_cast<long>(ego.getLane()));
    if(lanes_to_switch < 2) {
      Car goal = generateGoal(goal_lane, ego, predictions);
      double cost = calculateCost(ego, goal, predictions);
      if(cost < best_cost) {
        best_cost = cost;
        best_goal = goal;
      }
    }
  }

  target_lane = best_goal.getLane();
  return best_goal;
}

vector<Car> BehaviourPlanner::findLeaderInLane(size_t lane, double s, const vector<vector<Car>> &predictions) const {
  // Find leader car
  double leader_dist = max_s;
  vector<Car> leader;
  for (const vector<Car> &pred : predictions) {
    if(lane == pred[0].getLane()) {
      double dist = circuitDiff(pred[0].s, s);
      if(dist < leader_dist && dist >= 0) {
        leader_dist = dist;
        leader = pred;
      }
    }
  }

  return leader;
}

Car BehaviourPlanner::generateGoal(size_t goal_lane, Car ego, const vector<vector<Car>> &predictions) const {
  Car goal(ego);

  // Find leader car
  vector<Car> leader = findLeaderInLane(goal_lane, ego.s, predictions);

  // Calculate best target position along s
  double target_s_dot = min(max_speed, ego.s_dot + max_acc * time_horizon);
  double s_move = min(max_speed * time_horizon, ego.s_dot * time_horizon + 0.5 * max_acc * pow(time_horizon, 2));

  // If there is a leader car, add restriction to the target position if necessary
  if(!leader.empty()) {
    double goal_dist = circuitDiff(leader.back().s - fn_car_buffer(target_s_dot), ego.s);
    if(goal_dist > 0 && goal_dist < s_move) {
      target_s_dot = leader.back().s_dot;
      s_move = goal_dist;
    }
  }

  // Calculate best target position along d
  double target_d = lane_d(ego.getLane());
  double target_d_dot = 0;
  if(goal_lane != ego.getLane()) {
    target_d = lane_d(goal_lane);
  }

  goal.s = circuit(ego.s + s_move);
  goal.s_dot = target_s_dot;
  goal.s_ddot = 0;
  goal.d = target_d;
  goal.d_dot = target_d_dot;
  goal.d_ddot = 0;
  return goal;
}

double BehaviourPlanner::calculateCost(Car ego, Car goal, const vector<vector<Car>> &predictions) const {
  double lane_switch = abs(static_cast<double>(ego.getLane()) - static_cast<double>(goal.getLane()));
  double speed_cost = max((max_speed - goal.s_dot) / max_speed, 0.);
  auto plan_change_cost = static_cast<double>(goal.getLane() != target_lane);
  return plan_change_cost * 5 +  lane_switch * 10 + speed_cost * 150;
}
