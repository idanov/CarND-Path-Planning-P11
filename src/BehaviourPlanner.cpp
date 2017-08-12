#include "BehaviourPlanner.h"

vector<Car> BehaviourPlanner::updatePlan(const Car& ego, const vector<vector<Car>>& predictions) {
  Car best_goal = generateGoal(target_lane, ego, predictions);
  vector<Car> best_path = generateTrajectory(ego, best_goal, n_steps_react);
  double best_cost = calculateCost(ego, best_goal, best_path, predictions);

  // Favour finishing previous decisions before coming up with new ideas
  if(ego.getLane() != target_lane) {
    return best_path;
  }

  for(size_t goal_lane = 0; goal_lane < n_lanes; goal_lane++) {
    long lanes_to_switch = abs(static_cast<long>(goal_lane) - static_cast<long>(ego.getLane()));
    if(lanes_to_switch < 2) {
      Car goal = generateGoal(goal_lane, ego, predictions);
      const vector<Car> path = generateTrajectory(ego, best_goal, n_steps_react);
      double cost = calculateCost(ego, goal, path, predictions);
      if(cost < best_cost) {
        best_cost = cost;
        best_goal = goal;
        best_path = path;
      }
    }
  }

  target_lane = best_goal.getLane();
  return best_path;
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

Car BehaviourPlanner::generateGoal(size_t goal_lane, Car start, const vector<vector<Car>> &predictions) const {
  Car goal(start);

  // Find leader car
  vector<Car> leader = findLeaderInLane(goal_lane, start.s, predictions);

  // Calculate best target position along s
  double target_s_dot = min(max_speed, start.s_dot + max_acc * time_horizon);
  double acc = (target_s_dot - start.s_dot) / time_horizon;
  double s_move = start.s_dot * time_horizon + 0.5 * acc * pow(time_horizon, 2);

  // If there is a leader car, add restriction to the target position if necessary
  if(!leader.empty()) {
    double goal_dist = circuitDiff(leader.back().s - fn_car_buffer(target_s_dot), start.s);
    if(goal_dist > 0 && goal_dist < s_move) {
      target_s_dot = leader.back().s_dot;
      s_move = goal_dist;
    }
  }

  // Calculate best target position along d
  double target_d = lane_d(start.getLane());
  double target_d_dot = 0;
  if(goal_lane != start.getLane()) {
    target_d = lane_d(goal_lane);
  }

  goal.s = circuit(start.s + s_move);
  goal.s_dot = target_s_dot;
  goal.s_ddot = 0;
  goal.d = target_d;
  goal.d_dot = target_d_dot;
  goal.d_ddot = 0.6 * max_acc * (start.d - target_d) / LANE_WIDTH;
  return goal;
}

double BehaviourPlanner::calculateCost(Car start, Car goal, const vector<Car>& trajectory, const vector<vector<Car>> &predictions) const {

  int crash = 0;
  for(vector<Car> other : predictions) {
    for(int i = 0; i < trajectory.size(); i++) {
      if(trajectory[i].crashWith(other[i + 1])) {
        crash = 1;
        break;
      }
    }
  }

  double dangerous = 0;
  for(vector<Car> other : predictions) {
    for(int i = 0; i < trajectory.size(); i++) {
      if(fabs(trajectory[i].d - other[i + 1].d) <= car_width) {
        double front_buffer = fn_car_buffer(trajectory[i + 1].s_dot);
        double back_buffer = fn_car_buffer(other[i + 1].s_dot);
        double dist = circuitDiff(trajectory[i + 1].s, other[i + 1].s);
        if(dist > 0 && dist <= front_buffer) {
          dangerous = max(dangerous, (front_buffer - dist) / front_buffer);
        }

        if(dist < 0 && fabs(dist) <= back_buffer) {
          dangerous = max(dangerous, (back_buffer - fabs(dist)) / back_buffer);
        }
      }
    }
  }

  double speed_cost = max((max_speed - goal.s_dot) / max_speed, 0.);
  int lane_switch = abs(static_cast<int>(start.getLane()) - static_cast<int>(goal.getLane()));
  auto plan_change_cost = static_cast<int>(goal.getLane() != target_lane);
  return crash * 10000 + dangerous * 5000 + plan_change_cost * 5 +  lane_switch * 10 + speed_cost * 150;
}

vector<Car> BehaviourPlanner::generateTrajectory(Car start, Car goal, size_t delay) const {
  // Limit the delay to the length of old_path_s
  vector<Car> path = followTrajectory(start, delay);
  size_t n_future_steps = n_steps - path.size();

  if(!path.empty()) start = path.back();

  Car curr;
  vector<vector<double>> tail = traj.generate(start, goal, n_future_steps);
  for(size_t i = 1; i <= n_future_steps; i++) {
    curr = start;
    curr.followTrajectory(tail[0], tail[1], i);
    path.emplace_back(curr);
  }

  return path;
}

vector<Car> BehaviourPlanner::followTrajectory(Car start, size_t steps) const {
  // Limit the delay to the length of old_path_s
  vector<vector<double>> head = traj.getTrajectory();
  size_t delay = min(traj.getTrajectoryLength(), steps);

  vector<Car> path;
  Car curr;
  // Move the car forward
  for(size_t i = 1; i <= delay; i++) {
    curr = start;
    curr.followTrajectory(head[0], head[1], i);
    path.emplace_back(curr);
  }

  return path;
}
