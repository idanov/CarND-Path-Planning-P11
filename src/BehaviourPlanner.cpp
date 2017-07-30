#include "BehaviourPlanner.h"

BehaviourPlanner::BehaviourPlanner(string initial_state) {
  state = initial_state;
}

Car BehaviourPlanner::updatePlan(Car ego, const vector<vector<Car>>& predictions) {
  int lane = ego.getLane();
  vector<Car> leader_path;
  double leader_dist = 100000;
  for(vector<Car> v_path : predictions) {
    if(v_path[0].getLane() == lane && v_path[0].s > ego.s) {
      if(v_path[0].s - ego.s < leader_dist) {
        leader_path = v_path;
        leader_dist = leader_path[0].s - ego.s;
      }
    }
  }

  if(!leader_path.empty()) {
    double t = dt * leader_path.size();
    Car leaderEnd = *leader_path.end();
    ego.s = min(leaderEnd.s - car_buffer, ego.s + ego.s_dot * t);
    ego.s_dot = min(max_speed, leaderEnd.s_dot);
    ego.s_ddot = 0;
    ego.d = ego.getLane() * LANE_WIDTH + 2;
    ego.d_dot = 0;
    ego.d_ddot = 0;
  } else {
    double t = dt * n_steps;
    ego.s = ego.s + ego.s_dot * t;
    ego.s_dot = max_speed;
    ego.s_ddot = 0;
    ego.d = ego.getLane() * LANE_WIDTH + 2;
    ego.d_dot = 0;
    ego.d_ddot = 0;
  }

  return ego;
}
