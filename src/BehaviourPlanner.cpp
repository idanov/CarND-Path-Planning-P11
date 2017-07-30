#include "BehaviourPlanner.h"

BehaviourPlanner::BehaviourPlanner(string initial_state) {
  state = initial_state;
}

Car BehaviourPlanner::updatePlan(Car ego, vector<vector<Car>> predictions) {
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

  double t = 1;
  if(!leader_path.empty()) {
    ego.s = min(leader_path.end()->s - 10, ego.s + ego.s_dot * t);
    ego.s_dot = leader_path.end()->s_dot;
  } else {
    ego.s = ego.s + ego.s_dot * t;
    ego.s_dot = 21;
  }

  return ego;
}
