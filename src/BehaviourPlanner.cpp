#include "BehaviourPlanner.h"

BehaviourPlanner::BehaviourPlanner(string initial_state) {
  state = initial_state;
}

void BehaviourPlanner::update_state(Car ego, map<int, vector<Car>> predictions) {

}

Car BehaviourPlanner::getGoal(Car ego, map<int, vector<Car>> predictions) {
  int lane = ego.getLane();
  vector<Car> leader_path;
  double leader_dist = 100000;
  for(pair<int, vector<Car>> it : predictions) {
    vector<Car> v_path = it.second;
    if(v_path[0].getLane() == lane && v_path[0].s > ego.s) {
      if(v_path[0].s - ego.s < leader_dist) {
        leader_path = v_path;
        leader_dist = leader_path[0].s - ego.s;
      }
    }
  }

  double t = 1;
  if(!leader_path.empty()) {
    ego.s = min(leader_path.end()->s - 10, ego.s + ego.speed * t);
    ego.speed = leader_path.end()->speed;
  } else {
    ego.s = ego.s + ego.speed * t;
    ego.speed = 21;
  }

  return ego;
}
