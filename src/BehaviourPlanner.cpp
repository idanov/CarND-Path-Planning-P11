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


  double s_move = max_speed * dt * n_steps;
  double car_max_speed = max_speed;
  if(leader_idx != -1) {
    const vector<Car>& pred = predictions[leader_idx];
    car_max_speed = pred[pred.size() - 1].s_dot;
    s_move = min(s_move, circuitDiff(pred[pred.size() - 1].s - fn_car_buffer(car_max_speed), ego.s));
  }

  goal.s = circuit(goal.s + s_move);
  goal.s_dot = car_max_speed;
  goal.s_ddot = 0;
  goal.d = 6;
  goal.d_dot = 0;
  goal.d_ddot = 0;

  if(leader_idx != -1) {
    const vector<Car>& pred = predictions[leader_idx];
    cout<<"=================="<<endl;
    cout<<circuitDiff(pred[0].s, ego.s)<<","<<circuitDiff(pred[pred.size() - 1].s, goal.s)<<endl;
    cout<<s_move<<","<<car_max_speed<<endl;
    cout<<ego.display()<<" --> "<<goal.display()<<endl;
    cout<<pred[0].display()<<" --> "<<pred[pred.size() - 1].display()<<endl;
    cout<<"=================="<<endl;
  }

  return goal;
}
