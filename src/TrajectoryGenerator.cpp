#include "TrajectoryGenerator.h"

vector<vector<double>>
TrajectoryGenerator::generate(Car currState, Car goalState, vector<double> previous_path_x, vector<double> previous_path_y,
                              double end_path_s, double end_path_d) {
  const size_t steps = 3;
  const size_t past = old_path_s.size() - previous_path_x.size();
  currState = addDelay(currState, past, steps);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  cout<<"==="<<endl;
  currState.display();
  goalState.display();
  cout<<"==="<<endl;

  cout<<"Size: "<<old_path_s.size()<<endl;
  if(old_path_s.size() > past + steps) {
    old_path_s.erase(old_path_s.begin(), old_path_s.begin() + past);
    old_path_d.erase(old_path_d.begin(), old_path_d.begin() + past);
    cout<<"Size: "<<old_path_s.size()<<endl;
    old_path_s.resize(steps);
    old_path_d.resize(steps);
    cout<<"Size: "<<old_path_s.size()<<endl;
    cout<<"cccccc "<<old_path_s[0]<<" "<<currState.s<<endl;
  } else {
    old_path_s.clear();
    old_path_d.clear();
  }

  const function<double(double)> &s_fn = JMT({currState.s, currState.s_dot, currState.s_ddot}, {goalState.s, goalState.s_dot, goalState.s_ddot}, (n_steps - old_path_s.size()) * dt);
  const function<double(double)> &d_fn = JMT({currState.d, currState.d_dot, currState.d_ddot}, {goalState.d, goalState.d_dot, goalState.d_ddot}, (n_steps - old_path_s.size()) * dt);

  for(int i = 0; i < n_steps - old_path_s.size(); i++) {
    double s = s_fn(i * dt);
    double d = d_fn(i * dt);
    old_path_s.push_back(s);
    old_path_d.push_back(d);
  }

  cout<<endl<<"=========="<<endl;
  for(int i = 0; i < old_path_s.size(); i++) {
    const vector<double> &path = world.getXY(old_path_s[i], old_path_d[i]);
    cout<<"("<<old_path_s[i]<<","<<old_path_d[i]<<")  ";
    next_x_vals.push_back(path[0]);
    next_y_vals.push_back(path[1]);
  }
  cout<<endl<<"=========="<<endl;

  return {next_x_vals, next_y_vals};
}

std::function<double (double)> TrajectoryGenerator::JMT(vector<double> start, vector<double> end, double T) {
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
      3*T*T, 4*T*T*T,5*T*T*T*T,
      6*T, 12*T*T, 20*T*T*T;

  MatrixXd B = MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
      end[1]-(start[1]+start[2]*T),
      end[2]-start[2];

  VectorXd C = A.inverse() * B;

  vector <double> coeff = {start[0], start[1], .5*start[2]};
  for(int i = 0; i < C.size(); i++) {
    coeff.push_back(C.data()[i]);
  }

  return [coeff](double t) {
    double res = 0;
    for(int i = 0; i < coeff.size(); i++) {
      res += coeff[i] * pow(t, i);
    }
    return res;
  };
}

Car TrajectoryGenerator::addDelay(Car currState, size_t past, size_t steps) {
  if(past > 0 && steps > 1 && (old_path_s.size() - past) > steps) {
    // Calculate the car's position after the delay
    double s = old_path_s[past + steps + 1];
    // Calculate velocity from the last point
    double s_dot = circuitDiff(s, old_path_s[past + steps]) / dt;
    // Calculate acceleration from the change in velocity from first to last step
    double s_ddot = ((circuitDiff(old_path_s[past + 1], old_path_s[past]) / dt) - s_dot) / (steps * dt);
    // Move the car with the delay
    currState.s = s;
    currState.s_dot = s_dot;
    currState.s_ddot = s_ddot;

    // Calculate the car's position after the delay
    double d = old_path_d[past + steps + 1];
    // Calculate velocity from the last point
    double d_dot = (d - old_path_d[past + steps]) / dt;
    // Calculate acceleration from the change in velocity from first to last step
    double d_ddot = ((old_path_s[past + 1] - old_path_s[past]) / dt - d_dot) / (steps * dt);
    // Move the car with the delay
    currState.d = d;
    currState.d_dot = d_dot;
    currState.d_ddot = d_ddot;
  }

  return currState;
}

vector<double> TrajectoryGenerator::getLastVelocity(size_t prev_path_size) {
  double s_dot = 0;
  double d_dot = 0;
  if(prev_path_size > 1) {
    s_dot = circuitDiff(old_path_s[1], old_path_s[0]) / dt;
    d_dot = old_path_d[1] - old_path_d[0];
  }
  return {s_dot, d_dot};
}
