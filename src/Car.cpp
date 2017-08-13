#include "Car.h"
#include "helpers.h"

Car::Car(int id) {
  this->id = id;
  this->s = 0;
  this->s_dot = 0;
  this->s_ddot = 0;
  this->d = 0;
  this->d_dot = 0;
  this->d_ddot = 0;
}

Car::Car() {
  this->id = -1;
  this->s = 0;
  this->s_dot = 0;
  this->s_ddot = 0;
  this->d = 0;
  this->d_dot = 0;
  this->d_ddot = 0;
}

void Car::updatePos(double s, double d) {
  this->s = circuit(s);
  this->d = d;
}

size_t Car::getLane() const {
  return d < 0 ? 1000 : (size_t) (d / LANE_WIDTH);
}

double Car::getLaneD() const {
  return this->d - lane_d(this->getLane());
}

void Car::updateVelocity(double s_dot, double d_dot) {
  this->s_dot = s_dot;
  this->d_dot = d_dot;
}

void Car::display() const {
  cout<<(*this)<<endl;
}

Car Car::stateAt(double dt) const {
  Car state = Car(*this);
  state.s = circuit(s + dt * s_dot);
  return state;
}

std::ostream &operator<<(std::ostream &stream, const Car &car) {
  streamsize init_precision = stream.precision();
  stream.precision(2);
  stream<<"Car("<<car.id<<"): ";
  stream<<"s("<<fixed<<car.s<<", "<<fixed<<car.s_dot<<", "<<fixed<<car.s_ddot<<") | ";
  stream<<"d("<<fixed<<car.d<<", "<<fixed<<car.d_dot<<", "<<fixed<<car.d_ddot<<") | ";
  stream<<"lane("<<car.getLane()<<")";
  stream.precision(init_precision);
  return stream;
}

void Car::followTrajectory(const vector<double>& path_s, const vector<double>& path_d, size_t steps) {
  long idx = min(steps, path_s.size()) - 1;
  if(idx > 0) {
    double s_dot = circuitDiff(path_s[idx], path_s[idx - 1]) / dt;
    double d_dot = (path_d[idx] - path_d[idx - 1]) / dt;
    this->s_ddot = (s_dot - this->s_dot) / ((idx + 1) * dt);
    this->d_ddot = (d_dot - this->d_dot) / ((idx + 1) * dt);
    updatePos(path_s[idx], path_d[idx]);
    updateVelocity(s_dot, d_dot);
  } else if(idx == 0) {
    double s_dot = circuitDiff(path_s[idx], this->s) / dt;
    double d_dot = (path_d[idx] - this->d) / dt;
    this->s_ddot = (s_dot - this->s_dot) / dt;
    this->d_ddot = (d_dot - this->d_dot) / dt;
    updatePos(path_s[idx], path_d[idx]);
    updateVelocity(s_dot, d_dot);
  }
}

bool Car::crashWith(const Car &other) const {
  return (fabs(circuitDiff(s, other.s)) <= car_length) && (fabs(d - other.d) <= car_width);
}
