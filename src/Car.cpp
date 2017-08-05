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

int Car::getLane() const {
  return (int) (d / LANE_WIDTH);
}

double Car::getLaneD() const {
  return this->d - (this->getLane() * LANE_WIDTH + LANE_WIDTH / 2);
}

void Car::updateVelocity(double s_dot, double d_dot) {
  this->s_dot = s_dot;
  this->d_dot = d_dot;
}

string Car::display() const {
  stringstream res;
  res.precision(2);
  res<<"Car("<<id<<"): ";
  res<<"s("<<fixed<<s<<", "<<fixed<<s_dot<<", "<<fixed<<s_ddot<<") | ";
  res<<"d("<<fixed<<d<<", "<<fixed<<d_dot<<", "<<fixed<<d_ddot<<") | ";
  res<<"lane("<<getLane()<<")"<<endl;
  return res.str();
}

Car Car::stateAt(double dt) {
  Car state = Car(*this);
  state.s = circuit(s + dt * s_dot);
  return state;
}
