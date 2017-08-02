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

int Car::getLane() {
  return (int) d / (int) LANE_WIDTH;
}

double Car::getLaneD() {
  return this->d - (this->getLane() * LANE_WIDTH + LANE_WIDTH / 2);
}

void Car::updateVelocity(double s_dot, double d_dot) {
  this->s_dot = s_dot;
  this->d_dot = d_dot;
}

void Car::display() {
  cout<<"Car("<<id<<"): s("<<s<<","<<s_dot<<","<<s_ddot<<")   d("<<d<<","<<d_dot<<","<<d_ddot<<")"<<endl;
}

Car Car::stateAt(double dt) {
  Car state = Car(*this);
  state.s = circuit(s + dt * s_dot);
  return state;
}
