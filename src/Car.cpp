#include "Car.h"
#include "helpers.h"

Car::Car(int id) {
  this->id = id;
}

Car::Car() {
  this->id = -1;
}

void Car::updatePos(double s, double d, double x, double y) {
  this->s = s;
  this->d = d;
  this->x = x;
  this->y = y;
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
