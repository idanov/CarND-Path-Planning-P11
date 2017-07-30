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

void Car::updateSpeed(double vx, double vy) {
  this->speed = euclidean(vx, vy);
  this->yaw = rad2deg(atan2(vy, vx));
  if(this->yaw < 0) this->yaw += 360;
}

void Car::updateYawAndSpeed(double yaw, double speed) {
  this->yaw = yaw;
  this->speed = speed;
}

int Car::getLane() {
  return (int) d / (int) LANE_WIDTH;
}

double Car::getLaneD() {
  return this->d - (this->getLane() * LANE_WIDTH + LANE_WIDTH / 2);
}
