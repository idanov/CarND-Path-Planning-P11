#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <math.h>
#include "spline.h"
#include "helpers.h"

using namespace std;

class Map {
  vector<double> maps_x;
  vector<double> maps_y;
  vector<double> maps_s;
  vector<double> maps_dx;
  vector<double> maps_dy;

public:
  explicit Map(string map_file);
  vector<tk::spline> getSplines(double s, double d);
  vector<double> getXY(double s, double d);
  vector<double> getFrenetVelocity(double s, double d, double speed, double yaw);
};


#endif //PATH_PLANNING_MAP_H
