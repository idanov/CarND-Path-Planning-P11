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

  // The max s value before wrapping around the track back to 0
  const double max_s = 6945.554;

  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);

public:
  explicit Map(string map_file);
  vector<double> getXY(double s, double d);
  vector<double> getFrenet(double x, double y, double theta);
};


#endif //PATH_PLANNING_MAP_H
