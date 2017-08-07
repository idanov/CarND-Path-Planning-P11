#include "Map.h"

Map::Map(string map_file_) {
  // Load up map values for waypoint's x,y,s and d normalized normal vectors

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    maps_x.push_back(x);
    maps_y.push_back(y);
    maps_s.push_back(s);
    maps_dx.push_back(d_x);
    maps_dy.push_back(d_y);
  }

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d) {
  const vector<tk::spline> &curves = getSplines(s, d);
  return {curves[0](s), curves[1](s)};
}

vector<double> Map::getFrenetVelocity(double s, double d, double speed, double theta) {
  const vector<tk::spline> &curves = getSplines(s, d);
  double dx = curves[0].deriv(1, s);
  double dy = curves[1].deriv(1, s);

  double road_angle = atan2(dy, dx);
  if(road_angle < 0) road_angle += 2 * M_PI;
  double diff = theta - road_angle;

  double s_dot = fabs(speed * cos(diff));
  double d_dot = speed * sin(diff);

  return {s_dot, d_dot};
}

vector<tk::spline> Map::getSplines(double s, double d) {
  // Ensure s is [0, max_s]
  s = circuit(s);
  // Use log2(N) operations for finding the last passed waypoint
  const vector<double>::iterator &upper = std::upper_bound(maps_s.begin(), maps_s.end(), s);
  long prev_wp = upper - maps_s.begin();
  prev_wp -= 1;

  vector<double> nearest_s;
  vector<double> nearest_x;
  vector<double> nearest_y;

  for(int i = -3; i < 5; i++) {
    size_t n = maps_s.size();
    size_t wp = (n + prev_wp + i) % n;
    nearest_x.push_back(maps_x[wp] + d * maps_dx[wp]);
    nearest_y.push_back(maps_y[wp] + d * maps_dy[wp]);
    // Correct for circuit coordinates
    double temp_s = maps_s[wp];
    if(prev_wp + i < 0) {
      temp_s -= max_s;
    } else if(prev_wp + i >= n) {
      temp_s += max_s;
    }
    nearest_s.push_back(temp_s);
  }

  // Get the curve from the nearest 6 waypoints
  tk::spline curve_x;
  tk::spline curve_y;
  curve_x.set_points(nearest_s, nearest_x);
  curve_y.set_points(nearest_s, nearest_y);
  
  return {curve_x, curve_y};
}
