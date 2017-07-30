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

int Map::ClosestWaypoint(double x, double y) {

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int Map::NextWaypoint(double x, double y, double theta) {

  int closestWaypoint = ClosestWaypoint(x, y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta - heading);

  if (angle > pi() / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta) {
  int next_wp = NextWaypoint(x, y, theta);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d) {
  // Ensure s is [0, max_s]
  if(s >= max_s) s -= max_s;
  if(s < 0) s += max_s;
  // Use log2(N) operations for finding the last passed waypoint
  auto upper = std::upper_bound(maps_s.begin(), maps_s.end(), s);
  long prev_wp = upper - maps_s.begin();
  prev_wp -= 1;

  vector<double> nearest_s;
  vector<double> nearest_x;
  vector<double> nearest_y;

  for(int i = -2; i < 4; i++) {
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

  double x = curve_x(s);
  double y = curve_y(s);

  return {x, y};

}

vector<double> Map::getFrenetVelocity(double s, double d, double speed, double theta) {
  // Ensure s is [0, max_s]
  if(s >= max_s) s -= max_s;
  if(s < 0) s += max_s;
  // Use log2(N) operations for finding the last passed waypoint
  auto upper = std::upper_bound(maps_s.begin(), maps_s.end(), s);
  long prev_wp = upper - maps_s.begin();
  prev_wp -= 1;

  vector<double> nearest_s;
  vector<double> nearest_x;
  vector<double> nearest_y;

  for(int i = -2; i < 4; i++) {
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

  double x = curve_x(s);
  double y = curve_y(s);
  double x2 = curve_x(s + 1);
  double y2 = curve_y(s + 1);

  double road_angle = atan2(y2 - y, x2 - x);
  if(road_angle < 0) road_angle += 2 * M_PI;
  double diff = theta - road_angle;

  double s_dot = speed * cos(diff);
  double d_dot = speed * sin(diff);

  return {s_dot, d_dot};
}
