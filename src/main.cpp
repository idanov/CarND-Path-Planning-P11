#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "Map.h"
#include "Car.h"
#include "BehaviourPlanner.h"
#include "TrajectoryGenerator.h"
#include "PredictionModule.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  Map world("../data/highway_map.csv");
  std::map<int, Car> cars;
  PredictionModule predictor(world, cars);
  TrajectoryGenerator traj(world);
  BehaviourPlanner planner(traj, 1);
  Car ego(1000);

  h.onMessage([&world, &ego, &cars, &planner, &traj, &predictor](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          //////////////////////////////////////
          // Update ego's position and velocity
          //////////////////////////////////////
          const size_t past = traj.getTrajectoryLength() - previous_path_x.size();
          if(past > 0) {
            const vector<vector<double>> trajectory = traj.getTrajectory();
            ego.followTrajectory(trajectory[0], trajectory[1], past);
          } else {
            // If we don't have any historical data, we fallback to default values
            const vector<double> &frenetVelocity = world.getFrenetVelocity(j[1]["s"], j[1]["d"], mph2ms * ((double) j[1]["speed"]), deg2rad(j[1]["yaw"]));
            ego.updatePos(j[1]["s"], j[1]["d"]);
            ego.updateVelocity(frenetVelocity[0], frenetVelocity[1]);
          }

          /////////////////////////////////////////////////////
          // Previous path data given to the BehaviourPlanner
          ////////////////////////////////////////////////////
          traj.refreshPreviousPath(previous_path_x.size());

          ////////////////////////////////////////////////////////////////////////////////
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          ////////////////////////////////////////////////////////////////////////////////
          auto sensor_fusion = j[1]["sensor_fusion"];
          for(auto other : sensor_fusion) {
            int id = other[0];
            if(cars.find(id) == cars.end()) {
              cars[id].id = id;
            }

            // Update other cars' states
            cars[id].updatePos(other[5], other[6]);
            auto polar = cartesian2polar(other[3], other[4]);
            auto other_sd_dot = world.getFrenetVelocity(cars[id].s, cars[id].d, polar[0], polar[1]);
            cars[id].updateVelocity(other_sd_dot[0], other_sd_dot[1]);
          }

          /////////////////
          // Action
          /////////////////
          vector<vector<Car>> predictions = predictor.generatePredictions(n_steps);
          Car goalState = planner.updatePlan(ego, predictions);
          vector<vector<double>> path = traj.updateTrajectory(ego, goalState);

          vector<double> next_x_vals = path[0];
          vector<double> next_y_vals = path[1];


          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































