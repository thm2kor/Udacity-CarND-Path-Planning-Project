#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "config.h"
#include "spline.h"
#include "planner.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  bool first = true;
  Planner planner;
  double ref_vel = 0.0;
  int lane_start = LANE_CENTER;
  
  h.onMessage([&planner, &ref_vel, &first, &lane_start, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Step 1: Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];                  
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          // Provided previous path point size.
          int prev_size = previous_path_x.size();
          //std::cout << "prev_size is " << prev_size << std::endl;
          /*for ( int i = 0; i < prev_size; i++ ) {
            std::cout << "incoming " << previous_path_x [i] << " " << previous_path_y [i] << std::endl;
          }*/
          // Preventing collitions.
          if (prev_size > 0) {
            car_s = end_path_s;
          }
          
          json msgJson;

          /**
          * TODO: define a path made up of (x,y) points that the car will visit
          *   sequentially every .02 seconds
          */
          // set-up the traffic environment for the planner.
          planner.add_ego(car_x, car_y, car_s, car_d, car_yaw, car_speed);
          planner.add_other_traffic_participants(sensor_fusion);
          planner.set_waypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s);          
          // predict the next position of the based on the kinematic motion models
          planner.predict(PREDICTION_TIME);
          // get the next plausible state
          planner.get_next_state();
          // calculate the trajectory
          planner.prepare_trajectory(previous_path_x, previous_path_y, end_path_s, ref_vel);
          
          msgJson["next_x"] = planner.next_x_vals;
          msgJson["next_y"] = planner.next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          // clear the pointers to other vehicles.
          //std::cout << "Calculations based on " << planner.get_vehicle_count() << " vehicles" << std::endl;
          //planner.remove_other_vehicles();
          //std::cout << "After removing... vehicle count is " << planner.get_vehicle_count() << std::endl;
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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