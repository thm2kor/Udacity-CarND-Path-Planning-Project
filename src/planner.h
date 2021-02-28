#ifndef PLANNER_H
#define PLANNER_H

#include <string>
#include <vector>
#include <map>
#include <cstdio>
#include "vehicle.h"
#include "config.h"
#include <math.h>
using std::vector;

class Planner
{
public:
  //Ego vehicle and its current state
  Vehicle ego;
  Ego_State current_state;
  //Map of all vehicles including the ego vehicles
  std::map<int, Vehicle*> other_vehicles;
  int target_lane = 0; 
  //int reference_velocity = 0;
  //int target_velocity = 0;
  
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  int ego_key = -1;
public:
  Planner();
  ~Planner();
  // Stage  1: Initialization
  // For setting up the environment of the Planner.
  // This set of functions should be used to initialize the planner
  Vehicle get_ego();
  void add_ego(double x, double y, double s, double d, double yaw, double velocity);
  void add_vehicle(Vehicle *vehicle);
  void add_other_traffic_participants (vector<vector<double>> sensor_fusion);
  int get_vehicle_count();
  void set_waypoints(vector<double> &map_x, vector<double> &map_y, vector<double> &map_s);
  void update_vehicle(Vehicle *vehicle);
    
  // Stage  2: Prediction
  void predict(double delta_t);
  Vehicle* get_vehicle_ahead(int lane);
  Vehicle* get_vehicle_behind(int lane);
  bool is_lane_clear (int lane);
  bool is_lane_valid (int lane);
  
  // Stage  3: State Estimation
  Ego_State execute_next_state();
  Ego_State execute_followlane();
  Ego_State execute_left_lanechange();
  Ego_State execute_right_lanechange();
  
  // Stage 4: Execute Trajectory
  void prepare_trajectory (vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double &ref_vel);
  
  
  void remove_other_vehicles();

};

#endif // PLANNER_H
