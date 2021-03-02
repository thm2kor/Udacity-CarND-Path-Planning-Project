#ifndef PLANNER_H
#define PLANNER_H

#include <string>
#include <cstdio>
#include <vector>
#include <map>
#include <cstdio>
#include <math.h>
#include "vehicle.h"
#include "config.h"


using std::vector;

class Planner
{
private:
  Vehicle ego;                            // ego vehicle 
  int ego_key;                            // Unique identifier for ego vehicle
  Ego_State current_state;                // Current state of the ego vehicle  
  std::map<int, Vehicle*> other_vehicles; // Map of all vehicles including the ego vehicles
  int target_lane = 0;                    // The trajectory of the ego vehicle will 
  
  double ego_lane_velocity;               // velocity of vehicle ahead in the ego lane
  vector<double> avg_lane_velocity;       // velocities of non-ego lanes. 
  vector<double> too_close_vehicles;       // s_diff of ego lanes to ith lane
  vector<double> map_waypoints_x;         // x-coordinates of way-points
  vector<double> map_waypoints_y;         // y-coordinates of way-points
  vector<double> map_waypoints_s;         // s-coordinates of way-points

public:
  vector<double> next_x_vals;             // x-coordinates of points which would be sent to the simulator
  vector<double> next_y_vals;             // y-coordinates of points which would be sent to the simulator

 
public:
  Planner();
  ~Planner();
  // Stage  1: Initialization
  // For setting up the environment of the Planner.
  // This set of functions should be used to initialize the planner
  Vehicle get_ego();
  void add_ego(double x, double y, double s, double d, double yaw, double velocity);
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
  
  // Stage  3: State Estimation and Execution
  Ego_State execute_next_state();
  Ego_State execute_followlane();
  Ego_State execute_left_lanechange();
  Ego_State execute_right_lanechange();
  void calculate_lane_velocities();
  void update_vehicle_positions();
  vector<Ego_State> successor_states();
  // Stage 4: Execute Trajectory
  void prepare_trajectory (vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double &ref_vel);
  
  
  void remove_other_vehicles();

};

#endif // PLANNER_H
