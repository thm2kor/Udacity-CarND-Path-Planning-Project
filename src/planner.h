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
  std::vector<double> x_pts;
  std::vector<double> y_pts;
public:
  //Ego vehicle
  Vehicle ego;
  Ego_State current_state;
  //Map of all vehicle including the ego vehicles
  std::map<int, Vehicle*> other_vehicles;
  int target_lane = 0;
  int target_velocity = 0;
  
  int ego_key = -1;
public:
  Planner();
  ~Planner();
  // Road functions
  Vehicle get_ego();
  void add_ego(double x, double y, double s, double d, double yaw, double velocity);
  void add_vehicle(Vehicle *vehicle);
  void update_vehicle(Vehicle *vehicle);
  void update_kinematics(double delta_t);
  void remove_other_vehicles();
  Vehicle* get_vehicle_ahead(int lane);
  Vehicle* get_vehicle_behind(int lane);
  bool is_lane_clear (int lane);
  bool is_lane_valid (int lane);
  
  Ego_State get_next_state();
  Ego_State executeKeepLane();
  Ego_State executePrepareLaneChangeLeft();
  Ego_State executeLaneChangeLeft();
  Ego_State executePrepareLaneChangeRight();
  Ego_State executeLaneChangeRight();
};

#endif // PLANNER_H
