#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int id, double x, double y, double s, double d, double yaw, double velocity);
  void update_kinematics(double delta_t);
  double get_distance_to (Vehicle *other);
  double get_timegap_to (Vehicle *other);
  double get_collision_time_to (Vehicle *other);
  //Identifier for the vehicle
  int id; 
  // vehicle position in cartesian coordinates (current and predicted)
  double x; double y;
  vector<double> pred_trj_x; 
  vector<double> pred_trj_y; 
  // vehicle position in frenet coordinates  (current and predicted)
  double s; double d;
  vector<double> pred_trj_s; 
  vector<double> pred_trj_d; 
  // velocity in x and y directions (current and predicted)
  double vx; double vy; double v_magnitude;
  vector<double> pred_trj_vx;
  vector<double> pred_trj_vy;
  vector<double> pred_trj_v_mag;
  // acceleration in x and y directions (current and predicted)
  double ax; double ay; double a_magnitude;
  vector<double> pred_trj_ax;
  vector<double> pred_trj_ay;
  vector<double> pred_trj_a_mag; 
  // yaw angle (current and predicted)
  double yaw;
  vector<double> pred_trj_yaw;
  // lane on which the vehicle is currently driving
  int lane;
  //position w.r.t to the ego vehicle
  bool ahead;
  bool in_same_lane; bool in_right_lane; bool in_left_lane;
};

#endif  // VEHICLE_H