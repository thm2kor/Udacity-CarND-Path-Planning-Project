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
  ~Vehicle();
  void update_kinematics(double delta_t);
  double get_distance_to (Vehicle *other);
  double get_timegap_to (Vehicle *other);
  double get_collision_time_to (Vehicle *other);

public:
  //Identifier for the vehicle
  int id; 
  // vehicle position in cartesian coordinates 
  double x; double y;
  // vehicle position in frenet coordinates 
  double s; double d;
  // velocity in x and y directions 
  double vx; double vy; double v_magnitude; double v_delta;
  // acceleration in x and y directions 
  double ax; double ay; double a_magnitude;
  // yaw angle 
  double yaw;
  // lane on which the vehicle is currently driving
  int lane;
  // difference in s-vallue w.r.t to the Ego vehicle.
  float s_diff; 
};

#endif  // VEHICLE_H