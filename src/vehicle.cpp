#include "vehicle.h"
#include "config.h"
#include <iostream>
#include <cmath>

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){
  id = 0;
  x = 0.0;
  y = 0.0;

  s = 0.0;
  d = 0.0;

  vx = 0.0;
  vy = 0.0;
  v_magnitude = 0.0;

  ax = 0.0;
  ay = 0.0;
  a_magnitude = 0.0;
  
  yaw = 0.0;

  lane = -1;
  v_delta = 0.0;

  s_diff = 0.0;
}

Vehicle::Vehicle( int id, double x, double y, double vx, double vy, double s, double d) {
  this->id = id;
  this->x = x;
  this->y = y;
  
  this->s = s;
  this->d = d;

  this->vx = vx;
  this->vy = vy;
  v_magnitude = sqrt(pow(vx, 2) + pow(vy, 2)) ;

  ax = 0.0;
  ay = 0.0;
  a_magnitude = 0.0;
  
  this->yaw = 0;
  
  lane = get_lane(d); 
  v_delta = 0.0;
  s_diff = 0.0;

}

Vehicle::~Vehicle() {
}

void Vehicle::update_kinematics(double delta_t) {
  // future predicted Frenet s value will be its current s value plus its 
  // (transformed) total velocity (m/s) multiplied by the time elapsed 
  this->s+= v_magnitude*delta_t; //TODO include the acceleration component as well
}

double Vehicle::get_distance_to (Vehicle *other) {
  return sqrt(pow((other->x - this->x), 2) + pow((other->y - this->y), 2));
}

double Vehicle::get_timegap_to (Vehicle *other) {
  double dist = sqrt(pow((other->x - this->x), 2) + pow((other->y - this->y), 2));
  return (dist / this->v_magnitude);
}

double Vehicle::get_collision_time_to (Vehicle *other) {
  double dist = sqrt(pow((other->x - this->x), 2) + pow((other->y - this->y), 2));
  return abs(dist / (this->v_magnitude - other->v_magnitude));
}

