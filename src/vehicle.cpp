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
}

Vehicle::Vehicle(int id, double x, double y, double s, double d, double yaw, double velocity) {
  this->id = id;
  this->x = x;
  this->y = y;
  
  this->s = s;
  this->d = d;

  this->vx = 0.0f;
  this->vy = 0.0f;
  v_magnitude = velocity;

  ax = 0.0;
  ay = 0.0;
  a_magnitude = sqrt(pow(ax, 2) + pow(ay, 2));
  
  yaw = yaw;
  
  lane = get_lane(d); 
}

void Vehicle::update_kinematics(double delta_t) {

  pred_trj_x.clear();
  pred_trj_y.clear();
  pred_trj_s.clear();
  pred_trj_d.clear();
  pred_trj_vx.clear();
  pred_trj_vy.clear();
  pred_trj_v_mag.clear();
  pred_trj_ax.clear();
  pred_trj_ay.clear();
  pred_trj_a_mag.clear();
  pred_trj_yaw.clear();

  for (int i = 0; i < (delta_t / CYCLE_TIME); ++i) {
    double dt = i * CYCLE_TIME;
    pred_trj_x.push_back(x + (vx * dt + 0.5 * ax * pow(dt, 2)));
    pred_trj_y.push_back(y + (vy * dt + 0.5 * ay * pow(dt, 2)));

    pred_trj_vx.push_back(vx + ax * delta_t);
    pred_trj_vy.push_back(vy + ay * delta_t);
    pred_trj_v_mag.push_back(v_magnitude + a_magnitude * delta_t);
  }
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