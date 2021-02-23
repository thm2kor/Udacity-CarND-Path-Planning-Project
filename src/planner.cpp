#include <iostream>
#include "planner.h"

using std::vector;
using std::cout;
using std::endl;
using std::pair;

const char * state_names[] = { "INVALID", "FOLLOW", "PREP_LEFT", "LC_LEFT", "PREP_RIGHT", "LC_RIGHT"};

Planner::Planner()
{
  current_state = Ego_State::follow_vehicle_in_lane;
}

Planner::~Planner()
{
  
}

void Planner::update_kinematics(double delta_t){
  ego.update_kinematics(delta_t);
  for (auto &obj: other_vehicles) {
    obj.second->update_kinematics(delta_t);
  }
}

void Planner::remove_other_vehicles() {
  //remove all the previous vehicles
  for (auto &obj: other_vehicles) {
    delete obj.second;
  }
  other_vehicles.clear();
}

void Planner::update_vehicle(Vehicle *vehicle) {
  auto iterator = other_vehicles.find(vehicle->id);
  // if the vehicle is already avaialble, point the address to the new
  // object. delete the previous object
  if (iterator != other_vehicles.end()) {
    delete (*iterator).second;    
    (*iterator).second = vehicle;
    //cout << "vehicle id:" << vehicle->id << " updated." << endl;
  } else { //vehicle not found. Add a new vehicle to the map
    add_vehicle(vehicle);
  }
}

Vehicle *Planner::get_vehicle_ahead(int lane) {
  Vehicle *next_vehicle = nullptr;

  for (auto &obj: other_vehicles) {
    if (obj.second->lane == lane) {
      // vehicle found in given lane
      // check if it is the initial one or closer than the last stored one
      double delta_s = obj.second->s - ego.s;
      if (delta_s >= 0 && next_vehicle == nullptr) {
        next_vehicle = obj.second;
      } else if (delta_s >= 0 && next_vehicle) {
        double delta_s_next = next_vehicle->s - ego.s;
        if (delta_s < delta_s_next) {
          next_vehicle = obj.second;
        }
      }
    }
  }
  return next_vehicle;
}

bool Planner::is_lane_clear (int lane) {
  Vehicle *next_vehicle = nullptr;
  bool result = true;
  for (auto obj: other_vehicles) {
    if (obj.second->lane == lane) {
      //if the difference between a vehicle and the ego is less than 30
      if (abs(obj.second->s - ego.s) < 30) {
        result = false;
      } 
    }
  }
  return result;
}

Vehicle *Planner::get_vehicle_behind(int lane) {
  Vehicle *next_vehicle = nullptr;
  for (auto obj: other_vehicles) {
    if (obj.second->lane == lane) {
      // vehicle found in given lane
      // check if it is the initial one or closer than the last stored one
      double delta_s = obj.second->s - ego.s;
      if (delta_s < 0 && next_vehicle == nullptr) {
        next_vehicle = obj.second;
      } else if (delta_s < 0 && next_vehicle) {
        double delta_s_next = next_vehicle->s - ego.s;
        if (delta_s > delta_s_next) {
          next_vehicle = obj.second;
        }
      } 
    }
  }
  return next_vehicle;
}

Ego_State Planner::get_next_state() {
  Ego_State new_state = Ego_State::invalid_state;
  switch (current_state) {
    case Ego_State::follow_vehicle_in_lane:
      new_state = executeKeepLane();
      break;
    case Ego_State::prepare_lanechange_left:
      new_state = executePrepareLaneChangeLeft();
      break;
    case Ego_State::lanechange_left:
      new_state = executeLaneChangeLeft();
      break;
    case Ego_State::prepare_lanechange_right:
      new_state = executePrepareLaneChangeRight();
      break;
    case Ego_State::lanechange_right:
      new_state = executeLaneChangeRight();
      break;
    default:
      cout << "INVALID STATE." << endl;
      break;
  }
  if (new_state != current_state) {
    cout << "Lane state change: new=" << state_names[new_state] << ", previous= " << state_names[current_state] << endl;
    current_state = new_state;
  }
  cout << "Lane state change: new=" << state_names[new_state] << ", previous= " << state_names[current_state] << endl;
  return current_state;
}

void Planner::add_vehicle(Vehicle *vehicle)
{
  other_vehicles.insert(pair<int, Vehicle *>(vehicle->id, vehicle));
  //cout << "vehicle id:" << vehicle->id << " added." << endl;
}

Vehicle Planner::get_ego() {
  return this->ego;
}

void Planner::add_ego(double x, double y, double s, double d, double yaw, double velocity) {
  ego = Vehicle(-1, x, y, s, d, yaw, velocity);
}

bool Planner::is_lane_valid(int lane){
  return (lane >= 0) && (lane <=3);
}

Ego_State Planner::executeKeepLane() {
  // determine next behavior state
  // close vehicle ahead, prepare lane change to fastest lane
  Ego_State new_state = Ego_State::follow_vehicle_in_lane;
  Vehicle *ahead = get_vehicle_ahead(ego.lane);
  
  if (ahead) { //if there is a vehicle ahead of the ego vehicle
    if ((ahead->s > ego.s) && ((ahead->s - ego.s) < 30))
    { // if the distance between the ego and vehicle ahead is less than 30m
      // prepare to turn left, if there is a left lane available and clear of vehicles for 30m,
      if (is_lane_valid(ego.lane-1) && is_lane_clear(ego.lane-1) ){
        target_lane--;
        new_state = Ego_State::prepare_lanechange_left;
      } else if (is_lane_valid(ego.lane+1) && is_lane_clear(ego.lane+1)) {
        // left lane not available, prepare to turn right, if there is a right lane available,
        // and clear of vehicles for 30m,
        target_lane++;
        new_state = Ego_State::prepare_lanechange_right;
      } else {
        //no where to go. declerate and stay on the current lane
      }
    }
  } else {
    new_state = Ego_State::follow_vehicle_in_lane;
  }
  return new_state;
}

Ego_State Planner::executePrepareLaneChangeLeft(){
  return Ego_State::follow_vehicle_in_lane;
}

Ego_State Planner::executeLaneChangeLeft(){
  return Ego_State::follow_vehicle_in_lane;
}

Ego_State Planner::executePrepareLaneChangeRight(){
  return Ego_State::follow_vehicle_in_lane;
}

Ego_State Planner::executeLaneChangeRight(){
  return Ego_State::follow_vehicle_in_lane;
}