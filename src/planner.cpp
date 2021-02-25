#include <iostream>
#include "planner.h"
#include "spline.h"
#include "config.h"

using std::vector;
using std::cout;
using std::endl;
using std::pair;

const char * state_names[] = { "INIT", "FOLLOW CAR AHEAD", "PREP TO TRUN LEFT", "LANE CHANGE LEFT", "PREP TO TRUN RIGHT", "LANE CHANGE RIGHT"};

Planner::Planner()
{
  current_state = Ego_State::follow_vehicle_in_lane;
  target_lane = START_LANE;
}

Planner::~Planner()
{
  
}

void Planner::add_ego(double x, double y, double s, double d, double yaw, double velocity) {
  //cout << "adding ego with s value " << s << endl;
  ego = Vehicle(-1, x , y , 0, 0, s, d);
}

Vehicle Planner::get_ego() {
  return this->ego;
}

void Planner::add_other_traffic_participants (vector<vector<double>> sensor_fusion){
  for (int i = 0; i < sensor_fusion.size(); i++) {
    double d = sensor_fusion[i][6];             
    if (d >= 0 && d <= d_right(LANE_RIGHT)) {
      Vehicle *vehicle = new Vehicle(sensor_fusion[i][0], sensor_fusion[i][1],
                              sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4], 
                              sensor_fusion[i][5], sensor_fusion[i][6]);
      update_vehicle(vehicle);
    }
  }
}

void Planner::update_vehicle(Vehicle *vehicle) {
  auto iterator = other_vehicles.find(vehicle->id);
  // if the vehicle is already avaialble, point the address to the new
  // object. delete the previous object
  if (iterator != other_vehicles.end()) {
    delete (*iterator).second;    
    (*iterator).second = vehicle;
    // cout << "vehicle id:" << vehicle->id << " updated." << endl;
  } else { //vehicle not found. Add a new vehicle to the map
    add_vehicle(vehicle);
  }
}

void Planner::add_vehicle(Vehicle *vehicle)
{
  other_vehicles.insert(pair<int, Vehicle *>(vehicle->id, vehicle));
  // cout << "inserting vehicle id:" << vehicle->id << endl;
}

int Planner::get_vehicle_count() {
  return other_vehicles.size();
}

void Planner::set_waypoints(vector<double> &map_x, vector<double> &map_y, vector<double> &map_s) {
  map_waypoints_x = map_x;
  map_waypoints_y = map_y;
  map_waypoints_s = map_s;
}

void Planner::predict(double delta_t){
  ego.update_kinematics(delta_t);
  //cout << "checking on " << other_vehicles.size() << " vehicles" << endl;
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



Vehicle *Planner::get_vehicle_ahead(int lane) {
  Vehicle *next_vehicle = nullptr;

  for (auto &obj: other_vehicles) {
    if (obj.second->lane == lane) {
      // vehicle found in given lane
      // check if it is the initial one or closer than the last stored one
      // cout << "vehicle " << obj.second->id << " found in lane " << obj.second->lane << " with s value "  << obj.second->s << " :: S value of ego is " << ego.s << endl;
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
      new_state = execute_followlane();
      break;
    case Ego_State::prepare_lanechange_left:
      new_state = execute_prep_left_lanechange();
      break;
    case Ego_State::lanechange_left:
      new_state = execute_left_lanechange();
      break;
    case Ego_State::prepare_lanechange_right:
      new_state = execute_prep_right_lanechange();
      break;
    case Ego_State::lanechange_right:
      new_state = execute_right_lanechange();
      break;
    default:
      cout << "********INVALID STATE********" << endl;
      break;
  }
  if (new_state != current_state) {
    cout << "Lane state change: new=" << state_names[new_state] << ", previous= " << state_names[current_state] << endl;
    current_state = new_state;
  }
  //cout << "curr_state: " << current_state << " #### new_state: " << new_state << endl;
  //cout << "Lane state change: new=" << state_names[new_state] << ", previous= " << state_names[current_state] << endl;
  return current_state;
}



bool Planner::is_lane_valid(int lane){
  return (lane >= 0) && (lane <=3);
}

Ego_State Planner::execute_followlane() {
  // determine next behavior state
  // close vehicle ahead, prepare lane change to fastest lane
  Ego_State new_state = Ego_State::follow_vehicle_in_lane;
  Vehicle *ahead = get_vehicle_ahead(ego.lane);
  
  if (ahead) { //if there is a vehicle ahead of the ego vehicle
    // cout << "vehicle " << ahead->id << " ahead within a reaching distance of  " << (ahead->s - ego.s) << endl;
    if ((ahead->s > ego.s) && ((ahead->s - ego.s) < 30))
    { // if the distance between the ego and vehicle ahead is less than 30m
      // prepare to turn left, if there is a left lane available and clear of vehicles for 30m,
      cout << "do lane change .. left or right ? " << endl;
      /*if (is_lane_valid(ego.lane-1) && is_lane_clear(ego.lane-1) ){
        target_lane--;
        new_state = Ego_State::prepare_lanechange_left;
      } else if (is_lane_valid(ego.lane+1) && is_lane_clear(ego.lane+1)) {
        // left lane not available, prepare to turn right, if there is a right lane available,
        // and clear of vehicles for 30m,
        target_lane++;
        new_state = Ego_State::prepare_lanechange_right;
      } else {
        //no where to go. declerate and stay on the current lane
        cout << "do nothing ... " << endl;
        ego.v_magnitude -= MAX_ACCL;
      }*/
    }
    else {
      // cout << "Vehicle far away . Accelerate cautiously in lane " << ego.lane << endl;
      ego.v_magnitude += MAX_ACCL;
      //cout << "increasing speed to " << ego.v_magnitude << endl;
      new_state = Ego_State::follow_vehicle_in_lane;
    }
    
  } else {   
    // cout << "no vehicle ahead in lane " << ego.lane << ".Accelerate ..." << endl;
    ego.v_magnitude += MAX_ACCL;
    //cout << "increasing speed to " << ego.v_magnitude << endl;
    new_state = Ego_State::follow_vehicle_in_lane;
  }
  return new_state;
}

Ego_State Planner::execute_prep_left_lanechange(){
  return Ego_State::follow_vehicle_in_lane;
}

Ego_State Planner::execute_left_lanechange(){
  return Ego_State::follow_vehicle_in_lane;
}

Ego_State Planner::execute_prep_right_lanechange(){
  return Ego_State::follow_vehicle_in_lane;
}

Ego_State Planner::execute_right_lanechange(){
  return Ego_State::follow_vehicle_in_lane;
}

void Planner::prepare_trajectory (vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s) {

}