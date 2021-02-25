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

bool Planner::is_lane_clear (int lane) {
  Vehicle *next_vehicle = nullptr;
  bool result = true;
  for (auto obj: other_vehicles) {
    if (obj.second->lane == lane) {
      //if the difference between a vehicle and the ego is less than 30
      result &= abs(obj.second->s - ego.s) < 30;
    }
  }
  return result;
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
  return current_state;
}

bool Planner::is_lane_valid(int lane){
  return (lane >= 0) && (lane <=3);
}

Ego_State Planner::execute_followlane() {

  Ego_State new_state = Ego_State::follow_vehicle_in_lane;
  Vehicle *ahead = get_vehicle_ahead(ego.lane);
  
  if (ahead) { //if there is a vehicle ahead of the ego vehicle
    if ((ahead->s > ego.s) && ((ahead->s - ego.s) < 30))
    { // if the distance between the ego and vehicle ahead is less than 30m
      // prepare to turn left, if there is a left lane available 
      if (is_lane_valid(ego.lane-1)){
        target_lane--;
        new_state = Ego_State::prepare_lanechange_left;
      } else if (is_lane_valid(ego.lane+1)) {
        // left lane not available, prepare to turn right, if there is a right lane available,
        // and clear of vehicles for 30m,
        target_lane++;
        new_state = Ego_State::prepare_lanechange_right;
      } else {
        //no where to go. declerate and stay on the current lane
        ego.v_magnitude -= MAX_ACCL;
      }
    }
    else {
      // cout << "vehicle " << ahead->id << " ahead within a reaching distance of  " << (ahead->s - ego.s) << endl;
      ego.v_magnitude += MAX_ACCL;
      // cout << "increasing speed to " << ego.v_magnitude << endl;
      new_state = Ego_State::follow_vehicle_in_lane;
    }    
  } else {   
    //cout << "no vehicle ahead in lane " << ego.lane << ". Accelerate ..." << endl;
    ego.v_magnitude += MAX_ACCL;
    new_state = Ego_State::follow_vehicle_in_lane;
  }
  return new_state;
}

Ego_State Planner::execute_prep_left_lanechange(){
  Ego_State new_state = Ego_State::lanechange_left;
  //check if there are no vehicles 30 min before and after the ego vehicle
  if ( !is_lane_clear(target_lane) ){ 
    cout << "left lane not clear. stay in lane and look for alternate options" << endl;
    target_lane++;
    new_state = Ego_State::follow_vehicle_in_lane;  
  }
  return new_state;
}

Ego_State Planner::execute_left_lanechange(){
  ego.lane = target_lane;
  return Ego_State::follow_vehicle_in_lane;
}

Ego_State Planner::execute_prep_right_lanechange(){
  return Ego_State::follow_vehicle_in_lane;
}

Ego_State Planner::execute_right_lanechange(){
  return Ego_State::follow_vehicle_in_lane;
}

void Planner::prepare_trajectory (vector<double> previous_path_x, vector<double> previous_path_y, 
                                               double end_path_s, double &ref_vel) {
  vector<double> ptsx;
  vector<double> ptsy;
  
  next_x_vals.clear();
  next_y_vals.clear();
  
  double speed_diff = 0;
  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = deg2rad(ego.yaw);
  int prev_size = previous_path_x.size();
  // Do I have have previous points
  if ( prev_size < 2 ) {
      // There are not too many...
      double prev_car_x = ego.x - cos(ego.yaw);
      double prev_car_y = ego.y - sin(ego.yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(ego.x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(ego.y);
  } else {
      // Use the last two points.
      ref_x = previous_path_x[prev_size - 1];
      ref_y = previous_path_y[prev_size - 1];
      //std::cout << "Second: " << previous_path_x[prev_size - 2] << " " << previous_path_x[prev_size - 1] << std::endl;
      double ref_x_prev = previous_path_x[prev_size - 2];
      double ref_y_prev = previous_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
  }
  int lane = target_lane;
  // Setting up target points in the future.
  vector<double> next_wp0 = getXY(ego.s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(ego.s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(ego.s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  // Making coordinates to local car coordinates.
  for ( int i = 0; i < ptsx.size(); i++ ) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    //std::cout << ptsx[i] << " " << ptsy[i] << std::endl;
  }

  // Create the spline.
  tk::spline s;
  s.set_points(ptsx, ptsy);

  for ( int i = 0; i < prev_size; i++ ) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate distance y position on 30 m ahead.
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for( int i = 1; i < 50 - prev_size; i++ ) {
    ref_vel += MAX_ACCL;
    if ( ref_vel > MAX_SPEED ) {
      ref_vel = MAX_SPEED;
      //std::cout << "limiting speed to " << ref_vel << std::endl;
    } else if ( ref_vel < MAX_ACCL ) {
      ref_vel = MAX_ACCL;
    }
    //std::cout << "ref_vel is " << ref_vel << std::endl;
    double N = target_dist/(0.02*ref_vel/2.24);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}