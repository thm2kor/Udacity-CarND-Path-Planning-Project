#include <iostream>
#include <algorithm>
#include <iterator>
#include "cost.h"
#include "planner.h"
#include "spline.h"
#include "config.h"
#include <limits>
#include <functional>

using std::vector;
using std::cout;
using std::endl;
using std::pair;

const char * state_names[] = { "INIT", "FOLLOW CAR AHEAD", "LANE CHANGE LEFT", "LANE CHANGE RIGHT"};

Planner::Planner() {
  current_state = Ego_State::follow_vehicle_in_lane;
  target_lane = START_LANE;
  avg_lane_velocity = {0.0, 0.0, 0.0};
  too_close_vehicles = {std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};
  ego_key = -1;
}

Planner::~Planner() {
}

// Add the ego vehicle object with ego_key
void Planner::add_ego(double x, double y, double s, double d, double yaw, double velocity) {
  ego = Vehicle();
  ego.id = ego_key;
  ego.x = x;
  ego.y = y;
  ego.s = s;
  ego.d = d;
  ego.yaw = yaw;
  ego.lane = get_lane(d);
}

//Returns the ego vehicle object
Vehicle Planner::get_ego() {
  return this->ego;
}

//Prepares a vector of Vehicle objects representing the state of other traffic participants
void Planner::add_other_traffic_participants (vector<vector<double>> sensor_fusion) {
  for (int i = 0; i < sensor_fusion.size(); i++) {
    double d = sensor_fusion[i][6]; 
    int lane = get_lane(d);
    if (lane >=0 && lane <3) { // only add vehicles in valid lanes
      Vehicle *vehicle = new Vehicle(sensor_fusion[i][0], sensor_fusion[i][1],
                              sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4], 
                              sensor_fusion[i][5], sensor_fusion[i][6]);
      update_vehicle(vehicle);
    }
  }
}

// Includes the given non-ego vehicle to the other_vehicles vector. If the non-ego is already
// available, it is deleted and replaced with the new object 
void Planner::update_vehicle(Vehicle *vehicle) {
  auto iterator = other_vehicles.find(vehicle->id);
  // if the vehicle is already avaialble, point the address to the new
  // object. delete the previous object
  if (iterator != other_vehicles.end()) {
    delete (*iterator).second;    
    (*iterator).second = vehicle;
  } else { //vehicle not found. Add a new vehicle to the map
    other_vehicles.insert(pair<int, Vehicle *>(vehicle->id, vehicle));
  }
}

// Returns the count of non-ego vehicles + 1 ego vehicle 
int Planner::get_vehicle_count() {
  return other_vehicles.size();
}

// Sets the x, y and s points of the waypoints from the map 
void Planner::set_waypoints(vector<double> &map_x, vector<double> &map_y, vector<double> &map_s) {
  map_waypoints_x = map_x;
  map_waypoints_y = map_y;
  map_waypoints_s = map_s;
}

void Planner::update_vehicle_positions() {
  for (int i=0; i < too_close_vehicles.size(); i++) {
    vector<float> positions;
    for (auto &obj: other_vehicles) {
      if (obj.second->lane == i) {
        positions.push_back(obj.second->s_diff);
      }      
    }
    if ( positions.size() > 0 )
    {
      vector<float>::iterator min_dist = std::min_element(std::begin(positions), std::end(positions));
      int best_idx = std::distance(begin(positions), min_dist);
      too_close_vehicles[i] = positions[best_idx];      
    }
  }
}

// Predicts the position of the ego and non-ego vehicles after a time interval.
void Planner::predict(double delta_t){
  calculate_lane_velocities();

  ego.update_kinematics(delta_t);
  for (auto &obj: other_vehicles) { // iterate all the non-ego vehicles
    obj.second->update_kinematics(delta_t);
    // update the situation context of Ego vehicle
    obj.second->s_diff = fabs(obj.second->s - ego.s);
  }
  // Update the vehicle collision metrics. Required for Safety cost estimation.
  update_vehicle_positions();
}

// Remove the non-ego vehicle objects after a cycle of prediction and update.
void Planner::remove_other_vehicles() {
  //remove all the previous vehicles
  for (auto &obj: other_vehicles) {
    delete obj.second;
  }
  other_vehicles.clear();
}

// Return true if there is atleast one vehicles within 30m (configurable) ahead and behind the
// ego vehicle in the current lane
bool Planner::is_lane_clear (int lane) {
  bool result = true;
  for (auto obj: other_vehicles) {
    if (obj.second->lane == lane) {
#ifdef DEBUG
      if ( abs( ego.s - obj.second->s ) < SAFETY_DISTANCE) {
        cout << "Vehicle " << obj.second->id << " is @ s = " 
             << obj.second->s << " Ego.s is " << ego.s << endl;
      }
#endif
      result &= abs(obj.second->s - ego.s) > SAFETY_DISTANCE;
    }
  }
  return result;
}

// Returns the pointer to the nearest vehicle ahead of the ego vehicle
// Retruns null if there are no vehicle ahead.
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

// Returns the pointer to the nearest vehicle behind the ego vehicle
// Returns null if there are no vehicle behind.
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

// Based on the current state, execute the relevant action
Ego_State Planner::execute_next_state() {
  Ego_State new_state = Ego_State::invalid_state;
  switch (current_state) {
    case Ego_State::follow_vehicle_in_lane:
      new_state = execute_followlane();
      break;
    case Ego_State::lanechange_left:
      new_state = execute_left_lanechange();
      break;
    case Ego_State::lanechange_right:
      new_state = execute_right_lanechange();
      break;
    default:
      cout << "********INVALID STATE********" << endl;
      break;
  }
  if (new_state != current_state) {
    current_state = new_state;
  }
  return current_state;
}

// Returns true if the lane is 0, 1 or 2
bool Planner::is_lane_valid(int lane){
  return (lane >= 0) && (lane <3);
}

vector<Ego_State> Planner::successor_states() {
  vector<Ego_State> states;
  states.push_back(Ego_State::follow_vehicle_in_lane);
  if (ego.lane != 0){
    states.push_back(Ego_State::lanechange_left);
  } 
  if (ego.lane < 2){
    states.push_back(Ego_State::lanechange_right);
  }
  return states;
}

// Executes the follow_lane state. This is the default state for the ego.
// If ego vehicle within 30 meters (configurable) in front a vehicle, it
// evaluates alternate states and a cost for each of the possible states.
// The state with the lowest cost is returned for the next execution cycle.
Ego_State Planner::execute_followlane() {
  //get possible trajectories for the current state
  vector<Ego_State> states = successor_states();
  
  // Find the lane with the fastest "lane" velocity
  /*vector<double>::iterator best_cost = std::max_element(std::begin(avg_lane_velocity), std::end(avg_lane_velocity));
  int best_idx = std::distance(begin(avg_lane_velocity), best_cost);
  cout << "Lane velocities are : " << avg_lane_velocity << " ==> Max velocity at idx : " << best_idx << endl;*/
  double best_cost = 0.0;
  int index = calculate_best_cost( *this , best_cost); 
  
  Ego_State new_state = Ego_State::follow_vehicle_in_lane;
  Vehicle *ahead = get_vehicle_ahead(ego.lane);
  if (ahead) { 
    if ((ahead->s > ego.s) && ((ahead->s - ego.s) < SAFETY_DISTANCE)) //TODO check for vehicles from behind
    { 
#ifdef DEBUG
      cout << "vehicle " << ahead->id << " ahead within a reaching distance of  " << (ahead->s - ego.s) << endl;
      cout << "checking for vehicle on left lane with lane index " << ego.lane-1 << endl; 
#endif     
      if (is_lane_valid(ego.lane-1) && is_lane_clear(ego.lane-1)) { 
        target_lane = ego.lane-1;
        ego.v_magnitude = ahead->v_magnitude; // TODO: Check this ??
        return Ego_State::lanechange_left;
      } 
#ifdef DEBUG
      cout << "left lane not clear. checking for vehicle on right lane with lane index " << ego.lane+1 << endl;
#endif
      if (is_lane_valid(ego.lane+1) && is_lane_clear(ego.lane+1)) {
        target_lane = ego.lane+1;
        return Ego_State::lanechange_right;
      } else {
#ifdef DEBUG
        cout << "no free lanes to change. slow down in current lane .. " << endl;
#endif
        // no options to change lanes. stay in the same lane, but decelerate
        ego.v_delta -= MAX_ACCL;
        return Ego_State::follow_vehicle_in_lane;
      }
    }
    else { // Vehicles are ahead, but at a safe distance. Accelerate in the currrent lane
      ego.v_delta += MAX_ACCL;
    }    
  } else { // No vehicles ahead. Accelerate in the currrent lane  
    ego.v_delta += MAX_ACCL;
    new_state = Ego_State::follow_vehicle_in_lane;
  }
  return new_state;
}

// Executes the left lane change state as long as the current lane is same as the intended lane.
Ego_State Planner::execute_left_lanechange(){
  if (ego.lane == target_lane){
    return Ego_State::follow_vehicle_in_lane;
  } else {
    return Ego_State::lanechange_left;
  }
}

// Executes the right lane change state as long as the current lane is same as the intended lane.
Ego_State Planner::execute_right_lanechange(){
  if (ego.lane == target_lane){
    return Ego_State::follow_vehicle_in_lane;
  } else {
    return Ego_State::lanechange_right;
  }
}

// Dervice the next position of the ego car based on the previous paths.
// Depending on the trajectory, the ego's motion parameters are updates. 
// Logic based on the Q&A session in Project Highway Driving with David Silver and Aaron Brown
void Planner::prepare_trajectory (vector<double> previous_path_x, vector<double> previous_path_y, 
                                               double end_path_s, double &reference_velocity) {
  // STEP 1: Prepare a vector of anchor points which are coarsely 
  // covering the ego vehicle trajectory.
  vector<double> anchor_points_x;
  vector<double> anchor_points_y;
  
  next_x_vals.clear();
  next_y_vals.clear();
  
  double ref_x = ego.x;
  double ref_y = ego.y;
  double ref_yaw = deg2rad(ego.yaw);
  
  //previous_path_x and _y are the residual points which were not consumed by the simulator in the
  //previous run. 
  int prev_size = previous_path_x.size();
  if ( prev_size < 2 ) {
      anchor_points_x.push_back(ego.x - cos(ego.yaw));
      anchor_points_x.push_back(ego.x);

      anchor_points_y.push_back(ego.y - sin(ego.yaw));
      anchor_points_y.push_back(ego.y);
  } else {
      ref_x = previous_path_x[prev_size - 1];
      ref_y = previous_path_y[prev_size - 1];
      double ref_x_prev = previous_path_x[prev_size - 2];
      double ref_y_prev = previous_path_y[prev_size - 2];
      ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

      anchor_points_x.push_back(ref_x_prev);
      anchor_points_x.push_back(ref_x);

      anchor_points_y.push_back(ref_y_prev);
      anchor_points_y.push_back(ref_y);
  }
  
  vector<double> next_wp0 = getXY(ego.s + 30, (2 + (4*target_lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(ego.s + 60, (2 + (4*target_lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(ego.s + 90, (2 + (4*target_lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  anchor_points_x.push_back(next_wp0[0]);
  anchor_points_x.push_back(next_wp1[0]);
  anchor_points_x.push_back(next_wp2[0]);

  anchor_points_y.push_back(next_wp0[1]);
  anchor_points_y.push_back(next_wp1[1]);
  anchor_points_y.push_back(next_wp2[1]);
  
  // STEP 2: Transformation the 5 coarse points to local car coordinates
  // shift the ego reference angle to 0 degrees
  for ( int i = 0; i < anchor_points_x.size(); i++ ) {
    double shift_x = anchor_points_x[i] - ref_x;
    double shift_y = anchor_points_y[i] - ref_y;

    anchor_points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
    anchor_points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
  }
  
  // STEP 3: Prepare the Spline object and pass on the anchor points
  tk::spline spline;
  spline.set_points(anchor_points_x, anchor_points_y);

  for ( int i = 0; i < prev_size; i++ ) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  //STEP 4: Prepare fine granular points based on the coarse points defined in Step 1
  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for( int i = 1; i < PREDICTION_HORIZON - prev_size; i++ ) {
    reference_velocity += ego.v_delta;
    if ( reference_velocity > MAX_SPEED ) {
      reference_velocity = MAX_SPEED;
    } else if ( reference_velocity < MAX_ACCL ) {
      reference_velocity = MAX_ACCL;
    }

    double N = target_dist/(CYCLE_TIME * reference_velocity / FACTOR_MPH_TO_MS);
    double x_point = x_add_on + target_x/N;
    double y_point = spline(x_point);

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

void Planner::calculate_lane_velocities() {
  avg_lane_velocity = {0.0, 0.0, 0.0};
  vector<double> sum_velocities = {0.0, 0.0, 0.0};
  vector<int> number_vehicles = {0, 0, 0};

  for (auto obj: other_vehicles) {
    double delta_s = obj.second->s - ego.s;
    if (obj.second->lane != ego.lane && delta_s > 0.0 && delta_s < 100) {
      sum_velocities[obj.second->lane] += obj.second->v_magnitude;
      number_vehicles[obj.second->lane] += 1;
    }
  }

  // determine ego lane velocity on vehicle ahead only
  Vehicle *ahead = get_vehicle_ahead(ego.lane);

  if (ahead) {
    sum_velocities[ego.lane] = ahead->v_magnitude;
  } else {
    sum_velocities[ego.lane] = MAX_SPEED;
  }
  number_vehicles[ego.lane] = 1;

  for (int i=0; i<avg_lane_velocity.size(); i++) {
    if ( number_vehicles[i] == 0 ) {
      avg_lane_velocity[i] = MAX_SPEED;
    } else {
      avg_lane_velocity[i] = std::min (MAX_SPEED, sum_velocities[i] / number_vehicles[i]);
    }
  }
}