#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"


using std::string;
using std::vector;

/**
 * TODO: change weights for cost functions.
 */
const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);

float inefficiency_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const map<int, vector<Vehicle>> &predictions, 
                        map<string, float> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane. 
  // This function is very similar to what you have already implemented in 
  //   the "Implement a Second Cost Function in C++" quiz.
  float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = vehicle.target_speed;
  }

  float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
  if (proposed_speed_final < 0) {
    proposed_speed_final = vehicle.target_speed;
  }
    
  float cost = (2.0*vehicle.target_speed - proposed_speed_intended 
             - proposed_speed_final)/vehicle.target_speed;

  return cost;
}

float lane_speed(const map<int, vector<Vehicle>> &predictions, int lane) {
  // All non ego vehicles in a lane have the same speed, so to get the speed 
  //   limit for a lane, we can just find one vehicle in that lane.
  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); 
       it != predictions.end(); ++it) {
    int key = it->first;
    Vehicle vehicle = it->second[0];
    if (vehicle.lane == lane && key != -1) {
      return vehicle.v;
    }
  }
  // Found no vehicle in the lane
  return -1.0;
}

float calculate_cost(const Vehicle &vehicle, 
                     const map<int, vector<Vehicle>> &predictions, 
                     const vector<Vehicle> &trajectory) {
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, 
                                                       predictions);
  float cost = 0.0;

  // Add additional cost functions here.
  /*vector<std::function<float(const Vehicle &, const vector<Vehicle> &, 
                             const map<int, vector<Vehicle>> &, 
                             map<string, float> &)
    >> cf_list = {goal_distance_cost, inefficiency_cost};
  vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, 
                                               trajectory_data);
    cost += new_cost;
  }
  */
  return cost;
}

