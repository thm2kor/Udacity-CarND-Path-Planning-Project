#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <numeric>

using std::string;
using std::vector;

// Weights for costs
const int COST_SAFETY      = 100000; // 
const int COST_EFFICIENCY  = 1000; // 


double safety_cost( const Planner &planner , int index) {
  // penalizes driving behaivour which are closer to other vehicles
  // SAFETY_DISTANCE of 30 m (configurable)
  double cost = 0.0;
  vector<double> data = planner.too_close_vehicles;

  // run the logistic funtion distance difference between current lane and 
  // the lane with too many vehicles
  /*if (data[index] < SAFETY_DISTANCE) {
    cost = 1.0;
  } else {
    
  }*/
  cost = 1.0 / (1 + exp(data[index] - SAFETY_DISTANCE)) ;
  return cost;
}

double efficiency_cost( const Planner &planner , int index ) {
  // penalizes lanes with low average velocities
  double cost = 0.0;
  /*vector<double> data = planner.avg_lane_velocity;
  // find the index of the min element
  vector<double>::iterator min = std::min_element(std::begin(data), std::end(data));
  int idx_min = std::distance(begin(data), min);
  
  cost = logistic ((data[index] - data[idx_min]) / MAX_SPEED );*/
  return cost;
}

double calculate_total_cost( const Planner &planner , int index) {
  double cost = 0.0;
  vector<std::function<float(const Planner &, int )>> cf_list = {safety_cost, efficiency_cost};
  vector<float> weight_list = {COST_SAFETY, COST_EFFICIENCY};
  
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](planner, index);
    cost += new_cost;
  }  
  return cost;
}

