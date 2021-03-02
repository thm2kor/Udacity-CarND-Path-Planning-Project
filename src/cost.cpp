#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>


using std::string;
using std::vector;

// Weights for costs
const int COST_SAFETY      = 10000; // 
const int COST_EFFICIENCY  = 1000; // 

float safety_cost( const Planner &planner ) {
  float cost = 0.0;
  return cost;
}

float efficiency_cost( const Planner &planner ) {
  float cost = 0.0;
  return cost;
}

int calculate_best_cost( const Planner &planner , double &result) {
  float cost = 0.0;
  vector<std::function<float(const Planner &)>> cf_list = {safety_cost, efficiency_cost};
  vector<float> weight_list = {COST_SAFETY, COST_EFFICIENCY};
  
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](planner);
    cost += new_cost;
  }
  
  result = cost;
  return 0;
}

