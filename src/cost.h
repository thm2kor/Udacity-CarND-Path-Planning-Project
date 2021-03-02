#ifndef COST_H
#define COST_H

#include "planner.h"
#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

int calculate_best_cost(const Planner &planner, double &result);
float safety_cost(const Planner &planner);
float efficiency_cost(const Planner &planner);

#endif  // COST_H