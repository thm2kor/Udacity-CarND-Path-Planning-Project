#ifndef COST_H
#define COST_H

#include "planner.h"
#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

double calculate_total_cost(const Planner &planner,  int index);
double safety_cost(const Planner &planner , int index);
double efficiency_cost(const Planner &planner , int index);

#endif  // COST_H