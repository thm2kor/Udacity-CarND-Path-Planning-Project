#ifndef CONFIG_H
#define CONFIG_H

#include <math.h>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <iostream>
#include "vehicle.h"
#include <map>
// for convenience
using std::string;
using std::vector;

enum Ego_State {
  invalid_state               = 0,
  follow_vehicle_in_lane      = 1, 
  lanechange_left             = 2, 
  lanechange_right            = 3
};

enum lanes {
  left      = 0,
  center    = 1,
  right     = 2,
};

// debug functions for dumping the objects
std::ostream& operator<<(std::ostream& strm, const Vehicle &a);
std::ostream& operator<<(std::ostream& strm, std::map<int, Vehicle> &vehicles);
std::ostream& operator<<(std::ostream& strm, std::map<int ,vector<Vehicle>> &trajectories);
std::ostream& operator<<(std::ostream& strm, vector<Vehicle> &vehicles);
std::ostream& operator<<(std::ostream& strm, std::pair<std::vector<double>, std::vector<double>> points);
std::ostream& operator<<(std::ostream& strm, const vector<float> &d);
std::ostream& operator<<(std::ostream& strm, const vector<double> &d);
std::ostream& operator<<(std::ostream& strm, const vector<Ego_State> &d);

double deg2rad(double x); 
double rad2deg(double x); 

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) ;

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) ;

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) ;
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) ;
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) ;

double logistic(double x);
//convert speed between SI and US metric systems
double mph_to_ms(double mph);
double ms_to_mph(double ms); 

// return lane index based on the d frenet value
int get_lane(double d);

// Global Constants for the Project
// Road parameters
const int LANE_WIDTH = 4;
// Lane Index
const int LANE_LEFT = 0;
const int LANE_CENTER = 1;
const int LANE_RIGHT = 2;

const int START_LANE = LANE_CENTER; 

// Motion parameters
const double MAX_SPEED = 49.5; // speed limit in miles
const double MAX_ACCL = 0.224; // 
const double MAX_DECL = 0.224; // 


const int SAFETY_DISTANCE = 30;
static constexpr double CYCLE_TIME = 0.02; 
const int PREDICTION_HORIZON = 50; 
static constexpr double PREDICTION_TIME = CYCLE_TIME * (double) PREDICTION_HORIZON;
const double FACTOR_MPH_TO_MS =2.24;

#endif  // CONFIG_H