#ifndef CONFIG_H
#define CONFIG_H

enum Ego_State {
  invalid_state               = 0,
  follow_vehicle_in_lane      = 1, 
  prepare_lanechange_left     = 2,
  lanechange_left             = 3, 
  prepare_lanechange_right    = 4,
  lanechange_right            = 5
};

enum lanes {
  left      = 0,
  center    = 1,
  right     = 2,
};

//const char * lane_names[] = { "LEFT", "CENTER", "RIGHT" };


//convert speed between SI and US metric systems
double mph_to_ms(double mph);
double ms_to_mph(double ms); 

// return lane index based on the d frenet value
int get_lane(double d);
// return the 'd' coordinates of a given lanes
int d_left(int lane);
int d_center(int lane) ;
int d_right(int lane) ;

// Global Constants for the Project
// Road parameters
const int LANE_WIDTH = 4;
// Lane Index
const int LANE_LEFT = 0;
const int LANE_CENTER = 1;
const int LANE_RIGHT = 2;
// Motion parameters
const double MAX_SPEED = 49.5; // speed limit in miles
const double MAX_ACCL = 0.224; // 1g in SI
const double MAX_DECL = 0.224; // 2g in SI
// Weightages for costs
const int COST_FEASIBILITY = 10000; // 
const int COST_SAFETY      = 1000; // 
const int COST_LEGALITY    = 100; // 
const int COST_COMFORT     = 10; // 
const int COST_EFFICIENCY  = 1; // 

const int HORIZON_IN_METERS = 30;
static constexpr double CYCLE_TIME = 0.02; 
static const int PREDICTION_DISTANCE = 50; 
static constexpr double PREDICTION_TIME = CYCLE_TIME * (double) PREDICTION_DISTANCE;

/*static constexpr double DEFAULT_TIME_GAP = 9999.9; // default resp. max time gap [s]
static constexpr double DEFAULT_TIME_TO_COLLISION = 9999.9; // default resp. max time-to-collision [s]
static constexpr double DEFAULT_DISTANCE = 9999.9; // default resp. max distance to vehicles [m]
static constexpr double LOWER_TIME_GAP = 1.5; // min allowed time gap to vehicle ahead [s]
static constexpr double UPPER_TIME_GAP = 3.5; // time gap [s] to drive with speed limit
static constexpr double MIN_TIME_GAP_INIT_LANE_CHANGE = 3.0; // min allowed time gap to initiate a lane change [s]
static constexpr double MIN_TIME_GAP_LANE_CHANGE = 1.0; // min required time gap to vehicle in target lane [s]
static constexpr double MIN_DISTANCE_FRONT_LANE_CHANGE = 10.0; // min required distance to vehicle ahead in target lane [m]
static constexpr double MIN_DISTANCE_REAR_LANE_CHANGE = 10.0; // min required distance to vehicle behind in target lane [m]
static constexpr double MIN_TTC_FRONT_LANE_CHANGE = 6.0; // min required time-to-collision to vehicle ahead in target lane [s]
static constexpr double MIN_TTC_REAR_LANE_CHANGE = 6.0; // min required time-to-collision to vehicle behind in target lane [s]
static constexpr double FASTEST_LANE_FACTOR = 0.08; // the fastest lane velocity need to  be x% faster than the current [%]
*/
#endif  // CONFIG_H