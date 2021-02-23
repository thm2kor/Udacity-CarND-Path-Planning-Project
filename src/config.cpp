#include "config.h"

//convert speed between SI and US metric systems
double mph_to_ms(double mph) { 
  return mph / 2.24; 
} 

double ms_to_mph(double ms) { 
  return ms * 2.24; 
} 

// return lane index based on the d frenet value
int get_lane(double d) { 
  return (int)(d / LANE_WIDTH); 
}

// return the 'd' coordinates of a given lanes
int d_left(int lane) { 
  return (double)(lane * LANE_WIDTH); 
}

int d_center(int lane) { 
  return (double)((lane + 0.5) * LANE_WIDTH); 
}

int d_right(int lane) {
  return (double)((lane + 1) * LANE_WIDTH); 
}

