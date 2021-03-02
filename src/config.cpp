#include "config.h"

std::ostream& operator<<(std::ostream& strm, const Vehicle &a) {
  return strm << "Vehicle(id " << a.id << ") in (lane = " 
              << a.lane << ") x= (" << a.x << ") y= (" << a.y << ") (s= " << a.s << ") (v= " 
              << a.v_magnitude << ")" ;
}

std::ostream& operator<<(std::ostream& strm, std::map<int, Vehicle> &vehicles) {
  map<int, Vehicle>::iterator it = vehicles.begin();
  while (it != vehicles.end()) {
    strm << it->second << std::endl;
    ++it;
  }
  return strm << "---------------------" ;
}

std::ostream& operator<<(std::ostream& strm, std::map<int ,vector<Vehicle>> &trajectories) {
  map<int ,vector<Vehicle>>::iterator it = trajectories.begin();
  while (it != trajectories.end()) {
    vector<Vehicle> vehicles = it->second;
    for (int i=0;i<vehicles.size();i++){
      strm << vehicles[i] << std::endl;
    }
    ++it;
  }
  return strm << "---------------------" ;
}

std::ostream& operator<<(std::ostream& strm, vector<Vehicle> &vehicles) {
  for (int i=0;i<vehicles.size();i++){
    strm << vehicles[i] << std::endl;
  }
  return strm << "---------------------" ;
}

std::ostream& operator<<(std::ostream& strm, std::pair<std::vector<double>, std::vector<double>> points) {
  for (int i=0;i<points.first.size();i++){
    strm << points.first[i] << " , " <<points.second[i] << std::endl;
  }
  return strm << "---------------------" ;
}

std::ostream& operator<<(std::ostream& strm, const vector<float> &d) {
  strm << "[" ;
  for (int i=0;i<d.size();i++) {
    strm << d[i];
    if (i != d.size() - 1)
         strm << ", ";
  } 
  return strm << "]" << std::endl ;
}

std::ostream& operator<<(std::ostream& strm, const vector<double> &d) {
  strm << "[" ;
  for (int i=0;i<d.size();i++) {
    strm << d[i];
    if (i != d.size() - 1)
         strm << ", ";
  } 
  return strm << "]";
}

std::ostream& operator<<(std::ostream& strm, const vector<Ego_State> &d) {
  strm << "[" ;
  for (int i=0;i<d.size();i++) {
    strm << d[i];
    if (i != d.size() - 1)
         strm << ", ";
  } 
  return strm << "]";
}

double logistic(double x) {
  // A function that returns a value between 0 and 1 for x in the range[0, infinity] and - 1 to 1 for x in 
  // the range[-infinity, infinity]. Useful for cost functions.
  return 2.0 / (1 + exp(-x)) - 1.0;
}

//convert speed between SI and US metric systems
double mph_to_ms(double mph) { 
  return mph / 2.24; 
} 

double ms_to_mph(double ms) { 
  return ms * 2.24; 
} 

// return lane index based on the d frenet value
int get_lane(double d) { 
  // return (int)(d / LANE_WIDTH);
  int result = -1;
  if ( d > 0 && d < 4 ) {
    result = 0;
  } else if ( d > 4 && d < 8 ) {
    result = 1;
  } else if ( d > 8 && d < 12 ) {
    result = 2;
  } 
  return result;
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}
