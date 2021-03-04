# Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Objective
The objective of this project is to  design a path planner that is able to create smooth, safe paths for a car to follow along a 3 lane highway with traffic. Udacity provided a [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2) which provides the car's localization and sensor fusion data. In addition, there is a sparse map list of waypoints around the highway.

### Performance requirements
1. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.
2. The car should be able to make one complete loop around the 6946m highway.
3. The car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Implementation
### Planner
The `Planner` object is the central place where the next position of ego vehicle coordinates are predicted. It holds a map of pointer to `Vehicle` objects indexed by an unique identifier. The `Vehicle` object represents each moving vehicle from the Simulator. The kinematic motion parameters of every vehicle is saved in this object.

#### Setting Enviornment Data
1. The simulator sends the localization information ( `x` and `y` position in cartesian and `s` and `d` in frenet coordinates, `yaw` and `speed`) and unconsumed data points (`previous_path_x` and `previous_path_y`) from the previous run to the ego vehicle. The previous list's last point in frenet coordinates (`end_path_s` and `end_path_d`) are also sent by the simulator.

2. The simulator also sends the information about the cars on the right-hand side of the road. The `sensor_fusion` variable contains all the information about the cars on the right-hand side of the road. The data format for each car is: `[ id, x, y, vx, vy, s, d]`. The id is a unique identifier for that car. The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map. Finally s and d are the Frenet coordinates for that car.

3. In addition, the template code provided by Udacity loads the map data from [highway_map.csv](./data/highway_map.csv) which consists of a list of waypoints that go all the way around the track. The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow dividing line in the center of the highway. Each waypoint in the list contains ``[x,y,s,dx,dy]`` values. `x` and `y` are the waypoint's map coordinate position, the `s` value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

3. For every 20ms (`𝛿t`), the data mentioned above are copied to the Planner with the following functions:
```c++
void add_ego(double x, double y, double s, double d, double yaw, double velocity);
void add_other_traffic_participants (vector<vector<double>> sensor_fusion);
void set_waypoints(vector<double> &map_x, vector<double> &map_y, vector<double> &map_s);
```

#### Predicting future position of the vehicles
The new position of the ego vehicle and other traffic is estimated using the kinematic equation: Pos<sub>new</sub> = Speed<sub>current</sub> * 𝛿t
```c++
void Vehicle::update_kinematics(double delta_t) {
  // future predicted Frenet s value will be its current s value plus its
  // (transformed) total velocity (m/s) multiplied by the time elapsed
  this->s+= v_magnitude * delta_t;
}
```
#### Finite State Machine
To decide the next path position, the planner objects executes a finite state machine with 3 states:
1. Keep lane
2. Lane Change to left
3. Lane Change to right

##### Keep Lane
This is the default state of the program. The ego vehicle starts with a `current_lane = 1`, which is the centre lane. The ego vehicle will continue to accelerate in the current lane until it comes close to other vehicle in the same lane. The **safety distance is configured as 30m**. The acceleration of the ego vehicle will continue until the vehicle reaches a speed of 49.5 mph. The triggers and action for the valid successor states are summarized in the below table:

| State from | State to | Trigger | Action |
| ------ | ------ | ----- | ----- |
| Keep Lane | Keep Lane | No vehicles within the safety distance | `target_lane = current_lane` |
| Keep Lane | Change Left | 1. Ego vehicle is within the safety distance in the current lane AND 2. There are no other vehicles in the left lane within the safety distance  | `target_lane = ego.lane - 1` |
| Keep Lane | Change Right | 1. Ego vehicle is within the safety distance in the current lane AND 2. There are no other vehicles in the right lane within the safety distance | `target_lane = ego.lane + 1` |

##### Lane Change to left/right
Once the vehicle in these states, it continues in this state, until the lane change is completed.The triggers and action for the valid successor states are summarized in the below table:

| State from | State to | Trigger | Action |
| ------ | ------ | ----- | ----- |
| Change Left/Right | Change Left/Right | `target_lane != current_lane` | `target_lane = current_lane` |
| Change Left/Right | Keep Lane | Successful execution of the lane changes  |  |

#### Prepare the trajectory
The final step in the path planning is the preparation of the trajectory. The previous step identifies the `new s` coordinates. The trajectory is built using the [spline library](http://kluge.in-chemnitz.de/opensource/spline/) with in a 2 step approach.
1. In the first step, a set of 5 coarse points are defined. 2 from the previous run and 3 additional points which are 30m, 60m and 90 m from the current vehicle position.
2. In the second step, a set of fine granular future points (50) are derived using the spline functions, which helps to generate jerk-free trajectories.
3. To simplify the spline calculation based on those points, the coordinates are transformed (shift and rotation) to local car coordinates
This logic is based on helpful video in Project Highway Driving with David Silver and Aaron Brown.

## Result
The vehicle is able to complete the track length of 4.32 miles without any incidents. The ego keeps the speed limits of 49.5 mph and does not exceed the total acceleration of 10 m/s^2 and a jerk of 10 m/s^3. The recording of the result could be found [here](https://www.youtube.com/watch?v=2Hg2GHGQWIE). 

## Reflection

1. So far, this is one of the most complex project which I had faced in this course. Currently the path planning is done only considering the safety costs. Considering the scope of the program, i released the first version with only the safety cost. In the next versions, i will include the efficiency cost.
2. The current Implementation already supports the data required for the calculation of the efficiency cost. During the prediction step, two vectors are prepared `(vector<double> avg_lane_velocity; vector<double> too_close_vehicles;`). The average velocity of the lanes would be used to determine the costs of efficiency. My first implementation led to frequent lane changes. So i have disabled it. I need to play around with the weights of safety and efficiency costs to finalize the implementation.

---
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

---

### Additional information

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single header file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
