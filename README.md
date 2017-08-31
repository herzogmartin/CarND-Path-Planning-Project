# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulator provides the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

The following video shows, that the code archives the goals:

[![Path Planning](http://img.youtube.com/vi/td4sobZFshM/0.jpg)](http://www.youtube.com/watch?v=td4sobZFshM "CarND Path Planning Project")

### Simulator.
The Term3 Simulator to run the Path Planning Project can be downloaded from [releases tab] (https://github.com/udacity/self-driving-car-sim/releases).


## Description of the code
The written code for the project is in `main.cpp`.
The three following main functionalities are implemented:

1. Follow lane.
2. Setting of target speed.
3. Change lane if the speed on a different lane is faster.

### Follow lane

The map of the highway, the ego car's localization data and the remaining planned path is used to calculate 
a smooth trajectory for following the ego lane.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Ego car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH


#### Previous path data given to the Planner

This is the path given to the simulator which is not yet driven.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value


The idea is to follow the waypoints of the highway in Frenet coordinates. With this 
approach it is easy to stay on the lane, because the "d"-value is fix for every lane.
The lanes in the code are numbered the following way:

* lane 0 is left lane
* lane 1 is middle lane
* lane 2 is right lane

Every lane has a width of 4 m. Therefore the d-value in Frenet coordindates can be calculated 
in the way `2 + 4*targetLane`. With this calculation the car stays in the middle of
the lane number `targetLane`. In the beginning targetLane is set to `1`, because the simulator starts
with the car in the middle lane.

To generate a smooth path two problems must be addressed.

First there is a latency for running the simulator and recalculating a new path. 
The simulator is driving the path which was send before. Each point of the path has a
time gap of 20 ms. To ensure that always points for the simulator are available the path which
is sent has a length of `NUM_PATH_PTS`. `NUM_PATH_PTS` is set to 50 by default. This 
corresponds to 1s. 

The latency is not known therefore the new path is always reusing the remaining points of the 
previous path. The new path extends the previous path to get `NUM_PATH_PTS` number of points.
This is done in `main.cpp` lines 527 - 563. The extension of the path is planned in local 
coordinates from the last point of the previous path and then transformed to global coordinates.
The step-size in local x is set in such a way that the `ego_targetSpd` is driven. Within every 
step of the new path the `ego_targetSpd` is changed towards `target_spd`. The allowed change can 
be defined with `MAX_A_SPEEDUP` (for acceleration), `MAX_A_SPEEDDOWN` (for deceleration) and
`A_EMERGENCY` (for emergency braking).

The second problem is, that the path needs to be extended smoothly. Therefore the waypoints can't be 
used directly, because this is a sequence of straight lines. At each waypoint the
curvature would be infinite which would result in high lateral accelerations and jerks.
Therefore a spline is planned starting from the last two points of the previous path. The calculation 
of the last two points of the previous path together with the orientation's yaw angle 
can be found in lines 438 - 489. 
The spline has another NUM_SPLINE_PTS (default 3) settle points with a distance of DIST_SPLINE_PTS 
(default 30 m). The Frenet's coordinates of the 3 settle points are converted to xy global 
coordinates. These points are converted to local coordinates from the last point of the previous path. 
This has the advantage that the extension of the path is easier. In the code this can be found in 
lines 491 - 514.

### Setting of target speed
If the ego lane is not occupied, the `ego_targetSpd` is set to TARGET_SPD (default 49.5 mph).
If the ego lane is occupied by another vehicle the `ego_targetSpd` is set to ego_lane_spd. 
(See lines 380 - 390).
The ego lane is considered as occupied (lines 322 - 327), if there is another vehicle touching the ego lane
in front of the car within a time-gap of 1.5s. The width of the other vehicle is estimated
with 2 m and the ego lane with a width of 4m.
`ego_targetSpd` is calculated as the minimum speed of all other vehicles which are in the area 
which defines the ego lane to be occupied. If the time-gap is below 1s than `ego_targetSpd`
is reduced by 1 m/s to fall back. It the time-gap is below 0.5s than `ego_targetSpd`
is reduced by another 2 m/s to fall back faster (so in total with 3 m/s) and the 
flag `ego_lane_emergency` is set to allow higher decelerations.

### Lane Change
The goal is to do a lane change to the fastest lane if the speed on the current lane is too slow.

Therefore the minimum speed per lane of all cars from the fusion which are in front of the ego, is is calculated (lines 351 - 378). From this the fastest lane index `fast_lane` is calculated (lines 392 - 397). A lane change is initiated if there is another car within a timegap of 3s in the ego lane and the target lane is not the fastest lane (line 402) . The lane change is only initiated to lanes if the lateral distance is not above 4.1 m (width of one lane is 4m). Otherwise the nearer lane is set to `fast_lane` before (lines 404 - 411). This ensures that the lanes are changes one by one. Before the lane change is performed, it is checked, that there is enough space in `fast_lane` (lines 417 - 426). In order to perform the lane change `targetLane` is set to `fast_lane`. The spline interpolation which ensures to follow a lane smoothly, also ensures, that the lane change is smooth. It is necessary that the distance between the spline points `DIST_SPLINE_PTS` is not set too small (default 30m). If it's too small the lane change would be very aggressive. 

### Open Points
The following open points are known and can be addressed for further improvement:

* The lane change is not aborted or an emergency brake is performed if a lane change of another car is done at the same time .
* The ego car can be trapped in the left or right lane if the middle lane is occupied. The ego car does not fall back and searches for a gap in the occupied middle lane.


---



## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.



## Spline calculation

The splines are calculated using the spline function from the following header file: [http://kluge.in-chemnitz.de/opensource/spline/](http://kluge.in-chemnitz.de/opensource/spline/)

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

