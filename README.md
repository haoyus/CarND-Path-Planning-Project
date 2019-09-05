# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Key rubic points check
1. The car is able to drive at least 4.32 miles without incident             --- YES
2. The car drives according to the speed limit.                              --- YES
3. Max Acceleration and Jerk are not Exceeded.                               --- YES
4. Car does not have collisions.                                             --- YES
5. The car stays in its lane, except for the time between changing lanes.    --- YES
6. The car is able to change lanes.                                          --- YES

## Key components of my approach in main.cpp
1. Slow in path vehicle detection (line 148 - 159)
This part of code uses sensor fution output to search for a slow in path vehicle.

2. Neighbor lane opening detection (line 162 - 174)
This part of code uses sensor fusion output to search for vehicles within a certain longitudinal range near Host Vehicle.

3. Finite State Machine (line 178 - 220)
This is the key algorithm of planner, it has 3 states:

        I. LaneKeep;
        II. PrepareLaneChange;
        III. DuringLaneChange.

        I is the default state, the action is to follow Speed limit. Under condition "slow in path vehicle", FSM will transit to State II.

        II will examine the following conditions:
        cond-II-a: if the car is already during a lane change. If true, transit to III with no action;
        cond-II-b: if the car is not during lane change, and lane opening available. If true, take action of lane change and transit to III.
        cond-II-c: if the car is not during lane change, and lane opening unavailable. Take default action: follow slow car.

        III will examine condition "slow in target path vehicle", if true, take action of following that car with NO additional lane change allowed. Otherwise, will transit to I.

4. Map waypoints calculator (line 222 - 263)
This part of code calculates planned way points in Global X-Y coordinates, using previous path points, incremented Frenet S from end of previous path, and Frenet D determined by target lane index, which is determined by the FSM.

5. Path points generation (line 265 - 310)
This part of code converts the Way Points from Global X-Y to Ego X-Y, then use spline fitting library to fit them in a spline. Then inside the Ego coordinate, fit a series of points by incrementing X from coordinate origin. Finally, convert these points back into Global coordinate system and push them into the Path, which is pre-loaded with unfinished path points from last cycle. The max number of point in Path vector is 50.

## TODO list
Please see code comments.


## APPENDIX: Project description and requirements
   
#### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

#### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

#### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

#### Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

#### Dependencies

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

