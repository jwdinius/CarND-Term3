## Reflections on Path Planning Project

### General
The objective is to navigate a self-driving car around a track as quickly as possible while remaining both safe and comfortable for passengers riding in the vehicle.  Multiple vehicles exist in the roadway which must be accounted for when planning a path along the track.  This work addresses the following:
* Safety
* Comfort
* Minimizing time around the track by executing safe lane changes

Not addressed:
* Optimality using heuristic costs associated with state transitions at a given state

### Specifics
The main routine within ```main.cpp``` executes 5 steps in the trajectory planning process.
* Setup
* Process sensor fusion data
* Lane change logic
* Trajectory generation (local coordinates)
* Generate global path

#### Setup
This step does simple processing of data coming in from the simulator.  When available, path data is used to generate (smoothed) states for future processing.

#### Process sensor fusion data
For each vehicle, figure out if it's in the same lane our car is currently in.  If so, we can modulate the speed to avoid getting too close.  Also, based upon these data, we can figure out whether or not to initiate a lane change.  If the car is inside of some distance (```FRONT_TOO_CLOSE```) to another vehicle in front, lane changing logic will be called.

#### Lane change logic
If a lane change is triggered, the default check is to see if it is safe to change lanes to the left.  This follows the typical driving convention of trying to pass slower vehicles on the right.  However, if such a lane change would violate safety considerations (other leading and/or trailing vehicles being too close), a lane change to the right is checked.  If it is safe, while the left is not, a right lane change is initiated.  The output of this logic is a desired lane.

#### Trajectory generation
A smooth path is planned using [cubic splines](https://github.com/ttk592/spline).  Multiple points are generated out towards the horizon such that the host car will achieve it's desired lane quickly and without excessive acceleration or jerk.  The knot points are first generated globally and then transformed to the car coordinate system before executing the spline interpolator.

#### Generate global path
The previous path is used to generate the first part of the new path.  This ensures continuity between new and old paths.  The desired path has a specific size (```FULL_PATH```) and so all remaining points in the planned trajectory (total ```FULL_PATH - previous_path_size```) will be generated using the spline interpolator applied to projected x points.
