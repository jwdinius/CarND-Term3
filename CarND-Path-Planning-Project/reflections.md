## Reflections on Path Planning Project

### Difficulty
This project forced me to utilize the forums and resources much more than any project so far.  I believe the project introduction was good, however the lectures lacked the clear presentation of previous sections.

### General
The objective is to navigate a self-driving car around a track as quickly as possible while remaining both safe and comfortable for passengers riding in the vehicle.  Multiple vehicles exist in the roadway which must be accounted for when planning a path along the track.  This work addresses the following:
* Safety
* Comfort
* Minimizing time around the track by executing safe lane changes

Not addressed:
* Optimality using heuristic costs associated with state transitions at a given state

### Specifics
The main routine (```GetBestTrajectory```) is contained within the ```Plan``` class in ```plan.cpp```.  This is where most of the work regarding path planning is performed.

A path for 80 timesteps (each at 0.02 sec) is generated into the future that navigates a track while trying to minimize time around the track while not exceeding some maximum speed (50 mph).  There are many cars along the road that act as obstacles against achieving the desired objective.  To safely navigate around them, the planning logic keeps a buffer distance between vehicles and determines if a lane change would be effective and safe.  If both, a smooth lane change is planned and executed using [cubic splines](https://github.com/ttk592/spline). 

There is the case where something unexpected, and potentially dangerous, is unfolding (like another vehicle executing an unsafe lane change).  When this happens, the vehicle needs to make a quick corrective action, e.g. emergency braking.  In this case, a path length of 80 steps is much too long to execute such a maneuver.  Therefore, in the event of emergency, the path length of the planner is shortened to 10% of the original (8), and the path is saved to a special buffer.

The other objectives are concerned with comfort mostly; trajectories are chosen that avoid excessive jerk and acceleration.  I suppose argument could be made that both jerk and acceleration are safety factors as well.

Everything is done in Frenet coordinates to account for curvature in the road; lane keeping is much easier to define in these coordinates.  To avoid conflict within ```main.cpp```, a conversion to Cartesian coordinates is performed.

A simple state machine is integrated within ```Plan``` that executes transitions between states (lane keep and lane change).  Transitions account for limitations on maximum allowed acceleration (```#define AMAX```).  A check is performed to make sure that the lane is clear before allowing a transition to change into that lane (see ```path.cpp:184```).

Setting the maximum speed to 50 mph (converted to the simulator mks units) resulted in maximum velocity violations (due most likely to a simple numerical issue), so the maximum speed was throttled back to 48 mph and no issues were observed.

Splines are used to smooth out lane changes and it works really well; lane changes appear as if they would be very comfortable if I was a passenger in the car.  The interpolation is performed over an about 2.5 second period (see ```path.cpp:230```) where the start and final states are the center line of the current and future lanes, respectively.  The idea for using splines, and the actual package to use, came from another Github user (I'd reference here, but I can't remember the name).

