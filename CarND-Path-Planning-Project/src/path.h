#ifndef PATH_H
#define PATH_H

#include <string>
#include <vector>

using namespace std;

#define SMALL_PATH        2
#define FULL_PATH         50
#define DT                0.02
#define MAX_SPEED         48.
#define MPS_TO_MPH        2.24
#define TRAILING_VEL      5.0 // mph
#define FRONT_BUFFER              55. // meters; 
#define FRONT_TOO_CLOSE           30.
#define LANE_CHANGE_BUFFER_FRONT  25. // meters
#define LANE_CHANGE_BUFFER_BACK   15.
#define THROTTLE          .2 // mph

#define SPACING      30.    // Spacing (m) in waypoints

#define LANE_WIDTH         4. // in d-coordinate
#define TRACK_LENGTH      6945.554
#define SOMETHING_BIG TRACK_LENGTH

int getLaneID(double car_d);

string getLaneString(int lane_id);

bool isLaneChangeSafe(int num_points,     // number of planning points
                      double self_s,      // own-vehicle s
                      double self_v,      // own-vehicle reference velocity
                      int lane_to_check,  // own-vehicle lane
                      vector<vector<double> > sensor_fusion);

#endif //PATH_H
