/////////////////////////////////////////////
//
// helper functions
//
////////////////////////////////////////////

#include <cmath>
#include <string>
#include <iostream>
#include <vector>
#include "path.h"

using namespace std;

static const vector<string> lanes = {"left", "center", "right"};

// get lane number (0 - left, 1 - center, 2 - right)
int getLaneID(double car_d) {
  int lane;
  if (car_d >= 0 && car_d <= LANE_WIDTH) {
    lane = 0;
  } else if (car_d > LANE_WIDTH && car_d <= 2.*LANE_WIDTH) {
    lane = 1;
  } else if (car_d > 2.*LANE_WIDTH && car_d <= 3.*LANE_WIDTH) {
    lane = 2;
  }
  else {
    lane = -1;
  }
  return lane;
}

string getLaneString(int lane_id) {
  if (0 <= lane_id && lane_id <= 2){
    return lanes[lane_id];
  }
  else{
    return "unk";
  }
}

bool isLaneChangeSafe(int num_points,     // number of planning points
                      double self_s,      // own-vehicle s
                      double self_v,      // own-vehicle reference velocity
                      int lane_to_check,  // own-vehicle lane
                      vector<vector<double> > sensor_fusion) {
  bool change_safe = false;

  double closest_front =  SOMETHING_BIG;
  double closest_back  = -SOMETHING_BIG;

  // Calculate the closest Front and Back gaps
  for (int i = 0; i < sensor_fusion.size(); i++) {
    int other_car_lane = getLaneID(sensor_fusion[i][6]); // lane of the Traffic Car
    // if a car is in the lane_to_check
    if (other_car_lane == lane_to_check) {
      // get car's speed speed
      double vx    = sensor_fusion[i][3];
      double vy    = sensor_fusion[i][4];
      double speed = sqrt(vx*vx + vy*vy);
      // get it's s displacement
      double car_s = sensor_fusion[i][5];

      // propagate vehicle forward (assumes linear assumption is valid (i.e. propagation time is small))
      car_s += ((double)num_points * DT * speed);

      // see the gap from our Ego Car
      double dist_s = car_s - self_s;  // WAS: ego_car_s
      //cout << dist_s << endl;
      // find the closest cars
      if (dist_s > 0) {                  // FRONT gap
        closest_front = min(dist_s, closest_front);
      } 
      else if (dist_s <= 0) {          // BACK gap
        closest_back  = max(dist_s, closest_back);
      }
    }
  } // for-each-Traffic-car
        /*cout << "   gap (m): " 
          << " >>> Closest Front: "
          << setprecision(5)
          << SHORTEST_FRONT
          << ", closest Back: "
          << setprecision(5)
          << SHORTEST_BACK
          << " <<< "
          << endl;*/
  // Only if enough space in that lane, move to that lane
  if ( ( closest_front > LANE_CHANGE_BUFFER_FRONT)  &&
       (-closest_back  > LANE_CHANGE_BUFFER_BACK)) 
  {
    change_safe = true;
  }

  return change_safe;

}



