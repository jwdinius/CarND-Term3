#include <iostream>
#include <cmath>

#include "spline.h"

#include "predict.h"
#include "follow.h"

using namespace std;

// heading difference (eps) is used for computing s_dot and s_ddot
double Predictions::GetEps (Follow follow, double vx, double vy, double s, double d){
  vector<double> wp1 = follow.fresnetToCartesian(s+5, d);
  vector<double> wp0 = follow.fresnetToCartesian(s-5, d);
  double heading = atan2(wp1[1]-wp0[1], wp1[0]-wp0[0]);
  double veh_heading = atan2(vy, vx);
  return (veh_heading - heading);
}

void Predictions::update (Follow follow, vector<vector<double>>sensor_fusion){
  sf_vec = sensor_fusion;
  for (auto i=0; i<sensor_fusion.size();i++){
    // only consider vehicles with d>=0
    if (sensor_fusion[i][IDX_D]>=0) {
      int veh_id = (int) sensor_fusion[i][0];
      // add new vehicle if we haven't seen it before
      if (!vehicles.count(veh_id)) {
        Vehicle veh;
        vehicles[veh_id] = veh;
      }
      for (auto j = 0; j < STATES; j++) {
        // only push back actual states, not time
        vehicles[veh_id].state[j].push_back(sensor_fusion[i][j + 1]);

        if (vehicles[veh_id].state[j].size() > 2)
          vehicles[veh_id].state[j].erase(vehicles[veh_id].state[j].begin());
      }
      double vx0, vy0, s0, d0, v0, epsi0;
      vx0 = vehicles[veh_id].state[IDX_VX][0];
      vy0 = vehicles[veh_id].state[IDX_VY][0];
      s0 = vehicles[veh_id].state[IDX_S][0];
      d0 = vehicles[veh_id].state[IDX_D][0];

      // get total speed
      v0 = sqrt(pow(vx0, 2) + pow(vy0, 2));

      epsi0 = GetEps(follow, vx0, vy0, s0, d0);
      vehicles[veh_id].vs = v0 * cos(fabs(epsi0));
      if (vehicles[veh_id].state[0].size() == 1) vehicles[veh_id].vd = 0;
      vehicles[veh_id].as = 0;
      vehicles[veh_id].ad = 0;

      if (vehicles[veh_id].state[0].size() == 2) {
        // compute fresnet coordinate velocity and acceleration
        double vx1, vy1, s1, d1, v1, epsi1;
        vx1 = vehicles[veh_id].state[IDX_VX][1];
        vy1 = vehicles[veh_id].state[IDX_VY][1];
        s1 = vehicles[veh_id].state[IDX_S][1];
        d1 = vehicles[veh_id].state[IDX_D][1];

        v1 = sqrt(pow(vx1, 2) + pow(vy1, 2));
        epsi1 = GetEps(follow, vx1, vy1, s1, d1);

        double t = (s1 - fmod(s0,TRACK_LENGTH)) / (0.5 * (v1 + v0));
        vehicles[veh_id].as = (v1 * cos(fabs(epsi1)) - v0 * cos(fabs(epsi0))) / t;
        vehicles[veh_id].vs = v1 * cos(fabs(epsi1));
        vehicles[veh_id].ad = (d1-d0)/t - vehicles[veh_id].vd;
        vehicles[veh_id].vd = (d1-d0)/t;
      }
    }
  }
}

// return propagated vehicle s
double Predictions::PropagateVehicle(int vehID, double t) {
  double veh_s1 = 0;

  if (vehicles.count(vehID)) {
    Vehicle veh = vehicles[vehID];
    double veh_s0 = veh.state[IDX_S][1];
    double veh_vs0 = veh.vs;
    double veh_as = veh.as;

    veh_s1 = veh_s0 + veh_vs0 * t + veh_as * t *t * 0.5;
  }
  return veh_s1;
}
