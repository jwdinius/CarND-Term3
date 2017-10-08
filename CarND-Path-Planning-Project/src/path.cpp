#include <cmath>
#include <vector>
#include <string>
#include <iostream>

#include "path.h"

using namespace std;

#define FIFTY_MPH 21.460 // 50 mph in m/s - 2mph to stay below limit
#define TIMESTEP   0.020 // 50Hz timestep
#define AMAX       7.000 // max acceleration
#define LANE_D     4.000 // lane width - d channel
#define LANE_DTOL  0.020 // lane tolerance for completion
#define LANE_TEN   0.400 // 10% wander from center lane
#define VADJ       0.500 // lateral velocity tolerance
#define BUFFER    15.000 // buffer distance to other vehicles, for safety
#define PATH_OK       80 // number of path points for all-clear scenario
#define PATH_EMERG     8 // set a shorter path for emergency braking

// constructor
Path::Path(vector<double> wp_x,
           vector<double> wp_y,
           vector<double> wp_s,
           vector<double> wp_dx,
           vector<double> wp_dy){

  plan.Init(wp_x,
            wp_y,
            wp_s,
            wp_dx,
            wp_dy);
}


// MAIN CALL
vector<vector<double>> Path::Process(double car_s,
                                     double car_d,
                                     vector<vector<double>> sensor_fusion,
                                     vector<double> previous_path_x,
                                     vector<double> previous_path_y) {


  // update predictions for planner
  plan.predictions.update(plan.follow, sensor_fusion);
  // get the actual path to follow
  vector<vector<double>> next_path_XY_points = plan.GetBestTrajectory(car_s,
                                                                    car_d,
                                                                    sensor_fusion,
                                                                    previous_path_x,
                                                                    previous_path_y);

  return next_path_XY_points;


}
// END MAIN CALL

// basically a pass-through to follow
void Plan::Init(vector<double> wp_x,
                     vector<double> wp_y,
                     vector<double> wp_s,
                     vector<double> wp_dx,
                     vector<double> wp_dy) {
  // init splines
  follow.Init(wp_x,
              wp_y,
              wp_s,
              wp_dx,
              wp_dy);
}

// get distances to cars ahead
vector<vector<double>> Plan::CarsAhead(double car_s, int lane, double behind, double length, bool collision){
  vector<vector<double>> cars_ahead;
  // buffer distance to avoid collision
  double buffer = BUFFER;
  for (auto i = 0; i < predictions.sf_vec.size(); i++){
    // get location of center line of center lane in d-coordinate
    double d_lane_center = lane * LANE_D + LANE_D/2.;
    double d_veh = predictions.sf_vec[i][IDX_D+1];
    int veh_id = predictions.sf_vec[i][0];
    double veh_ad = predictions.vehicles[veh_id].ad;

    bool cars_left = ((d_lane_center-d_veh)>=LANE_D/2. && (d_lane_center-d_veh)<(LANE_D/2.+LANE_TEN) && (veh_ad > VADJ));
    bool cars_right = ((d_lane_center-d_veh)<=-LANE_D/2. && (d_lane_center-d_veh)>-(LANE_D/2.+LANE_TEN) && (veh_ad < -VADJ));
    bool collision_adjacent = collision && (cars_left || cars_right);

    // vehicle in lane ahead
    if (fabs(d_lane_center-d_veh)<LANE_D/2.) {
      double distance = predictions.sf_vec[i][IDX_S+1] - (car_s+behind);
      if (distance < 0) distance = TRACK_LENGTH + distance;
      if (distance < length){
        cars_ahead.push_back({predictions.sf_vec[i][0], distance});
      }
    }
    // collision with adjacent vehicle predicted 
    else if (collision_adjacent) {
      double distance = predictions.sf_vec[i][IDX_S+1] - (car_s+behind);
      if (distance < 0.) distance =TRACK_LENGTH + distance;
      if (distance < buffer) {
        cars_ahead.push_back({predictions.sf_vec[i][0], distance});
      }
    }
  }
  // sorting vehicles by distance ahead
  sort(cars_ahead.begin(), cars_ahead.end(),
       [](const std::vector<double>& a, const std::vector<double>& b) {
            return a[1] < b[1];
        });
  return cars_ahead;
}

vector<string> Plan::GetNextStates(string cur_state){
  return next_states[cur_state];
}

vector<vector<double>> Plan::GetBestTrajectory(double car_s,
                                               double car_d,
                                               vector<vector<double>> sensor_fusion,
                                               vector<double> previous_path_x,
                                              vector<double> previous_path_y){

  // backup path info for when emergency condition is met
  static vector<vector<double>>path_backup = {{},{},{},{},{}};
  // set max speed to 50 mph
  static double v_max = FIFTY_MPH;
  
  // number of points along planned path
  int path_length = PATH_OK;
  
  // number of points along path in the event of a triggered emergency
  int path_emerg = PATH_EMERG;

  static bool lane_change = false;
  // create static object for lane shift spline
  static tk::spline d_t;

  // next path
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // previous path size (either 100 when everything is ok, or 10 for emergency)
  int prev_size;
  
  // distance to car ahead (in meters)
  double buffer = BUFFER;
  

  // s of last planned path point (make it persistent with static)
  static double s0 = car_s;
  double s1 = s0;
  
   //v of last planned path point (make it persistent with static)
  static double v0 = 0.;
  double v = v0;
  
  // fresnet accel
  double a; // a_s

  // time step for path points (50Hz)
  const double t = TIMESTEP; 
  
  // time step for lane change path spline generation
  static double d_time = TIMESTEP;

  // d of the last planned path point
  static double d_next = car_d; 

  // d for target lane during shift
  static double d_shift = d_next; 

  // lane of the last planned path point
  int current_lane = (int)(d_next / 4); 
  
  // target lane during shift
  static int target_lane = current_lane; 

  // id of vehicle ahead (-1 if there is none)
  double veh_id = -1;

  // cars_ahead - car's ahead in lane including improbable shift from adjacent lanes
  // this will serve as a check for the lane being clear to move into
  auto cars_ahead = CarsAhead(car_s, current_lane, 0, 30, true);

  prev_size = previous_path_x.size();
  if (cars_ahead.size()>0) {
    double distance;
    double v_veh0 = predictions.vehicles[cars_ahead[0][0]].vs;
    double a_veh0 = predictions.vehicles[cars_ahead[0][0]].as;
    distance = cars_ahead[0][1];
    v_max = min(v_veh0 + a_veh0, FIFTY_MPH);
    veh_id = cars_ahead[0][0];
    
    //emergency breaking
    if (distance < buffer ) { 
      int diff = path_length - prev_size;

      prev_size = min(path_emerg, prev_size);
      if (prev_size == path_emerg ){

        s0 = path_backup[0][path_emerg-1+diff];
        v0 = path_backup[1][path_emerg-1+diff];
        d_next = path_backup[2][path_emerg-1+diff];
        d_time = path_backup[3][path_emerg-1+diff];
        lane_change = path_backup[4][path_emerg-1+diff];
        for (auto j=0;j<5;j++){
          path_backup[j].erase(path_backup[j].begin()+path_emerg+diff,path_backup[j].end());
        }
      }
    }
    // consider making a lane change if we haven't already done so
    if(!lane_change){
      vector<int> next_lanes;
      if (current_lane == 0) next_lanes={1};
      else if (current_lane == 1) next_lanes={0,2};
      else if (current_lane == 2) next_lanes={1};

      for (int i = 0; i < next_lanes.size(); i++){
        auto cars_ahead_lane = CarsAhead(car_s, next_lanes[i], -10, 60);
        if (cars_ahead_lane.size() == 0) {
          lane_change = true;
          target_lane = next_lanes[i];
          d_shift = LANE_D * target_lane + LANE_D/2.;
          // make sure to stay in the lane
          if (d_shift == 10) d_shift = 9.8;

          vector<double> spline_t_points, spline_d_points;
          // interp times to smoothly change lanes over 6 seconds
          spline_t_points = {-1, 0, 0.5, 2.5, 4, 5, 6};
          spline_d_points = {d_next, d_next, d_next, d_shift, d_shift, d_shift, d_shift};

          d_t.set_points(spline_t_points, spline_d_points);
          break;
        }
      }
    }
  } else v_max = FIFTY_MPH;

  // begin trajectory planning
  // fill next with previous points
  if (prev_size > 0){
    for (auto i = 0; i < prev_size; i++){
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }
  }
  
  // now populate with new
  for (auto i = prev_size; i < path_length; i++) { // filling path with new points
    double vd = 0;//substitude of a_t and a_n to decrease a_s during lane change
    if (lane_change) { // shift in process
      d_time += TIMESTEP;
      d_next = d_t(d_time);
      vd = 1;
      if (d_time > 4.02){ // shift is finished
        lane_change = false;
        d_time = 0;
        vd = 0;
      }
    } else {
      d_next = (LANE_D * current_lane + LANE_D/2.);
      // stay on road
      if (current_lane == 2) d_next = 9.8;
    }
    a = AMAX;
    double a_vmax = a;
    double a_buf = a;
    // estimating next path planning using optimistic a and v
    double t2 = t*t;
    s1 = s0 + v0*t + 0.5*a*t2;
    v = v0 + a*t;

    // v_s correction considering curve
    // calculating v_x, v_y as derivatives dx/dt = dx/ds * ds/dt
    double x_dot_s, y_dot_s; // calculating v_s to adjust v on curves
    x_dot_s = follow.x_s.deriv(1,s1) + (d_next) * follow.dx_s.deriv(1,s1);
    y_dot_s = follow.y_s.deriv(1,s1) + (d_next) * follow.dy_s.deriv(1,s1);
    double v_coef = sqrt(x_dot_s*x_dot_s + y_dot_s*y_dot_s);

    // checking max speed considering v_s correction
    if (v * v_coef > sqrt(v_max*v_max-vd*vd)) {
      v = v_max / v_coef;
      a_vmax = (v - v0) / t;
      // enforce minimum condition
      if (a_vmax < -AMAX) {
        a_vmax = -AMAX;
      }
    }
    // calculating new path point considering collision safe buffer
    // if there is a vehicle ahead compute an acceleration to maintain buffer
    if (veh_id > -1){
      double veh_s = predictions.PropagateVehicle(veh_id, i * t);
      double distance = veh_s - s1;
      if (distance < 0) distance = TRACK_LENGTH + distance;
      if (distance < buffer){
        a_buf = (buffer - s0 - v0*t)*0.5 / (t * t);
        if (a_buf < -AMAX){
          a_buf = -AMAX;
        }
      }
    }
    // calculating next path point considering collision safe buffer ahead and max speed constraints
    a = min(a_vmax, a_buf);
    v = v0 + a*t;
    s1 = s0 + v0*t + 0.5*a*t2;
    a = AMAX;

    s0 = s1;
    v0 = v;

    // mod out track length
    s0 = fmod(s0, TRACK_LENGTH);
    // storing s,v,d,d_time,lane_change in case of emergency breaking
    path_backup[0].push_back(s0);
    path_backup[1].push_back(v0);
    path_backup[2].push_back(d_next);
    path_backup[3].push_back(d_time);
    path_backup[4].push_back((double)lane_change);
    if (path_backup[0].size()>path_length){
      for (auto j=0;j<5;j++) path_backup[j].erase(path_backup[j].begin());
    }

    // push new path points to the result
    vector<double> nextXY;
    nextXY = follow.fresnetToCartesian(s0, d_next);
    next_x_vals.push_back(nextXY[0]);
    next_y_vals.push_back(nextXY[1]);
  }
  return {next_x_vals, next_y_vals};
}

