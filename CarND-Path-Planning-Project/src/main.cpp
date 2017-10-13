#include <fstream>
#include <math.h>
#include <iostream>
#include <thread>
#include <vector>

#include <uWS/uWS.h>
#include "json.hpp"

#include "path.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-2;
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

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = TRACK_LENGTH;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  map_waypoints_x.push_back(map_waypoints_x[0]);
  map_waypoints_y.push_back(map_waypoints_y[0]);
  map_waypoints_s.push_back(TRACK_LENGTH);
  map_waypoints_dx.push_back(map_waypoints_dx[0]);
  map_waypoints_dy.push_back(map_waypoints_dy[0]);

  int   lane                = 1;  // Start in CENTER lane
  int   lane_change_wp      = 0;

  h.onMessage([&max_s, &lane, &lane_change_wp, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                       uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      //auto sdata = string(data).substr(0, length);
      //cout << sdata << endl;
      static const vector<string> lanes = {"left", "center", "right"};
      if (length && length > 2 && data[0] == '4' && data[1] == '2') {

        auto s = hasData(data);

        if (s != "") {
          auto j = json::parse(s);

          string event = j[0].get<string>();

          if (event == "telemetry") {
            // j[1] is the data JSON object

            // Main car's localization Data
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            double car_d = j[1]["d"];
            double car_yaw = deg2rad(j[1]["yaw"]);
            double car_speed = j[1]["speed"];

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            json msgJson;

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            // Step 1: setup current state info
            // output: own car states for future processing
            int p_size = previous_path_x.size();
            int next_wp;
            double x, y, xp, yp;
            double delta_x, delta_y;
            double dist;
            double delta_yaw = deg2rad(car_yaw);

            // if the previous path wasn't big enough, use the current pose to find the next waypoint
            if (p_size < SMALL_PATH) {
              next_wp = NextWaypoint(car_x, car_y, delta_yaw, map_waypoints_x, map_waypoints_y);
              end_path_s = car_s;
            }
            else {
              x         = previous_path_x[p_size-1];
              y         = previous_path_y[p_size-1];
              xp        = previous_path_x[p_size-2];
              yp        = previous_path_y[p_size-2];
              delta_x   = x - xp;
              delta_y   = y - yp;
              delta_yaw = atan2(delta_y, delta_x);

              next_wp   = NextWaypoint(x, y, delta_yaw, map_waypoints_x, map_waypoints_y);

              car_x     = previous_path_x[p_size-1];
              car_y     = previous_path_y[p_size-1];

              dist      = distance(x, y, xp, yp);

              car_speed = MPS_TO_MPH*(dist / DT);
            }

            // Step 2: process sensor fusion
            // output: ref_v based upon nearby vehicles
            double closest_s    = max_s;
            bool lane_change    = false;
            
            // loop over other vehicles (o_* represents other vehicle info)
            double o_d, o_s, o_vx, o_vy, o_speed;
            int o_lane;
            double front;
            double ref_v = MAX_SPEED; // if nothing is too close in front of us, go all out
            //cout << " " << lane << endl;
            for (auto i = 0; i < sensor_fusion.size(); i++) {
              o_vx   = sensor_fusion[i][3];
              o_vy   = sensor_fusion[i][4];
              o_s    = sensor_fusion[i][5];
              o_d    = sensor_fusion[i][6];
              o_lane = getLaneID(o_d);
              //cout << o_d << ", " << o_lane << endl;
              if (o_lane == lane) {
                // compute speed
                o_speed = distance(o_vx,o_vy,0.,0.);
                // use speed to propagate s
                o_s    += ((double)p_size) * DT * o_speed * MPS_TO_MPH;
                front   = o_s - end_path_s;
                //cout << front << endl;
                // check if other car is out front
                if ( (front > 0)  && (front < FRONT_BUFFER) && (front < closest_s) ) {
                  closest_s = front;

                  if (front > FRONT_TOO_CLOSE) {
                    // follow the front car if we aren't too close
                    ref_v = o_speed * MPS_TO_MPH;
                    lane_change = true;
                  } 
                  else {  // FRONT TOO CLOSE!
                    // go slower than front car
                    ref_v *= (front / FRONT_TOO_CLOSE) * o_speed * MPS_TO_MPH - TRAILING_VEL; 
                    lane_change = true;
                  }
                }
              }
            }

            ref_v = (ref_v > MAX_SPEED) ? MAX_SPEED : ref_v;

            // Step 3: lane change logic
            // output: desired lane
            int delta_wp     = next_wp - lane_change_wp;
            int remaining_wp = delta_wp % map_waypoints_x.size();
            
            if ( lane_change && (remaining_wp > SMALL_PATH) ) {
              /*cout << "..Checking Lane Change from: "
                   << getLaneInfo(LANE)
                   << ", at s: " << car_s << endl;*/
              bool lane_change_done = false;
              // First - check LEFT lane
              bool lane_safe;
              if (lane > 0) {
                // DEFAULT: try going to the left lane first (protocol)
                // Check if lane change left is safe
                lane_safe = isLaneChangeSafe(p_size, 
                                             end_path_s, 
                                             ref_v,
                                             lane - 1,
                                             sensor_fusion );
                
                if (lane_safe) { // OK to go to left
                  lane_change_done = true;
                  lane -= 1;  // go Left by one lane
                  lane_change_wp = next_wp;
                  //cout << "going left" << endl;
                }
              }
              // if we haven't already changed lane, check to see if the right lane is safe
              if (lane < 2 && !lane_change_done) {
                bool lane_safe = true;

                // Check if OK to go RIGHT
                lane_safe = isLaneChangeSafe(p_size, 
                                             end_path_s,
                                             ref_v, 
                                             lane + 1,
                                             sensor_fusion);
                
                if (lane_safe) {
                  lane_change_done = true;
                  lane += 1;  // go Right by one
                  lane_change_wp = next_wp;
                  //cout << "going right" << endl;
                }
              }
            }

            // step 4: create a smooth path to achieve desired trajectory
            // Create list of widely spaced XY Anchor Points, evenly spaced at 30meters (SPACING)
            // output: spline interpolator for planning future path
            vector<double> knots_x;
            vector<double> knots_y;

            // if previous path size is too small, use the previous state and construct a tangent point
            if (p_size < SMALL_PATH) {

              // find point tangent to current path
              double tan_car_x = car_x - cos(delta_yaw);
              double tan_car_y = car_y - sin(delta_yaw);

              // construct knot points
              knots_x.push_back(tan_car_x);
              knots_x.push_back(car_x);

              knots_y.push_back(tan_car_y);
              knots_y.push_back(car_y);
              //cout << "here" << endl;
            } 
            // use a couple points from the previous trajectory
            else {
              knots_x.push_back(previous_path_x[p_size-2]);
              knots_x.push_back(previous_path_x[p_size-1]);

              knots_y.push_back(previous_path_y[p_size-2]);
              knots_y.push_back(previous_path_y[p_size-1]);
            }

            // In Frenet coordinates, add spaced out waypoints to the starting points
            double d_target = LANE_WIDTH / 2. + double(lane) * LANE_WIDTH;  // d coord for target lane

            vector<double> next_wp0 = getXY((end_path_s + SPACING),    d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY((end_path_s + SPACING*2.), d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY((end_path_s + SPACING*3.), d_target, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            // add to the knot points
            knots_x.push_back(next_wp0[0]);
            knots_x.push_back(next_wp1[0]);
            knots_x.push_back(next_wp2[0]);

            knots_y.push_back(next_wp0[1]);
            knots_y.push_back(next_wp1[1]);
            knots_y.push_back(next_wp2[1]);

            // transform (translate and rotate) to local coordinates
            for (auto i = 0; i < knots_x.size(); i++)
            {
              double d_x = knots_x[i] - car_x;
              double d_y = knots_y[i] - car_y;

              knots_x[i] =  d_x * cos(delta_yaw) + d_y * sin(delta_yaw);
              knots_y[i] = -d_x * sin(delta_yaw) + d_y * cos(delta_yaw);
              //cout << knots_x[i] << endl;
            }

            // create a desired trajectory object
            tk::spline des_traj;
            //cout << knots_x.size() << ", " << knots_y.size() << endl;
            
            // setup spline interpolator
            des_traj.set_points(knots_x, knots_y);

            // Step 5. Fill up the rest of the path 
            // output: new path to send to simulator!!

            // add points from previous path
            for (auto i = 0; i < p_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // break up spline points
            double   tgt_x        = SPACING;
            double   tgt_y        = des_traj(tgt_x);
            double   tgt_distance = distance(tgt_x, tgt_y, 0., 0.);

            // increment holder for trajectory points
            double x_base = 0.;

            for (auto i = 1; i < FULL_PATH-p_size; i++) {
                // if too slow, speed up by a small amount
                if (car_speed < ref_v) {
                  car_speed += THROTTLE; 
                } // else if too fast, slow down by a small amount
                else if (car_speed > ref_v) {
                  car_speed -= THROTTLE;
                }
                // Calculate spacing between points based upon speed 
                int N            = (int)(tgt_distance / (DT * car_speed/MPS_TO_MPH)); // num of points
                double x_point   = x_base + tgt_x / (double)(N);
                double y_point   = des_traj(x_point);

                x_base       = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                // rotate back to global alignment
                x_point = (x_ref * cos(delta_yaw) - y_ref * sin(delta_yaw));
                y_point = (x_ref * sin(delta_yaw) + y_ref * cos(delta_yaw));
                // translate by global origin
                x_point += car_x;
                y_point += car_y;

                // add points to send to the simulator
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
          
            }

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            //flag = false;

            auto msg = "42[\"control\","+ msgJson.dump()+"]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          }
        } else {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1) {
        res->end(s.data(), s.length());
      } else {
        // i guess this should be done more gracefully?
        res->end(nullptr, 0);
      }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
      ws.close();
      std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}























