#include "follow.h"

// constructor
Follow::Follow(vector<double> wp_x,
               vector<double> wp_y,
               vector<double> wp_s,
               vector<double> wp_dx,
               vector<double> wp_dy) {
  // setup interpolation routines
  x_s.set_points(wp_s, wp_x);
  y_s.set_points(wp_s, wp_y);
  dx_s.set_points(wp_s, wp_dx);
  dy_s.set_points(wp_s, wp_dy);

}

// fresnet to cartesian coordinate transform
vector<double> Follow::fresnetToCartesian(double s,double d) {
  // interpolate in Fresnet coordinates and convert to Cartesian
  double x = x_s(s) + dx_s(s) * d;
  double y = y_s(s) + dy_s(s) * d;
  return {x,y};
}

// reinitializer for trajectory generator
void Follow::Init(vector<double> wp_x,
                  vector<double> wp_y,
                  vector<double> wp_s,
                  vector<double> wp_dx,
                  vector<double> wp_dy) {
  map_x = wp_x;
  map_y = wp_y;

  x_s.set_points(wp_s, wp_x);
  y_s.set_points(wp_s, wp_y);
  dx_s.set_points(wp_s, wp_dx);
  dy_s.set_points(wp_s, wp_dy);
}
