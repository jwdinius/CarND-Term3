#ifndef FOLLOW_H
#define FOLLOW_H

#include <vector>

// use a third-party package to do polynomial trajectory generation
#include "spline.h"

// defines
#define STATES   6
#define IDX_X    0
#define IDX_Y    1
#define IDX_VX   2
#define IDX_VY   3
#define IDX_S    4
#define IDX_D    5

#define TRACK_LENGTH 6946.

using namespace std;

class Follow {
public:
    // map
    vector<double> map_x;
    vector<double> map_y;
    // smoothed trajectory
    tk::spline x_s;
    tk::spline y_s;
    tk::spline dx_s;
    tk::spline dy_s;
    // constructors
    Follow(vector<double> wp_x,
           vector<double> wp_y,
           vector<double> wp_s,
           vector<double> wp_dx,
           vector<double> wp_dy);
    Follow() {}
    // destructor
    ~Follow() {}
    // convert to Cartesian
    vector<double> fresnetToCartesian(double s,double d);
    // initialize
    void Init(vector<double> wp_x,
              vector<double> wp_y,
              vector<double> wp_s,
              vector<double> wp_dx,
              vector<double> wp_dy);
};


#endif //FOLLOW_H
