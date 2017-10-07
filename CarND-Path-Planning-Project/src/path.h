#ifndef PATH_H
#define PATH_H

#include <vector>
#include <map>

#include "follow.h"
#include "predict.h"

using namespace std;

// following quiz exercise:
//    next states are as follows
//
//    next["KL"] = {"KL", "PLCL", "PLCR"};  lane keeping: you can continue lane keeping, or prepare to change lanes (if able)
//    next["PLCL"] = {"LCL", "PLCL", "KL"}; preparing to change to the left lane: continue, change lane left, or lane keep
//    next["PLCR"] = {"LCR", "PLCR", "KL"}; preparing to change to the right lane: continue, change lane right, or lane keep
//    next["LCL"] = {"LCL", "KL"};          lane change left: continue, or keep lane
//    next["LCR"] = {"LCR", "KL"};          lane change right: continue, or keep lane

class Plan{
public:
    // smoothed trajectory planned
    Follow follow;
    // key-value pair for allowable state transitions in state machine (see above)
    map<string,vector<string>> next_states;
    // predictions
    Predictions predictions;

    // constructor
    Plan() {}
    // destructor
    ~Plan() {}

    // return allowable states
    vector<string> GetNextStates(string cur_state);

    // get best trajectory wrt heuristics
    vector<vector<double>> GetBestTrajectory(double car_s,
                                             double car_d,
                                             vector<vector<double>> sensor_fusion,
                                             vector<double> previous_path_x,
                                             vector<double> previous_path_y);

    // initialize
    void Init(vector<double> wp_x,
              vector<double> wp_y,
              vector<double> wp_s,
              vector<double> wp_dx,
              vector<double> wp_dy);

    // distances to cars ahead
    vector<vector<double>> CarsAhead(double car_s, int lane, double behind=0, double length=100,bool collision=false);
};

class Path {
public:
    Plan plan;

    Path(vector<double> wp_x,
         vector<double> wp_y,
         vector<double> wp_s,
         vector<double> wp_dx,
         vector<double> wp_dy);

    ~Path() {}

    vector<vector<double>> Process(double car_s,
                                   double car_d,
                                   vector<vector<double>> sensor_fusion,
                                   vector<double, allocator<double>> previous_path_x,
                                   vector<double, allocator<double>> previous_path_y);
};

#endif //PATH_H
