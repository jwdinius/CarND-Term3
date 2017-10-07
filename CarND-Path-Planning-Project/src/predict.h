#ifndef PREDICT_H
#define PREDICT_H

#include "follow.h"

#include <vector>
#include <map>

using namespace std;

// class Vehicle handles state propagation
class Vehicle {
public:
    // state vector:
    // time, x pos, y pos, x vel, y vel, fresnet s, fresnet d
    vector <vector <double>> state;
    double as, ad; // fresnet second derivatives
    double vs, vd; // fresnet first derivatives

    // constructor
    Vehicle() : state(STATES,vector<double>(0.0)) {}
    // destructor
    ~Vehicle() {}
    // return state vector
    vector<vector<double>> getState();
};

// class Prediction handles state prediction
class Predictions {
public:
  // key-value pair {vehicle id, state}
  map <int, Vehicle> vehicles;
  // sensor fusion vector at current time
  vector<vector<double>> sf_vec;

  //constructor
  Predictions()  {}
  // destructor
  ~Predictions() {}

  // update states
  void update (Follow follow, vector<vector<double>>sensor_fusion);
  // get heading difference
  double GetEps (Follow follow, double vx, double vy, double s, double d);
  // propagate vehicle
  double PropagateVehicle(int vehID, double t);
  // identify vehicles in lane
  vector<int> CarsInLane(double car_s, double car_d);
};


#endif //PREDICT_H