//  Self-driving Car Engineer Nanodegree - Udacity
//  structs.h
//  path_planning
//
//  Created by Krishtof Korda on 04/Nov/17.
//

#ifndef structs_h
#define structs_h

#include <vector>
//#include "OtherVehicle.h"

using namespace std;

//KK A structure for holding all of the road map data
struct RoadMap{
  vector<double> waypoints_x;
  vector<double> waypoints_y;
  vector<double> waypoints_s;
  vector<double> waypoints_dx;
  vector<double> waypoints_dy;
  
  int num_lanes;
  double lane_width; //KK  meters
  double speed_limit; //KK mph
};

//KK A structure holding the x, y values of a trajectory
struct Trajectory{
  vector<double> x_vals;
  vector<double> y_vals;
};

//KK Enumeration for the switch the state of Ego
enum States {
  CC, //KK cruise control with no car in front
  ACC, //KK keep lane
  LCL, //KK lane change left
  LCR, //KK lane change right
};

#endif /* structs_h */
