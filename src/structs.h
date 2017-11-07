//
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

struct RoadMap{
  vector<double> waypoints_x;
  vector<double> waypoints_y;
  vector<double> waypoints_s;
  vector<double> waypoints_dx;
  vector<double> waypoints_dy;
  
  int num_lanes;
  double lane_width; //  meters
  double speed_limit; // mph
};

//struct Lane{
//  int id;
//  vector<OtherVehicle> all_vehicles;
//  OtherVehicle closest_vehicle;
//  bool empty = true;
//};

struct Trajectory{
  vector<double> x_vals;
  vector<double> y_vals;
};

enum States {
  CC, // cruise control with no car in front
  KL, // keep lane
  LCL, // lane change left
  LCR, // lane change right
};

#endif /* structs_h */
