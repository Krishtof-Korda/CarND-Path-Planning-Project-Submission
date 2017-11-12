//  Self-driving Car Engineer Nanodegree - Udacity
//  OtherVehicle.hpp
//  path_planning
//
//  Created by Krishtof Korda on 04/Nov/17.
//

#ifndef OtherVehicle_h
#define OtherVehicle_h

#include <stdio.h>
#include <vector>
#include "tools.h"
#include "structs.h"
#include <utility>
//#include "cost_functions.h"

using namespace std;
using namespace tools;

//KK Houses the data of a vehicle other than Ego.
class OtherVehicle{
public:
  
  //Constructor
  OtherVehicle();
  
  //Destructor
  virtual ~OtherVehicle();
  
  double id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  float d;
  double yaw;
  double speed;
  int lane;
  double dist_from_ego = DBL_MAX;
  
  //KK Place holder to predict points where vehicle will be in the future
  Trajectory predicted_trajectory;

  //KK return -1 for empty vehicle
  bool isEmpty();
  
  //KK Generate a spline trajectory for given variables
  Trajectory generate_predicted_trajectory(RoadMap roadMap);
};
#endif /* OtherVehicle_h */
