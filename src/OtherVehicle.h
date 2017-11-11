//
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



//kk Houses the data of a vehicle other than Ego.
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
  
  Trajectory predicted_trajectory;

  //kk return -1 for empty vehicle
  bool isEmpty();
  
  //KK Generate a spline trajectory for given variables
  Trajectory generate_predicted_trajectory(RoadMap roadMap);

};
#endif /* OtherVehicle_h */
