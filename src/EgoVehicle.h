//
//  EgoVehicle.hpp
//  path_planning
//
//  Created by Krishtof Korda on 01/Nov/17.
//

#ifndef EgoVehicle_h
#define EgoVehicle_h

#include <stdio.h>
#include <vector>
#include "tools.h"
#include "structs.h"
#include "OtherVehicle.h"
//#include "cost_functions.h"

using namespace std;
using namespace tools;


//kk house all the Ego vehicle data
class EgoVehicle{
public:
  //Constructor
  EgoVehicle(RoadMap roadMap);
  //Destructor
  virtual ~EgoVehicle();
  
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;
  int lane;
  double ref_vel;
  double vel;
  double max_accel; //KK max accel or decel
  double car_width = 1.5; //kk car width in meters
  double car_length = 2.5; //kk car length in meters
  double car_buffer = 0.1; //kk buffer from other cars
  double end_path_s;
  double end_path_d;
  double react_gap;
  bool clear_left;
  bool clear_right;
  bool too_close;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  int prev_size;
  vector< OtherVehicle > closest_vehicles;
  Trajectory calculated_trajectory;
  RoadMap roadMap;
  States Maneuver = CC;

  //kk Update the state of Ego given the other vehicles on the road
  void update(vector<double> car_data, vector< vector<double> > previous_path, vector<OtherVehicle> vehicles, vector<double> &next_x_vals, vector<double> &next_y_vals);
  
  //KK find the closest vehicles
  void find_closest_vehicles(vector<OtherVehicle> vehicles);
  
  //KK Generate a spline trajectory for given variables
  Trajectory generate_trajectory();
  
  //KK Generate predictions for each of the closest cars
  void check_for_collision();
  
  
};




#endif /* EgoVehicle_h */
