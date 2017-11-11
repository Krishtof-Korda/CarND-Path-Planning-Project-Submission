//
//  tools.hpp
//  path_planning
//
//  Created by Krishtof Korda on 01/Nov/17.
//

#ifndef tools_h
#define tools_h


#include <stdio.h>
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline/src/spline.h"
#include "json.hpp"
#include <vector>

#define DBL_MAX          1.7976931348623158e+308 // max value


using namespace std;

namespace tools
{
  // For converting back and forth between radians and degrees.
  constexpr double pi();
  double deg2rad(double x);
  double rad2deg(double x);
  // For converting between mph and m/s back and forth.
  double mph2ms(double v);
  double ms2mph(double v);

  // Checks if the SocketIO event has JSON data.
  string hasData(string s);

  // Euclidian distance
  double distance(double x1, double y1, double x2, double y2);

  // Finds the closest waypoint to Ego
  int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

  // Finds next way point in front of Ego
  int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
  vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

  // Transform from Frenet s,d coordinates to Cartesian x,y
  vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
  
  //KK Helper to get the lane number (lane 0 is most far left
  int get_lane(double d, double lane_with, int num_lanes);

  
}


#endif /* tools_h */
