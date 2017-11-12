//  Self-driving Car Engineer Nanodegree - Udacity
//  OtherVehicle.cpp
//  path_planning
//
//  Created by Krishtof Korda on 04/Nov/17.
//

#include "OtherVehicle.h"

OtherVehicle::OtherVehicle(){
  
  this->id = -1; //KK initialize vehicle as empty
}

OtherVehicle::~OtherVehicle(){}

bool OtherVehicle::isEmpty() // return a logical for whether vehicle is empty
{
  return (this->id == -1);
}

/*********************************************************************************************
KK generate predicted trajectory of the other vehicle for cost function comparison
 this was not used due to time constraints of the project.
*********************************************************************************************/
Trajectory OtherVehicle::generate_predicted_trajectory(RoadMap roadMap)
{
  
  //Aa Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  //Aa Later we will interpolate these waypoints with a spline and fill it in with more points that control speed
  vector<double> ptsx;
  vector<double> ptsy;
  
  //Aa reference x,y, yaw states
  //Aa either we will reference the starting point as where the car is or at the previous paths end point
  double ref_x = x;
  double ref_y = y;
  double ref_yaw = deg2rad(yaw);
  
  //Aa if previous size is almost empty, use the car as starting reference
  
  //Aa Use two point that make the path tagent to the car
  double prev_car_x = x - cos(yaw);
  double prev_car_y = y - sin(yaw);
  
  ptsx.push_back(prev_car_x);
  ptsx.push_back(x);
  
  ptsy.push_back(prev_car_y);
  ptsy.push_back(y);
  
  //Aa In Frenet add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(s+30,(2+4*lane),roadMap.waypoints_s,roadMap.waypoints_x,roadMap.waypoints_y);
  vector<double> next_wp1 = getXY(s+60,(2+4*lane),roadMap.waypoints_s,roadMap.waypoints_x,roadMap.waypoints_y);
  vector<double> next_wp2 = getXY(s+90,(2+4*lane),roadMap.waypoints_s,roadMap.waypoints_x,roadMap.waypoints_y);
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  
  for(int i = 0; i<ptsx.size(); i++)
  {
    //Aa shift car reference angle to 0 degrees
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;
    
    ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y *sin(0-ref_yaw));
    ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y *cos(0-ref_yaw));
  }
  
  //Aa create a spline
  tk::spline s;
  
  //Aa set (x,y) points to the spline
  s.set_points(ptsx, ptsy);
  
  //Aa Define the actual (x,y) points we will use for the planner
  vector<double> traj_x_vals;
  vector<double> traj_y_vals;
  
  
  //Aa Calculate how to break up spline points so that we travel at our desired ref velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));
  double x_add_on = 0;
  
  //Aa Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
  for(int i=1; i<=50; i++)
  {
    double N = target_dist/(.02*speed); //KK speed in m/s.
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);
    
    x_add_on = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    //Aa roatate back to normal after rotating it earlier
    x_point = x_ref *cos(ref_yaw) - y_ref *sin(ref_yaw);
    y_point = x_ref *sin(ref_yaw) + y_ref *cos(ref_yaw);
    
    x_point += ref_x;
    y_point += ref_y;
    
    traj_x_vals.push_back(x_point);
    traj_y_vals.push_back(y_point);
  }
  Trajectory trajectory{traj_x_vals, traj_y_vals};
  this->predicted_trajectory = trajectory;
  return trajectory;
}
/*********************************************************************************************
 *********************************************************************************************/
