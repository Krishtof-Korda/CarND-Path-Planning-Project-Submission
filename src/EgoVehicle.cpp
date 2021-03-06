//  Self-driving Car Engineer Nanodegree - Udacity
//  EgoVehicle.cpp
//  path_planning
//
//  Created by Krishtof Korda on 01/Nov/17.
//

#include "EgoVehicle.h"
#include "OtherVehicle.h"


EgoVehicle::EgoVehicle(RoadMap roadMap){
  
  this->ref_vel = roadMap.speed_limit * .99; //KK reference velocity set to just below speed limit
  this->vel = 0; //KK current commanded velocity of Ego
  this->roadMap = roadMap; //KK map of waypoints given by "../data/highway_map.csv"
  this->max_accel = .4; //KK max accel or decel to prevent over acceleration
  this->num_points = 20; //KK number of point to plan ahead used in generate_trajectory()
}

EgoVehicle::~EgoVehicle(){
  
}

/********************************************************************************************/
//KK Update trajectory of Ego given car data, previous path, the other vehicles on the road.
/********************************************************************************************/
void EgoVehicle::update(vector<double> car_data, vector< vector<double> > previous_path, vector<OtherVehicle> vehicles, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
//  //DEBUG
//  cout << "Updating Ego...." << endl;
  
  this->x = car_data[0];
  this->y = car_data[1];
  this->s = car_data[2];
  this->d = car_data[3];
  this->lane = get_lane(this->d, roadMap.lane_width, roadMap.num_lanes);
  this->yaw = car_data[4];
  this->speed = car_data[5];
  this->end_path_s = car_data[6];
  this->end_path_d = car_data[7];
  this->previous_path_x = previous_path[0];
  this->previous_path_y = previous_path[1];
  //Aa size of left over points from previous path of 50 points that was generated
  this->prev_size = previous_path_x.size();
  
//  printf("PREV_SIZE: %d\n", prev_size); //DEBUG
  
  //kk if we have left overs put Ego at the end of the left overs
  if(prev_size > 0)
  {
    this->s = this->end_path_s;
    this->d = this->end_path_d;
    //this->lane = get_lane(this->d, roadMap.lane_width, roadMap.num_lanes);
  }
  
  /******************************************************************************************/
  //KK Find the vehicles in front, left, right
  /******************************************************************************************/
  bool only_in_front = true;
  // Find the closest vehicle in FRONT and predict its location at Ego's end_path
  OtherVehicle Vehicle_Front = find_closest_vehicle(this->lane, vehicles, only_in_front);
  
  // Find the closest vehicle in LEFT and predict its location at Ego's end_path
  OtherVehicle Vehicle_Left  = find_closest_vehicle(this->lane-1, vehicles, not only_in_front);

  // Find the closest vehicle in RIGHT and predict its location at Ego's end_path
  OtherVehicle Vehicle_Right = find_closest_vehicle(this->lane+1, vehicles, not only_in_front);

  //Print distance from Ego for closest vehicles in each lane
  printf("Distance to closest vechicles (left, front, right): (%g, %g, %g)\n", Vehicle_Left.dist_from_ego,
         Vehicle_Front.dist_from_ego, Vehicle_Right.dist_from_ego);
  /******************************************************************************************/
  /******************************************************************************************/
  
  
  //kk safety gap between car in front based on current speed.
  double react_time = 1; // reaction time in seconds
  this->react_gap = react_time * mph2ms(this->vel);
  
//  // DEBUG
//  printf("Ego vel: %f\n\n", this->vel);
  
  //kk Adaptive cruise control proportional acceleration/deceleration gains
  double k_decel = 2 / fabs(Vehicle_Front.dist_from_ego); //KK slow down gain
  double k_accel = 100 * fabs(Vehicle_Front.dist_from_ego); //KK speed up gain
 
  
  /*****************************************************************************************/
  //KK Decide which maneuver to choose
  /*****************************************************************************************/
  
  //KK Check if lane is open in front or car is far enough away, Choose cruise control at max reference velocity
  if ( Vehicle_Front.isEmpty() || (Vehicle_Front.dist_from_ego > react_gap) )
    Maneuver = CC; //KK set the non-adaptive cruise control

  else //KK We have a car in front. We need to decide whether to stay and follow, change left or change right.
  {
//    //DEBUG
//    printf("CAR IN MY LANE TOO CLOSE!!!!........\n");
  
    Maneuver = ACC; //KK Default to ACC unless overwritten later
    
    //KK Determine the buffers for an allowable lane change
    //KK adjust buffer based on the closing speed of the vehicle relative to Ego
    double left_closing_speed = ( Vehicle_Left.speed - mph2ms(this->vel) );
    double left_speed_buffer= max(left_closing_speed, 0.0) * react_time;
    
    double right_closing_speed = ( Vehicle_Right.speed - mph2ms(this->vel) );
    double right_speed_buffer= max(right_closing_speed, 0.0) * react_time;
    
    //KK Add buffer for the length of the Ego and general car buffer
    double left_rear_buffer = left_speed_buffer + car_length/2 + car_buffer;
    double right_rear_buffer = right_speed_buffer + car_length/2 + car_buffer;
    
    bool left_clear = ( Vehicle_Left.s < (this->s - left_rear_buffer) ) ||
                      ( Vehicle_Left.s > (this->s + react_gap) );
    
    bool right_clear = ( Vehicle_Right.s < (this->s - right_rear_buffer) ) ||
                        ( Vehicle_Right.s > (this->s + react_gap) );
    
    
    
    //KK if both left and right are open add a boolean to help decide which lane to change to.
    bool left_car_further = true;
    if ( left_clear && right_clear ) //KK if left and right clear for lane change
      //KK Check which car in front is closer left or right
      if ( (Vehicle_Left.dist_from_ego > 0) && (Vehicle_Right.dist_from_ego > 0) )
        left_car_further = (Vehicle_Left.dist_from_ego) > (Vehicle_Right.dist_from_ego);
    
    //KK check if there is a left lane and if it is clear and if left car is further away than right
    if ( (this->lane > 0)  &&  (Vehicle_Left.isEmpty() || (left_clear && left_car_further) ) )
      Maneuver = LCL; //KK We don't have a car on the left TOO close and we can change lane LEFT
 
    //KK check if there is a right lane and if it is clear
    else if ( (this->lane < this->roadMap.num_lanes-1) && (Vehicle_Right.isEmpty() || right_clear) )
      Maneuver = LCR;
  }
  /*****************************************************************************************/
  /*****************************************************************************************/
  
  
  
  /**************Switch Case to Execute Maneuver********************/
  switch (Maneuver) {
    
    case CC: // Cruise control with no car in front
      printf("MANEUVER EXECUTED CC.........\n\n");
      /******************************** Cruise Control***********/
      if(this->vel < this->ref_vel) this->vel += max_accel;
      if(this->vel > this->ref_vel) this->vel -= max_accel;
      /********************************************************/
      break;
    
    case LCL:
      printf("MANEUVER EXECUTED LCL.........\n\n");
      this->lane = max(this->lane-1, 0); //KK subract 1 from lane unless we are already in far left lane
      break;
      
    case LCR:
      printf("MANEUVER EXECUTED LCR.........\n\n");
      this->lane = min(this->lane+1, roadMap.num_lanes-1); //KK add 1 to lane unless we are already in far right lane
      break;
      
    case ACC: //Keep Lane and engage adaptive cruise control
      printf("MANEUVER EXECUTED ACC.........\n\n");
      /**************Adaptive Cruise Conntrol********************/
      //KK uses the proportional gains defined earlier k_decel, k_accel
      if(this->vel > Vehicle_Front.speed) this->vel -= min(k_decel, max_accel);
      else this->vel += min(k_accel, max_accel);
      /***********************************************************/
      break;
      
    default:
      break;
  }
      /***********************************************************/
  
  
  
  /***********************************************************/
  //kk Generate the trajectory based on the current target lane determined by switch case
  /***********************************************************/
  generate_trajectory();
  /***********************************************************/
  
  //DEBUG
  //printf ("Assigning next x, y vals...\n");
  
  /***********************************************************/
  //kk assign generated trajectory to next_x_vals, next_y_vals
  /***********************************************************/
  next_x_vals = calculated_trajectory.x_vals;
  next_y_vals = calculated_trajectory.y_vals;
  /***********************************************************/
  
  //DEBUG
  //printf ("calctrajx size: %lu\n",calculated_trajectory.x_vals.size());
  
}
/********************************************************************************************/


/********************************************************************************************/
//KK find the closest vehicles in a given lane
/********************************************************************************************/
OtherVehicle EgoVehicle::find_closest_vehicle(int lane, vector<OtherVehicle> vehicles, bool only_in_front)
{
  
  OtherVehicle closest_vehicle;
  
//  //DEBUG
//  cout<<"Finding closest vehicle to Ego...."<<endl;
  
  //KK First Check if legitimate lane
  if (lane < 0 || lane > this->roadMap.num_lanes - 1)
    return closest_vehicle;
  
  double min_distance = DBL_MAX;
  bool consider_this_vehicle;
  for (auto vehicle : vehicles)
  {
    if (lane == vehicle.lane) //KK check if vehicle is in the lane of interest
    { //KK distance (IN FRONT OR BEHIND) in s from ego of other car
      double dist_s = vehicle.s - this->s;
      vehicle.dist_from_ego = dist_s;
      
      if (only_in_front && dist_s < 0)
      { //KK Don't consider the vehicle if only interested in front
        consider_this_vehicle = false;
      }
      else
      { //KK Mark the vehicle for consideration
        dist_s = fabs(dist_s);
        consider_this_vehicle = true;
      }
      // This will find the "closest" regardless if it is behind Ego or in front of Ego.
      if (consider_this_vehicle && dist_s < min_distance)
      { //KK overwrite the previous min distance if the new one is smaller
        min_distance = dist_s;
        closest_vehicle = vehicle;
      }
    }
  }
  return closest_vehicle;
}
/********************************************************************************************/



/********************************************************************************************
KK Trajectory Generator Function ported from Aaron's (Aa) walkthrough
Generate a spline trajectory for given variables
This was used instead of jerk minimizing trajectory due to time constraints of the project.
 It uses the last know position in the future trajectory to add points to the end of the
 trajectory up to the number of points to be in the trajectory.
 
 Three waypoints each at 30, 60, 90 meters away and the current reference position of the car
 are used to build a spline of N points evenly spaced.
********************************************************************************************/
Trajectory EgoVehicle::generate_trajectory()
{
//  //DEBUG
//  printf ("Generating trajectory for Ego....\n");
  
  this->prev_size = previous_path_x.size();
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
  if(prev_size < 2 )
  {
    //Aa Use two point that make the path tagent to the car
    double prev_car_x = x - cos(yaw);
    double prev_car_y = y - sin(yaw);
    
    ptsx.push_back(prev_car_x);
    ptsx.push_back(x);
    
    ptsy.push_back(prev_car_y);
    ptsy.push_back(y);
  }
  
  //Aa else use the previous path's end point as starting reference
  else
  {
    //Aa Redefine reference state as previous path end point
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];
    
    
    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
    
    //Aa Use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
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
  tk::spline spl;
  
  //Aa set (x,y) points to the spline
  spl.set_points(ptsx, ptsy);
  
  //Aa Define the actual (x,y) points we will use for the planner
  vector<double> x_vals;
  vector<double> y_vals;
  
  //Aa Start with all of the previous path points from last time
  for(int i=0; i<previous_path_x.size(); i++)
  {
    x_vals.push_back(previous_path_x[i]);
    y_vals.push_back(previous_path_y[i]);
  }
  
  //Aa Calculate how to break up spline points so that we travel at our desired ref velocity
  double target_x = 30.0; //KK 30 meters ahead of Ego's current reference position
  double target_y = spl(target_x);
  double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));
  double x_add_on = 0;
  
  //Aa Fill up the rest of our path planner after filling it with previous points, here we
  // will always output a number of points = num_points set in the class.
  for(int i=1; i<=num_points-prev_size; i++)
  {
    double N = target_dist/(.02*mph2ms(vel)); //KK converted ref_vel to m/s.
    double x_point = x_add_on + target_x/N;
    double y_point = spl(x_point);
    
    x_add_on = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    //Aa roatate back to normal after rotating it earlier
    x_point = x_ref *cos(ref_yaw) - y_ref *sin(ref_yaw);
    y_point = x_ref *sin(ref_yaw) + y_ref *cos(ref_yaw);
    
    x_point += ref_x;
    y_point += ref_y;
    
    x_vals.push_back(x_point);
    y_vals.push_back(y_point);
    
//    DEBUG
//    printf ("Trajectory x, y point: (%f, %f) \n\n", x_point, y_point);
    
  }
  Trajectory trajectory{x_vals, y_vals};
  this->calculated_trajectory = trajectory;
  return trajectory;
}
/********************************************************************************************/




/********************************************************************************************
KK TODO: attempted function to check for a collision. Not implemented due to time constraints
 of the project. Place for future improvement
********************************************************************************************/
void EgoVehicle::check_for_collision(){
  
  //unpack calculated trajectory of Ego in s and d
  vector<double> Ego_x_vals = calculated_trajectory.x_vals;
  vector<double> Ego_y_vals = calculated_trajectory.y_vals;
  int size = Ego_x_vals.size();
  
  for(auto vehicle : closest_vehicles){
    
    //kk generate the vehicles predicted trajectory
    vehicle.generate_predicted_trajectory(roadMap);
    
    //unpack predicted trajectory of other vehicle
    vector<double> car_x_vals = vehicle.predicted_trajectory.x_vals;
    vector<double> car_y_vals = vehicle.predicted_trajectory.y_vals;
    
    //KK TODO: loop through points and check whether matching points between car
    //KK and Ego have collided.
    
  }
}
/********************************************************************************************/















