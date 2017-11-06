//
//  EgoVehicle.cpp
//  path_planning
//
//  Created by Krishtof Korda on 01/Nov/17.
//

#include "EgoVehicle.h"
#include "OtherVehicle.h"



EgoVehicle::EgoVehicle(RoadMap roadMap){
  
  this->lane = 1;
  this->ref_vel = 49.5;
  this->vel = 0;
  this->roadMap = roadMap;
  this->max_accel = .4; //KK max accel or decel
  this->clear_left = true;
  this->clear_right = true;
  this->clear_forward = true;
  this->clear_rear = true;
  
}

EgoVehicle::~EgoVehicle(){
  
}
/********************************************************************************************/
//kk Update Ego given car data, previous path, the other vehicles on the road.
/********************************************************************************************/
void EgoVehicle::update(vector<double> car_data, vector< vector<double> > previous_path, vector<OtherVehicle> vehicles, vector<double> &next_x_vals, vector<double> &next_y_vals)
{
  //DEBUG
  cout << "Updating Ego...." << endl;
  
  this->x = car_data[0];
  this->y = car_data[1];
  this->s = car_data[2];
  this->d = car_data[3];
  this->yaw = car_data[4];
  this->speed = car_data[5];
  this->end_path_s = car_data[6];
  this->end_path_d = car_data[7];
  this->previous_path_x = previous_path[0];
  this->previous_path_y = previous_path[1];
  //Aa size of left over points from previous path of 50 points that was generated
  this->prev_size = previous_path_x.size();
  
  printf("PREV_SIZE: %d\n", prev_size); //DEBUG
  
  find_closest_vehicles(vehicles);
  
  //Aa
  if(prev_size > 0)
  {
    this->s = this->end_path_s;
  }
  
  bool too_close = false;
  
  //KK speed of car in front for use later
  double tandem_car_speed = 0;
  //KK distance of car in front for use later
  double tandem_car_distance = 100;
  
  //DEBUG
  printf("closest_vechicles.size: %lu\n", closest_vehicles.size());
  
  for(int i=0; i<closest_vehicles.size(); i++)
  {
    
    double vx = closest_vehicles[i].vx;
    double vy = closest_vehicles[i].vy;
    double check_speed = sqrt(vx*vx + vy*vy);
    double check_car_s = closest_vehicles[i].s;
    double d = closest_vehicles[i].d;
    
    //DEBUG
    printf("closest_vehicles[%d].d: %f\n", i, d);
    
    ///////////////////Check if we have car in our lane//////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////
    if(d<(2+4*this->lane+2) && d>(2+4*this->lane-2))
    {
      
      //DEBUG
      printf("CAR IN MY LANE!!!!........\n");
      
      //Aa if using previous point can project s value out
      check_car_s += (double)this->prev_size*.02*check_speed;
      
      //kk reaction gap between car in front based on current speed.
      double react_time = 0.1; // reaction time in seconds
      this->react_gap = react_time * this->vel;
      
      //DEBUG
      printf("CHECK_CAR_S - S, REACT_GAP: (%f, %f)\n", check_car_s-this->s, react_gap);
      printf("CHECK_CAR_S, S: (%f, %f)\n", check_car_s, this->s);
      
      //Aa check s value greater than mine and s gap
      if(check_car_s > this->s && check_car_s - this->s < react_gap)
      {
        
        //DEBUG
        printf("CAR TOO CLOSE!!!!........\n");
        
        //Aa do some logic here, lower ref vel so we don't crash into the car in front of us
        // could also flag to try to change lanes.
        too_close = true;
        tandem_car_speed = check_speed;
        tandem_car_distance = fabs(check_car_s - this->s);
      }
    }
    
    ///////////////////Check if we are clear in specific directions//////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////
    
    double lane_width = roadMap.lane_width;
    double half_lane = lane_width/2;
    double num_lanes = roadMap.num_lanes;
    double ego_lane_center = lane_width*lane + half_lane;
    double adjacent_lane = lane_width + half_lane;
    
    
    if(d<ego_lane_center && d>(ego_lane_center - adjacent_lane) && d<0 && d<num_lanes*lane_width) //KK car on my left
    {
      if(check_car_s > this->s && check_car_s - this->s < react_gap+car_length/2+car_buffer) //kk car in front on left
      {
        clear_left = false;
        clear_forward = false;
      }
      else if(check_car_s < this->s && this->s - check_car_s < react_gap+car_length/2+car_buffer) //kk car behind on left
      {
        clear_left = false;
        clear_rear = false;
      }
      else {clear_left = true; clear_forward = true; clear_rear = true;}
    }
    
    if(d>ego_lane_center && d<(ego_lane_center + adjacent_lane) && d<0 && d<num_lanes*lane_width) //KK car on my right
    {
      if(check_car_s > this->s && check_car_s - this->s < react_gap+car_length/2+car_buffer) //kk car in front on right
      {
        clear_right = false;
        clear_forward = false;
      }
      else if(check_car_s < this->s && this->s - check_car_s < react_gap+car_length/2+car_buffer) //kk car behind on right
      {
        clear_right = false;
        clear_rear = false;
      }
      else {clear_right = true; clear_forward = true; clear_rear = true;}
    
    }
    
    ///////////////////////////////////////////////////////////////
  }
  
  double k_decel = 2/tandem_car_distance; //KK slow down gain
  double k_accel = 100*tandem_car_distance; //KK speed up gain
  
  
  
  
  
  //kk if vehicle is too close in front do a lane change if possible
  if(too_close)
  {
    //DEBUG
    printf("Vel, tandem speed: (%f, %f)\n", vel, tandem_car_speed);
    
    /**************Adaptive Cruise Conntrol********************/
    if(this->vel > tandem_car_speed)
    {
      this->vel -= min(k_decel, max_accel);
    }
    else
    {
      this->vel += min(k_accel, max_accel);
    }
    /***********************************************************/
    
    
    
    /**************Case Clear Left and Forward********************/
    if(clear_left && clear_forward) Maneuver=LCL;
    /***********************************************************/
    
    
    /**************Case Clear Left and Rear********************/
    if(clear_left && clear_rear) Maneuver=LCL;
    /***********************************************************/
    
    
    /**************Case Clear Right and Forward********************/
    if(clear_right && clear_forward) Maneuver=LCR;
    /***********************************************************/
    
    
    /**************Case Clear Right and Rear********************/
    if(clear_right && clear_rear) Maneuver=LCR;
    /***********************************************************/
    
    
    /**************Case Clear Left and Right********************/
    if(clear_left && clear_right) Maneuver=LCL; //default lane change left
    /***********************************************************/
  
  }
  
  //kk else if we are going too slow speed up
  else if(this->vel < this->ref_vel)
  {
    this->vel += max_accel;
    
  }
  
  /**************Switch Case to Make Decision********************/
  switch (Maneuver) {
    case LCL:
      lane = max(lane-1, 0);
      break;
    
    case LCR:
      lane = max(lane+1, 2);
      break;
      
    case KL:
      lane = lane+0;
      break;
      
    default:
      break;
  }
  /***********************************************************/
  
  
  
  //kk Generate the trajectory based on the current target lane
  generate_trajectory();
  
  //DEBUG
  printf ("Assigning next x, y vals...\n");
  
  //kk assign generate trajectory to next_x_vals, next_y_vals
  next_x_vals = calculated_trajectory.x_vals;
  next_y_vals = calculated_trajectory.y_vals;
  
  //DEBUG
  printf ("calctrajx size: %lu\n",calculated_trajectory.x_vals.size());
//  printf ("nextx size: %lu\n",next_x_vals.size());
  
//  //DEBUG
//  for(int i=0; i<next_x_vals.size(); i++)
//    printf ("Next values x,y point: (%f, %f)\n\n",next_x_vals[i],next_y_vals[i]);
  
}
/********************************************************************************************/


/********************************************************************************************/
//KK find the closest vehicles
/********************************************************************************************/
void EgoVehicle::find_closest_vehicles(vector<OtherVehicle> vehicles)
{
  
  //DEBUG
  cout<<"Finding closest cars...."<<endl;
  
  const int search_range = 22*2; //KK search with 22m/sec*2sec meters
  vector<OtherVehicle> closest_vehicles;
  for(auto vehicle : vehicles){
    
    double dist_s = fabs(vehicle.s-this->s); //KK distance in s from ego of other car
    double d = vehicle.d;
    
    //KK find cars within search_range
    int num_lanes = roadMap.num_lanes;
    double lane_width = roadMap.lane_width;
    if(dist_s < search_range && d > 0 && d <= num_lanes*lane_width){
      closest_vehicles.push_back(vehicle);
      
      //DEBUG
      printf ("closest vehicle (id, s, d): (%f, %f, %f) \n\n", vehicle.id, vehicle.s, vehicle.d);
    }
  }
  this->closest_vehicles = closest_vehicles;
}
/********************************************************************************************/



/********************************************************************************************/
//KK Trajectory Generator Function ported from Aaron's walkthrough///////////////
// Generate a spline trajectory for given variables
/********************************************************************************************/
Trajectory EgoVehicle::generate_trajectory()
{
  //DEBUG
  printf ("Generating trajectory for Ego....\n");
  
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
  double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));
  double x_add_on = 0;
  
  //Aa Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
  for(int i=1; i<=50-prev_size; i++)
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
    
    //DEBUG
    //printf ("Trajectory x, y point: (%f, %f) \n\n", x_point, y_point);
    
  }
  Trajectory trajectory{x_vals, y_vals};
  this->calculated_trajectory = trajectory;
  return trajectory;
}
/********************************************************************************************/




/********************************************************************************************/
//TODO: check for a collision
/********************************************************************************************/
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
    
  }
}
/********************************************************************************************/















