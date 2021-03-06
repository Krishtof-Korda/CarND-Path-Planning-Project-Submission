#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "tools.h"
#include "EgoVehicle.h"
#include "structs.h"
#include "OtherVehicle.h"

using namespace std;
using namespace tools;


// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  //KK Road parameters
  int num_lanes = 3;
  double lane_width = 4; //  meters
  double speed_limit = 50; // mph
  
  //KK Road map of waypoints
  RoadMap roadMap{map_waypoints_x, map_waypoints_y, map_waypoints_s,
    map_waypoints_dx, map_waypoints_dy, num_lanes, lane_width, speed_limit};

  
  //KK Declare Ego vehicle
  EgoVehicle Ego(roadMap);
  
  h.onMessage([&Ego](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          int prev_size = previous_path_x.size();
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];
          
          
          //KK package car data from sim
          vector<double> car_data = {car_x, car_y, car_s, car_d, car_yaw, car_speed,
            end_path_s, end_path_d};
          
          //KK package previous path data from sim
          vector< vector<double> > previous_path = {previous_path_x, previous_path_y};
          
          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          /*******************************************************************************/
          //KK populate cars on the road from sensor_fusion
          /*******************************************************************************/
          vector<OtherVehicle> vehicles_on_road;
          for(int i=0; i<sensor_fusion.size(); i++){
            OtherVehicle car;
            
            car.id = sensor_fusion[i][0];
            car.x = sensor_fusion[i][1];
            car.y = sensor_fusion[i][2];
            car.vx = sensor_fusion[i][3];
            car.vy = sensor_fusion[i][4];
            //car.s = double(sensor_fusion[i][5]);
            car.d = sensor_fusion[i][6];
            car.speed = sqrt(car.vx*car.vx + car.vy*car.vy);
            car.s = double(sensor_fusion[i][5]) + double(prev_size * 0.02 * car.speed); // predicted at end of path
            car.lane = get_lane(car.d, Ego.roadMap.lane_width, Ego.roadMap.num_lanes);
            
            vehicles_on_road.push_back(car);
          }
          /*******************************************************************************/
          /*******************************************************************************/
          
        
          //kk declare next x, y values to pass to sim
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          /*******************************************************************************/
          //kk jump in to Ego and start the process of making a decision
          /*******************************************************************************/
          Ego.update(car_data, previous_path, vehicles_on_road, next_x_vals, next_y_vals);
          /*******************************************************************************/
          /*******************************************************************************/
          
         
          json msgJson;
        
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
