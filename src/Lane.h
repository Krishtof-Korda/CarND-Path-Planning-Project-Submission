//
//  Lane.h
//  path_planning
//
//  Created by Krishtof Korda on 06/Nov/17.
//

#ifndef Lane_h
#define Lane_h

#include "OtherVehicle.h"

struct Lane{

  int id;
  vector<OtherVehicle> all_vehicles;
  OtherVehicle closest_vehicle;
  bool empty = true;
  
};
#endif /* Lane_h */
