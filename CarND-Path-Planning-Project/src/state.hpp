//
//  state.hpp
//  Path_Planning
//
//  Created by Тарасов Павел on 16/11/2017.
//
//

#ifndef state_hpp
#define state_hpp

#include <stdio.h>
#include <stdio.h>
#include <vector>
#include <math.h>
#include "json.hpp"
#include "geo.hpp"


using namespace std;
using json = nlohmann::json;

class vehicle
{
    double s;
    double d;
    double v;
    
    //state at time
    double state_s(double t);
    
};

class current_state
{
    public:
    
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;
    
    //double car_speed_ms = mph_to_ms(car_speed);
    
    // Previous path data given to the Planner
    vector<float> previous_path_x;
    vector<float> previous_path_y;
    // Previous path's end s and d values
    double end_path_s;
    double end_path_d;
    
    //previous x and y coordinates
    double prev_car_x,    prev_car_y;
    //current angle
    double ref_yaw;
    
    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    vehicle* sensor_fusion;
    
    void load (string s);
    
};

#endif /* state_hpp */
