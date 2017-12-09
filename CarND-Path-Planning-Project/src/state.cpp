//
//  state.cpp
//  Path_Planning
//
//  Created by Тарасов Павел on 16/11/2017.
//
//

#include "state.hpp"

double vehicle::state_s(double t)
{
    return v+s*t;
}

void current_state::load(string s)
{
    
    auto j = json::parse(s);
    //loading main info
    car_x = j[1]["x"];
    car_y = j[1]["y"];
    car_s = j[1]["s"];
    car_d = j[1]["d"];
    car_yaw = j[1]["yaw"];
    car_speed = mph_to_ms(j[1]["speed"]);
    
    //double car_speed_ms = mph_to_ms(car_speed);
    
    // Previous path data given to the Planner
    auto previous_path_x_tmp = j[1]["previous_path_x"];
    auto previous_path_y_tmp = j[1]["previous_path_y"];
    // Previous path's end s and d values
    end_path_s = j[1]["end_path_s"];
    end_path_d = j[1]["end_path_d"];
    
    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_fusion_tmp = j[1]["sensor_fusion"];
    
    int cnt_cars=sensor_fusion_tmp.size();
    
    
    
    int prev_size = previous_path_x.size();
    
    //information about angle and current speed
    double ref_x = car_x;
    double ref_y = car_y;
    ref_yaw = deg2rad(car_yaw);
    car_speed=0;
    
    if (prev_size < 2)
    {
        prev_car_x = car_x - cos(car_yaw);
        prev_car_y = car_y - sin(car_yaw);
    }
    else
    {
        
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];
        
        prev_car_x = previous_path_x[prev_size - 2];
        prev_car_y = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
        car_speed = distance(ref_x,ref_y,prev_car_x,prev_car_y)/0.02;
    }
    
    
}
