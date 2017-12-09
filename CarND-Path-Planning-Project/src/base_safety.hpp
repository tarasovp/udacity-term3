//
//  base_safety.hpp
//  Path_Planning
//
//  Created by Тарасов Павел on 09/12/2017.
//
//

#ifndef base_safety_hpp
#define base_safety_hpp

#include <math.h>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "geo.hpp"
#include <algorithm>

//maximal acceleration, 9 m/s^2
#define max_acc 9

//number of points to calc -- 3 seconds planning
#define number_of_points 50


int current_line (float car_d);

//get maximal speed for self driving in the lane
//we have to be able to stop if car in front starts stopping with max speed
//a little bit inaccruate - do not use time lag, so add +5 meter
float get_max_speed_for_line(double dist_next, double speed_next, double speed_my );

vector <float> next_car (vector<vector<float>> sensor_fusion,
                         float car_s, float line, int time_to_calc, int forward = 1 );


double is_line_safe (int line, float car_s, float car_v, vector<vector<float>> sensor_fusion, float time_to_calc  );

vector<vector<double>> smooth_path (vector <double> ptsx,
                                    vector <double> ptsy,
                                    double ref_x, double ref_y, double ref_yaw,
                                    double car_speed, double target_speed,
                                    vector<double> previous_path_x,
                                    vector<double> previous_path_y);

#endif /* base_safety_hpp */
