//
//  data_collect.hpp
//  Path_Planning
//
//  Created by Тарасов Павел on 27/10/2017.
//
//

#ifndef data_collect_hpp
#define data_collect_hpp

#include <stdio.h>
#include <vector>
#include "json.hpp"
#include "spline.h"


using namespace std;

//TODO - decode json only once

//get last 2 coords in prev and vs vd
vector<vector<double>> last_coords(string s);

//get  s,d,vs,vd coordinates of other cars
vector<vector<double>> other_coords(string s);

#endif /* data_collect_hpp */
