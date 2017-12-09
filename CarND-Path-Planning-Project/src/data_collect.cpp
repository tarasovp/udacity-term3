//
//  data_collect.cpp
//  Path_Planning
//
//  Created by Тарасов Павел on 27/10/2017.
//
//

#include "data_collect.hpp"

// for convenience
using json = nlohmann::json;
int lane = 1;
int target_lane=-1;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

