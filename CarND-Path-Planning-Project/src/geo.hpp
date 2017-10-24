//
//  geo.hpp
//  Path_Planning
//
//  Created by Тарасов Павел on 24/10/2017.
//
//

#ifndef geo_hpp
#define geo_hpp

#include <stdio.h>
#include <vector>
#include <math.h>
using namespace std;

double deg2rad(double x) ;
double rad2deg(double x) ;
double distance(double x1, double y1, double x2, double y2);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

//speed mph to m/s ans m/s to mph
float ms_to_mph (float ms);
float mph_to_ms (float mph);


#endif /* geo_hpp */
