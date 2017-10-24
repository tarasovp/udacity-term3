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
#include "spline.h"
#include "geo.hpp"
#include <algorithm>

#define num_state 3

//maximal acceleration, 9 m/s^2
#define max_acc 9

//number of points to calc -- 3 seconds planning
#define number_of_points 15

//max speed in m/s (49.2 mph)
#define max_speed 22.0

using namespace std;

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


//we have to follow the line
vector<double> follow_the_line (double s, double s1, double next_car_s, double next_car_s1)
{

  vector<double> res;
  return res;

  //if the next car if very far or moving very fast
  /*if (next_car_s > s + 90 || next_car_s < 0 || next_car_s1 > max_speed) {
    //

  }
*/
  //otherwise we have to solve optimization problem,
  //our goal is to go the same speed as the next car just behind it
  return res;

}

//change line. we've to chacke that it's possible and than do it
vector<vector<double>> change_line (double s, double s1, double d, double next_car_s, double next_car_s1,
                                    double next_car_new_line, double next_car_new_lines1,
double prev_car_new_line, double prev_car_new_lines1) {

  vector<vector<double>>  res;
  return res;

}


//get maximal speed for self driving in the lane
//we have to be able to stop if car in front starts stopping with max speed
//a little bit inaccruate - do not use time lag, so add +5 meter
float get_max_speed_for_line(double dist_next, double speed_next, double speed_my )
{
    //if 100+ meter
    if (dist_next>100 || (speed_next>speed_my && dist_next>10)) return 100;
    
    //if car is far away
    if (dist_next>10+speed_my/2)
    {
        float d=max(dist_next-10,0.0);
        return speed_next+sqrt(2*max_acc*d);
    }
    
    //else we have to slow down anyway
    return speed_next*0.9;
    
    
    
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  //todo - fix path
  string map_file_ = "../../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
    double target_speed=0;
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




  h.onMessage([&target_speed, &max_s, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          double car_speed = mph_to_ms(j[1]["speed"]);
            
          //double car_speed_ms = mph_to_ms(car_speed);

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
            
            int cnt_cars=sensor_fusion.size();
            json msgJson;
            
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            int prev_size = previous_path_x.size();
            
            double prev_car_x, prev_car_y;
            
            vector<double> ptsx;
            vector<double> ptsy;
            
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            if (prev_size < 2)
            {
                prev_car_x = car_x - cos(car_yaw);
                //if (cos(car_yaw) == 0) prev_car_x -= 0.1;
                prev_car_y = car_y - sin(car_yaw);
                
                
            }
            else
            {
                
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];
                
                prev_car_x = previous_path_x[prev_size - 2];
                prev_car_y = previous_path_y[prev_size - 2];
                ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
                
                //actual car speed
                car_speed = distance(ref_x,ref_y,prev_car_x,prev_car_y)/0.02;
                
                //if (prev_car_x == car_x) prev_car_x -= 0.1;
            }
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(ref_y);
            
            vector<double>   last_coords=getFrenet(ref_x,
                                                   ref_y,
                                                   ref_yaw, map_waypoints_x, map_waypoints_y
                                                   );
            
            
            //set car coordinates to last s and d
            car_s = last_coords[0];
            car_d = last_coords[1];
            
            //if lane already changed
            if (fabs(target_lane*4+2 - car_d)<0.3)
            {
                lane=target_lane;
                target_lane=-1;
            }
            
            
            //dist to the next car
            double max_dist=9999;
            double next_speed=9999;
            
            //cars arround - distance and speed
            double cars_arround[3][2][2]={
                {{-max_s,0},{max_s,0}},
                {{-max_s,0},{max_s,0}},
                {{-max_s,0},{max_s,0}}};
            
            
            for (int i=0; i < cnt_cars; i++)
            {
                double next_s=sensor_fusion[i][5];
                double next_d=sensor_fusion[i][6];
                double vx=sensor_fusion[i][3];
                double vy=sensor_fusion[i][4];
                double next_car_speed=sqrt(vx*vx+vy*vy);
                
                //dirty approximation of car position in future
                next_s+=next_speed*0.02*prev_size;
                
                double dist=(next_s-car_s);
                if (dist<0) dist+=max_s;
                
                
                //distance from the back
                double back_dist = dist-max_s;
                
                //for each line find next and previous car
                for (int j=0;j<3;j++)
                {
                    //check that car is raw in the line
                    if (next_d>4*j-1 && next_d<4*j+5)
                    {
                        if (dist<cars_arround[j][1][0])
                        {
                            cars_arround[j][1][0]=dist;
                            cars_arround[j][1][1]=next_car_speed;
                        }
                    
                        if (back_dist>cars_arround[j][0][0])
                        {
                            cars_arround[j][0][0]=back_dist;
                            cars_arround[j][0][1]=next_car_speed;
                        }
                        
                    }
                }
                
                if (next_d>4*lane && next_d<4*lane+4)
                {
                    // id, x, y, vx, vy, s, d
                    if (dist<max_dist)
                    {
                        max_dist=dist;
                        
                        next_speed=sqrt(vx*vx+vy*vy);
                    }
                    
                }
                
                    
                
               
            }
            
            
            //distance to the next car in current line
            max_dist=cars_arround[lane][1][0];
            next_speed=cars_arround[lane][1][0];
            
            cout << "lane:" << lane <<" around: " << cars_arround[lane][1][0] << endl;
            
            //if we see the next car and not in lane change
            if (max_dist<100 and target_lane==-1)
            {
                int free_lines[3]={0,0,0};
                float approx_dist_1min[3]={0,0,0};
                
                float safely=20+car_speed/2;
                
                
                for (int i=0;i<3;i++)
                {
                    free_lines[i]=cars_arround[i][0][0]<-1*safely && cars_arround[i][1][0]>safely;
                    
                    //approximate free space in minute TODO - 1 min
                    approx_dist_1min[i]=cars_arround[i][1][0];
                    //+cars_arround[i][1][1]*60;
                    
                }
                
                
                
                if (lane==0 && free_lines[1]==1 &&
                    approx_dist_1min[1]>approx_dist_1min[0]) target_lane=1;
                
                if (lane==2 && free_lines[1]==1 &&
                    approx_dist_1min[1]>approx_dist_1min[2]) target_lane=1;
                
                if (lane==1)
                {
                    int t=-1;
                    if (free_lines[0] && free_lines[2])
                    {
                        if (approx_dist_1min[0]>approx_dist_1min[2]) t=2;
                        else t=0;
                    }
                    else
                    {
                        if (free_lines[0]) t=0;
                        if (free_lines[2]) t=2;
                    }
                    
                    
                    if (t>-1 and approx_dist_1min[t]>approx_dist_1min[1])
                        target_lane=t;
                    
                }
                
                
                
            }

            
            double max_line_speed=get_max_speed_for_line(max_dist, next_speed, car_speed);
            
            //if target lane have to look for 2 lines
            if (target_lane>-1)
            {
                max_dist=cars_arround[target_lane][1][0];
                next_speed=cars_arround[target_lane][1][0];
                
                double max_line_speed2=get_max_speed_for_line(max_dist, next_speed, car_speed);
                max_line_speed=min(max_line_speed2,max_line_speed);
                
            }
            
            cout << "dist:" << max_dist << " speed" <<  car_speed << " nspeed:"  <<  next_speed << " max:" << max_line_speed << endl;
            
            
            if (max_line_speed<car_speed and target_speed>0.5) target_speed-=0.25;
            if (max_line_speed>car_speed and target_speed<max_speed)
                target_speed+=0.25;
            
            cout <<"target_speed" << target_speed << endl;
            
            //if (max_dist<20) target_speed-=0.5;
            //if (max_dist>=20 && target_speed<49.5) target_speed+=0.5;

          
            int togo=lane;
            if (target_lane!=-1) togo=target_lane;

          for (int i = 0; i < 3; i++)
          {
            vector<double> nextvp = getXY(last_coords[0] + i * 30 + 30, (2 + 4 * togo),
                                          map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(nextvp[0]);
            ptsy.push_back(nextvp[1]);

          }

          

          for (int i = 0; i < ptsx.size(); i++)
          {
            double shiftx = ptsx[i] - ref_x;
            double shifty = ptsy[i] - ref_y;
            ptsx[i] = (shiftx * cos(0 - ref_yaw)) - shifty * sin(0 - ref_yaw);
            ptsy[i] = (shiftx * sin(0 - ref_yaw)) + shifty * cos(0 - ref_yaw);
            //cout <<                ptsx[i] << endl;

            if (ptsx[i] < 0 && i > 0)
            {
              cout << 'fuck!' << endl;

            }
          }


          tk::spline s;

          s.set_points(ptsx, ptsy);

          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);

          }
           
            
            
            
          double x_addon = 0;
          double current_speed=car_speed;
            
          for (int i = 1; i < number_of_points - prev_size; i++)
          {
            if (current_speed<=target_speed) current_speed=min(current_speed+max_acc*0.02,target_speed);
              if (current_speed>target_speed) current_speed=max(current_speed-max_acc*0.02,target_speed);
              
              
            //convert to mph and to 0.2 ??? todo
            double x_point = x_addon + current_speed * 0.02;
            double y_point = s(x_point);

            x_addon = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

        

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

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
  h.onHttpRequest([](uWS::HttpResponse * res, uWS::HttpRequest req, char *data,
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
