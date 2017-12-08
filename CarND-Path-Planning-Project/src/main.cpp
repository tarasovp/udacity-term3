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
//#include "state.hpp"

#define num_state 3

//maximal acceleration, 9 m/s^2
#define max_acc 9

//number of points to calc -- 3 seconds planning
#define number_of_points 50

//max speed in m/s (49.2 mph)
#define max_speed 22.0

using namespace std;

// for convenience
using json = nlohmann::json;
int lane = 1;
int target_lane=-1;

std::fstream fs;



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


int current_line (float car_d)
{
    return round ((car_d-2)/4);
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
    return speed_next/2;
    
}


vector <float> next_car (vector<vector<float>> sensor_fusion,
                         float car_s, float line, int time_to_calc, int forward =1 )
{
    int cnt_cars = sensor_fusion.size();
    float dist = 9999;
    float ns = -1;
    for (int j=0;j<cnt_cars; j++)
    {
        double next_s=sensor_fusion[j][5];
        double next_d=sensor_fusion[j][6];
        double vx=sensor_fusion[j][3];
        double vy=sensor_fusion[j][4];
        double next_car_speed=sqrt(vx*vx+vy*vy);
        
        next_s += next_car_speed * time_to_calc * 0.02;
        
        //todo - when next_s > max_s ....
        if ((next_d>line*4) && (next_d<line*4+4) &&
             ((next_s> car_s && forward) ||
            (next_s< car_s && !forward)))
        {
            float tmp = fabs(next_s-car_s);
            if (tmp<dist)
            {
                dist = tmp;
                ns = next_car_speed;
            }
        }
        
    }

    return {dist, ns};
}




double is_line_safe (int line, float car_s, float car_v, vector<vector<float>> sensor_fusion, float time_to_calc  )
{
   
    float dist = 99999;
    
     //for 3 seconds
    for (int time=0; time<15;time+=1)
    {
        float s = car_s+car_v*time* 0.02;
        for (int dir=0;dir<=1;dir++)
        {
            auto nc = next_car(sensor_fusion,car_s, line,time_to_calc, dir);
            float pos = car_s+nc[0]+nc[1]*(time+time_to_calc)* 0.02;
            
            float tmp_dist =abs(pos-s);
            if (abs(pos-s)<dist) dist = abs(pos-s);
            
            //cout << "line=" << line << " dir = " << dir <<
            //" pos=" << pos << " car_s=" << car_s << " tmp_dist:" <<  tmp_dist << "dist:" << dist <<  endl;
            
            
           
        }
        
    }
    
    //cout << "line: "<< line << " dist " << dist << endl;
    
    if (dist < 30) return 0;
    return 1;
    
    
}

int main() {
  uWS::Hub h;
    /*
    string l;
    ifstream myfile ("log.txt");
    fs.open ("out.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    
    
    while ( getline (myfile,l) )
    {
        try{
        auto j = json::parse(l);
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
        
        //fs << l << endl;
        fs << is_line_safe(0, car_s, car_speed, sensor_fusion,0  ) << " ";
        fs << is_line_safe(1, car_s, car_speed, sensor_fusion,0  ) << " ";
        fs << is_line_safe(2, car_s, car_speed, sensor_fusion,0  ) << " ";
        fs << endl;
        }
        catch(const exception e){
            cout <<"exception!" << endl;
            
        }
        
    }
    myfile.close();
    cout <<"ok!" << endl;
    return 0;
  
    */
  fs.open ("log.txt", std::fstream::in | std::fstream::out | std::fstream::app);


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
        fs << s << "\n";
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
            
            ;
            
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            int prev_size = previous_path_x.size();
            
            double prev_car_x, prev_car_y;
            
            vector<double> ptsx;
            vector<double> ptsy;
            
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            //cout << "aaaa" << endl;
            cout <<is_line_safe(0, car_s, car_speed, sensor_fusion,previous_path_x.size()  ) << " "
            << is_line_safe(1, car_s, car_speed, sensor_fusion,previous_path_x.size()  ) << " " <<
             is_line_safe(2, car_s, car_speed, sensor_fusion,previous_path_x.size()  ) << endl;
            
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
            
            
           
            
            auto nc  =next_car(sensor_fusion, end_path_s, lane, previous_path_x.size());
            float max_line_speed = get_max_speed_for_line(nc[0],nc[1],car_speed);
            
            //cout << "lane:" << lane <<" around: " << cars_arround[lane][1][0] << endl;
            
            //if we see the next car and not in lane change
            if (nc[0]<100 and target_lane==-1)
            {
                int free_lines[3]={0,0,0};
                float approx_dist_1min[3]={0,0,0};
                
                
                float best=lane;
                float best_dist=0;
                
                for (int i=0;i<3;i++)
                {
                    free_lines[i]=is_line_safe(i, end_path_s, car_speed, sensor_fusion, previous_path_x.size());
                    
                    auto tmp =next_car(sensor_fusion, end_path_s, i,previous_path_x.size() );
                    approx_dist_1min[i]=tmp[0];
                    
                    if (tmp[0]>best_dist and free_lines[i]) {
                       best=i;
                       best_dist=tmp[0];
                    }
                    
                }
                
                if (best>lane and free_lines[lane+1]==1)
                    target_lane=lane+1;
                
                if (best<lane and free_lines[lane-1]==1)
                    target_lane=lane-1;
                
                if (target_lane!=-1) cout << "change line " << target_lane << endl;
                
                //cout << "free:" << free_lines[0] << " " <<
                //                free_lines[1] << " " <<
               // free_lines[2] << endl;
                
                //cout << "aprox:" << approx_dist_1min[0] << " " <<
               // approx_dist_1min[1] << " " <<
                //approx_dist_1min[2] << endl;

                
                
                
            }

            
            
            
            if (max_line_speed<car_speed and target_speed>0.5) target_speed-=0.25;
            if (max_line_speed>car_speed and target_speed<max_speed)
                target_speed+=0.25;
            
            
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
              
              
            //convert to mph
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
    
    
    fs.close();

}
