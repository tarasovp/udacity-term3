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
#include "base_safety.hpp"

#define num_state 3

//max speed in m/s (49.2 mph)
#define max_speed 22.0

using namespace std;

// for convenience
using json = nlohmann::json;
int lane = 1;
int target_lane = -1;

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



int main() {
  uWS::Hub h;

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
  double target_speed = 0;
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



          int cnt_cars = sensor_fusion.size();
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

            //actual car speed
            car_speed = distance(ref_x, ref_y, prev_car_x, prev_car_y) / 0.02;
          }

          ptsx.push_back(prev_car_x);
          ptsx.push_back(ref_x);

          ptsy.push_back(prev_car_y);
          ptsy.push_back(ref_y);

          vector<double>   last_coords = getFrenet(ref_x,
                                         ref_y,
                                         ref_yaw, map_waypoints_x, map_waypoints_y
                                                  );


          //set car coordinates to last s and d
          car_s = last_coords[0];
          car_d = last_coords[1];

          //if lane already changed
          if (fabs(target_lane * 4 + 2 - car_d) < 0.3)
          {
            lane = target_lane;
            target_lane = -1;
          }




          auto nc  = next_car(sensor_fusion, end_path_s, lane, previous_path_x.size());
          float max_line_speed = get_max_speed_for_line(nc[0], nc[1], car_speed);

          if (nc[0] < 100 and target_lane == -1)
          {
            int free_lines[3] = {0, 0, 0};
            float approx_dist_1min[3] = {0, 0, 0};


            float best = lane;
            float best_dist = 0;

            for (int i = 0; i < 3; i++)
            {
              free_lines[i] = is_line_safe(i, end_path_s, car_speed, sensor_fusion, previous_path_x.size());

              auto tmp = next_car(sensor_fusion, end_path_s, i, previous_path_x.size() );
              approx_dist_1min[i] = tmp[0];

              if (tmp[0] > best_dist and free_lines[i]) {
                best = i;
                best_dist = tmp[0];
              }

            }

            if (best > lane and free_lines[lane + 1] == 1)
              target_lane = lane + 1;

            if (best < lane and free_lines[lane - 1] == 1)
              target_lane = lane - 1;

            if (target_lane != -1) cout << "change line " << target_lane << endl;




          }




          if (max_line_speed<car_speed and target_speed>0.5) target_speed -= 0.25;
          if (max_line_speed > car_speed and target_speed < max_speed)
            target_speed += 0.25;


          int togo = lane;
          if (target_lane >= 0 and target_lane < 3)
          {
            //if changing line we have
            auto nc  = next_car(sensor_fusion, end_path_s, togo, previous_path_x.size());
            float max_line_speed_new = get_max_speed_for_line(nc[0], nc[1], car_speed);

            max_line_speed_new = min(max_line_speed_new, max_line_speed);

            togo = target_lane;
          }


          for (int i = 0; i < 3; i++)
          {
            vector<double> nextvp = getXY(last_coords[0] + i * 30 + 30, (2 + 4 * togo),
                                          map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(nextvp[0]);
            ptsy.push_back(nextvp[1]);

          }


          auto next_vals = smooth_path ( ptsx,
                                         ptsy,
                                         ref_x,  ref_y, ref_yaw,
                                         car_speed,  target_speed,
                                         previous_path_x,
                                         previous_path_y   );


          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_vals[0];
          msgJson["next_y"] = next_vals[1];

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
