//
//  base_safety.cpp
//  Path_Planning
//
//  Created by Тарасов Павел on 09/12/2017.
//
//

#include "base_safety.hpp"


int current_line (float car_d)
{
    return round ((car_d - 2) / 4);
}


//get maximal speed for self driving in the lane
//we have to be able to stop if car in front starts stopping with max speed
//a little bit inaccruate - do not use time lag, so add +5 meter
float get_max_speed_for_line(double dist_next, double speed_next, double speed_my )
{
    //if 100+ meter
    if (dist_next > 100 || (speed_next > speed_my && dist_next > 10)) return 100;

    //if car is far away
    if (dist_next > 10 + speed_my / 2)
    {
        float d = max(dist_next - 10, 0.0);
        return speed_next + sqrt(2 * max_acc * d);
    }

    //else we have to slow down anyway
    return speed_next / 2;

}


vector <float> next_car (vector<vector<float>> sensor_fusion,
                         float car_s, float line, int time_to_calc, int forward)
{
    int cnt_cars = sensor_fusion.size();
    float dist = 9999;
    float ns = -1;
    for (int j = 0; j < cnt_cars; j++)
    {
        double next_s = sensor_fusion[j][5];
        double next_d = sensor_fusion[j][6];
        double vx = sensor_fusion[j][3];
        double vy = sensor_fusion[j][4];
        double next_car_speed = sqrt(vx * vx + vy * vy);

        next_s += next_car_speed * time_to_calc * 0.02;

        //todo - when next_s > max_s ....
        if ((next_d > line * 4) && (next_d < line * 4 + 4) &&
                ((next_s > car_s && forward) ||
                 (next_s < car_s && !forward)))
        {
            float tmp = fabs(next_s - car_s);
            if (tmp < dist)
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
    for (int time = 0; time < 15; time += 1)
    {
        float s = car_s + car_v * time * 0.02;
        for (int dir = 0; dir <= 1; dir++)
        {
            auto nc = next_car(sensor_fusion, car_s, line, time_to_calc, dir);
            float pos = car_s + nc[0] + nc[1] * (time + time_to_calc) * 0.02;

            float tmp_dist = abs(pos - s);
            if (abs(pos - s) < dist) dist = abs(pos - s);

            //cout << "line=" << line << " dir = " << dir <<
            //" pos=" << pos << " car_s=" << car_s << " tmp_dist:" <<  tmp_dist << "dist:" << dist <<  endl;

        }

    }

    //cout << "line: "<< line << " dist " << dist << endl;

    if (dist < 30) return 0;
    return 1;


}


vector<vector<double>> smooth_path (vector <double> ptsx,
                                    vector <double> ptsy,
                                    double ref_x, double ref_y, double ref_yaw,
                                    double car_speed, double target_speed,
                                    vector<double> previous_path_x,
                                    vector<double> previous_path_y   )
{
    vector<double> next_x_vals;
    vector<double> next_y_vals;


    int prev_size = previous_path_x.size();
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
    double current_speed = car_speed;

    for (int i = 1; i < number_of_points - prev_size; i++)
    {
        if (current_speed <= target_speed) current_speed = min(current_speed + max_acc * 0.02, target_speed);
        if (current_speed > target_speed) current_speed = max(current_speed - max_acc * 0.02, target_speed);


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

    return {next_x_vals, next_y_vals};

}
