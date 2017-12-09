# -*- coding: utf-8 -*-
from helpers import logistic, to_equation, differentiate, nearest_approach_to_any_vehicle, get_f_and_N_derivatives
from constants import *
import numpy as np


def total_accel_cost(traj):
    s, d, T = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a_s = to_equation(s_d_dot)
    
    d_dot = differentiate(d)
    d_d_dot = differentiate(d_dot)
    a_d = to_equation(d_d_dot)
    
    total_acc = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        acc = a_s(t)+a_d(t)
        total_acc += abs(acc*dt)
    acc_per_second = total_acc / T
    
    return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC )
    
def max_accel_cost(traj):
    s, d, T = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a_s = to_equation(s_d_dot)
    
    d_dot = differentiate(d)
    d_d_dot = differentiate(d_dot)
    a_d = to_equation(d_d_dot)
    
    all_accs = [a_s(float(T)/100 * i)+a_d(float(T)/100 * i) for i in range(100)]
    max_acc = max(all_accs, key=abs)
    if abs(max_acc) > MAX_ACCEL: return 1
    else: return 0
    

def max_jerk_cost(traj):
    s, d, T = traj
    
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = differentiate(s_d_dot)
    jerk_s = to_equation(jerk)
    
    d_dot = differentiate(d)
    d_d_dot = differentiate(d_dot)
    jerk = differentiate(d_d_dot)
    jerk_d = to_equation(jerk)
    
    
    all_jerks = [jerk_s(float(T)/100 * i)+jerk_d(float(T)/100 * i) for i in range(100)]
    max_jerk = max(all_jerks, key=abs)
    if abs(max_jerk) > MAX_JERK: return 1
    else: return 0

def total_jerk_cost(traj):
    s, d, t = traj
    T=t
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = differentiate(s_d_dot)
    jerk_s = to_equation(jerk)
    
    d_dot = differentiate(d)
    d_d_dot = differentiate(d_dot)
    jerk = differentiate(d_d_dot)
    jerk_d = to_equation(jerk)
    
    total_jerk = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        j = abs(jerk_s(t))+abs(jerk_d(t))
        total_jerk += abs(j*dt)
    jerk_per_second = total_jerk / T
    return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC )

def collision_cost(traj, other_vehicles, cnt_points = 100):
    """
    Binary cost function which penalizes collisions.
    """
    
    nearest=9999
    T= traj[-1]
    q=traj[:2]


    for i in range(cnt_points):
        t=T*float(i)/cnt_points
        s=sum([q[0][j]*t**j for j in range(6)])
        d=sum([q[1][j]*t**j for j in range(6)])
   
        other_sd=[np.array(v.state_in(t))[[0,3]] for v in other_vehicles]
        #tmp_nearst = min([((s1-s)**2+(d1-d)**2)**(0.5) for s1,d1 in other_sd])
        tmp_nearst = min([(((s1-s)/2)**2+(d1-d)**2)**(0.5) for s1,d1 in other_sd])
        nearest=min([tmp_nearst, nearest])
        
    #print (nearest, 2*VEHICLE_RADIUS)
    if nearest < 2*VEHICLE_RADIUS: return 1.0
    else : return 0.0

def buffer_cost(traj,  other_vehicles, cnt_points = 100):
    """
    Penalizes getting close to other vehicles.
    """
    nearest=9999
    T= traj[-1]
    q=traj[:2]

    for i in range(cnt_points):
        t=T*float(i)/cnt_points
        s=sum([q[0][j]*t**j for j in range(6)])
        d=sum([q[1][j]*t**j for j in range(6)])
   
        other_sd=[np.array(v.state_in(t))[[0,3]] for v in other_vehicles]
        tmp_nearst = min([(((s1-s)/2)**2+(d1-d)**2)**(0.5) for s1,d1 in other_sd])
        nearest=min([tmp_nearst, nearest])
        
    #print (nearest)
    return logistic(2*VEHICLE_RADIUS / nearest)

def cost_traj(traj,  other_vehicles):
    tmp= [max_accel_cost(traj),total_accel_cost(traj),\
            max_jerk_cost(traj), total_jerk_cost(traj),\
            collision_cost(traj, other_vehicles),\
            buffer_cost(traj, other_vehicles)]
    print ('cost:', tmp)
    return tmp