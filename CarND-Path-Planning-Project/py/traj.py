# -*- coding: utf-8 -*-
from ptg import JMT
from helpers import Vehicle
from constants import MAX_JERK, MAX_ACCEL
import numpy as np

from cost import cost_traj

weights=np.array([1000,5,1000,20,1000,50])

def gen_trajectories (car_speed, best_line, end_path_s, end_path_d):

    all_goals=[]
    delta_d=best_line*4+2-end_path_d

    vehicle = Vehicle([end_path_s,car_speed,0,end_path_d, 0,0])
    
    rt = [float(i)/2 for i in range(1,10)]

    start_s = [end_path_s, car_speed, 0]
    start_d = [end_path_d, 0, 0]

    for t in rt:
        for acc in [0]:
            target=vehicle.state_in(t)
            target[0]+=acc*t
            target[3]+=delta_d
            all_goals.append((target[:3],target[3:],t))

    trajectories = []
    for goal in all_goals:
        s_goal, d_goal, t = goal
        s_coefficients = JMT(start_s, s_goal, t)
        d_coefficients = JMT(start_d, d_goal, t)
        trajectories.append(tuple([s_coefficients, d_coefficients, t]))

    return trajectories
    
def get_best_trajectrory (trajectories, other_vehicles):
	rr=[]
	for i in range(len(trajectories)):
		rr.append(cost_traj(trajectories[i], other_vehicles))

	tmp= np.dot(np.array(rr), weights.T)
	best_traj=np.argmin(tmp)
	return best_traj, tmp[best_traj]

