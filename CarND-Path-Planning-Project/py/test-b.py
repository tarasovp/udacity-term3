# -*- coding: utf-8 -*-
import argparse
import base64
from datetime import datetime
import os
import shutil
import json

import numpy as np
import socketio
import eventlet
import eventlet.wsgi
#from PIL import Image
from flask import Flask
from io import BytesIO

from geo import getXY, deg2rad, distance
from scipy.interpolate import interp1d
from math import cos,sin,fabs, sqrt

#from keras.models import load_model
#import h5py
#from keras import __version__ as keras_version

sio = socketio.Server()
app = Flask(__name__)
model = None
prev_image_array = None


#последние координаты s,d
last_coords=[]
#количество точек предсказываем
#todo - предсказывать больше использовать меньше
cnt_points=50
#максимальная скорость
max_speed=21
#длина круга
max_s = 6945.554
#максимальное ускорение
max_acc=8
#стартовая линия
line=1
#линия куда хотим
line_to_change=-1
#время для изменения линии
time_to_change_line=2.5

f=None


def get_interpolated_points (new_points, ref_yaw, last_x, last_y, target_speed=max_speed, car_speed=max_speed):
	"""
	Возвращает предыдущие точки+новые по сплайну
	todo - брать от -2 координаты
	угол поворота тоже надо от них считать
	"""
	
	newpoints_xy=[getXY(a[0],a[1]) for a in new_points]
	ptsx=last_x+[a[0] for a in newpoints_xy]
	ptsy=last_y+[a[1] for a in newpoints_xy]
	
	ref_x,ref_y=last_x[-1], last_y[-1]
	
	pts=[]
	for i in range(len(ptsx)):
		shiftx = ptsx[i] - ref_x
		shifty = ptsy[i] - ref_y
		ptsx[i] = (shiftx * cos(0 - ref_yaw)) - shifty * sin(0 - ref_yaw)
		ptsy[i] = (shiftx * sin(0 - ref_yaw)) + shifty * cos(0 - ref_yaw)
	   
	s = interp1d(ptsx, ptsy, kind='cubic')     
	
	x_addon=0
	
	next_x_vals=last_x
	next_y_vals=last_y
	
	current_speed=car_speed
	#print ('car_speed=', car_speed)
	
	for i in range(1,cnt_points-len(last_x)):
		if current_speed<target_speed: current_speed=min((current_speed+max_acc*0.02,target_speed))
		if current_speed>target_speed: current_speed=max((current_speed-max_acc*0.02,target_speed))
		
		if current_speed>max_speed:
			current_speed=max_speed
		
		x_point = x_addon + current_speed*0.02
		y_point = s(x_point)

		x_addon = x_point

		x_ref = x_point
		y_ref = y_point

		x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw)
		y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw)

		x_point += ref_x
		y_point += ref_y

		next_x_vals.append(x_point)
		next_y_vals.append(y_point)

	return next_x_vals, next_y_vals


def get_max_speed(other_cars, line, my_s, my_v):
	"""
	Просто получает максимальную скорость в данном ряду
	"""
	
	nearst=max_s
	nearst_speed=0
	
	for s,d, v in other_cars:
		#если тачка в нашем ряду
		if d>line*4 and d<line*4+4:
			distance=(s-my_s)%max_s
			if distance<nearst:
				nearst=distance
				nearst_speed=v
	
	#если тачка далеко то пофигу
	if nearst>100:
		return 100
	
	#если слишком близко
	if nearst<5:
		return 1
	
	return nearst_speed+sqrt(2*max_acc*(nearst-5))
	

def line_ok (other_cars, target_line, my_s):
	"""
	определяем можем ли сдвинуться в эту полосу
	"""
	
	for a in other_cars:
		if a[1]>=target_line*4 \
										and a[1]<target_line*4+4 \
										and (a[0]-my_s)%max_s-max_s<-7 \
										and (a[0]-my_s)%max_s<5:
			#print (a, my_s, target_line)
			return False
										
	return True
										
	
	
def get_best_line(other_cars, line, my_s, my_v):
	"""
	Определяем куда лучше сдвинуться если это вообще возможно
	"""
	
	possible=[line_ok(other_cars,i,my_s) and fabs(line-i)<=1  for i in range(3)]
	#print ('possible: ', possible)
	max_speed=[get_max_speed(other_cars, i, my_s, my_v) if possible[i] else -100 for i in range(3)]
	
	best=np.argmax(max_speed)
	if max_speed[best]>max_speed[line]:
		return best
	return line
	
		
			
def get_variables(data):
	"""
	выдает основные переменные
	"""
	x,y=data['x'],data['y']
	s,d=data['s'],data['d']
	end_path_s, end_path_d= data['end_path_s'],data['end_path_d']
	prev_x, prev_y=data['previous_path_x'], data['previous_path_y']
	
	if len(prev_x)==0:
		end_path_s, end_path_d = s,d
	
	
	ref_yaw=deg2rad(data['yaw'])
	other_cars=np.array([[a[-2],a[-1], sqrt(a[-3]**2+a[-4]**2)] for a in data['sensor_fusion']])
	
	#добавляем к s скорость * время
	for i in range(len(other_cars)):
		other_cars[i][0]+=other_cars[i][2]*len(prev_x)*0.02
	
	
	#last_s,last_d=data['end_path_s'], data['end_path_d']
	
	while len(last_coords)>len(prev_x): last_coords.pop(0)
	if len(last_coords)==0:
		last_coords.append((s,d))
	
	#todo - здесь должен быть cos/sin
	if len(prev_x)==0:
		prev_x, prev_y = [x-0.001,x], [y,y]
		
	if len(prev_x)==1:
		prev_x, prev_y=[prev_x[0]-0.001,prev_x[0]], [prev_y[0],prev_y[0]]
		
	car_speed = distance(prev_x[-1],prev_y[-1],prev_x[-2],prev_y[-2])/0.02;
	
	
	return prev_x, prev_y, end_path_s, end_path_d, car_speed, ref_yaw, other_cars
				

def JMT(start, end, T):
    """
    Calculates Jerk Minimizing Trajectory for start, end and T.
    """
    a_0, a_1, a_2 = start[0], start[1], start[2] / 2.0
    c_0 = a_0 + a_1 * T + a_2 * T**2
    c_1 = a_1 + 2* a_2 * T
    c_2 = 2 * a_2
    
    A = np.array([
            [  T**3,   T**4,    T**5],
            [3*T**2, 4*T**3,  5*T**4],
            [6*T,   12*T**2, 20*T**3],
        ])
    B = np.array([
            end[0] - c_0,
            end[1] - c_1,
            end[2] - c_2
        ])
    a_3_4_5 = np.linalg.solve(A,B)
    alphas = np.concatenate([np.array([a_0, a_1, a_2]), a_3_4_5])
    return alphas

def get_new_points (end_path_s, best_line):# (end_path_s, end_path_d, line, best_line, other_cars, car_speed):
	"""
	Выдает новые точки
	"""
	
	#если линии совпадают хули тут думать
	return [(end_path_s+(i+1)*30,best_line*4+2) for i in range(3)]
		
	"""else:
		pstart_s=(end_path_s, car_speed, 0)
		pstart_d=(end_path_d, 0, 0)
		
		jmts=[]
		
		#макс за 2.5 секунды 2.5*20=50 метров
		for i in range(5,50):
			pend_s=(end_path_s+i, car_speed,0)
			pend_d=(best_line*4+2, 0,0)
			
			jmts.append((JMT(pstart_s,pend_s,time_to_change_line), JMT(pstart_s,pend_s,time_to_change_line)))"""
			
			
		



@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        f.write(json.dumps(data)+'\n')
    	
        prev_x, prev_y, end_path_s, end_path_d, car_speed, ref_yaw, other_cars=get_variables(data)
        	
        global line
        best_line=line
        
        #если мы в центре линии то пытаемся определить куда
        if end_path_d > line*4+2 - 0.1 and end_path_d < line*4+2 +0.1:
        	best_line=get_best_line(other_cars, line, end_path_s, car_speed)
        	
        new_points=get_new_points (end_path_s, best_line)#end_path_s, end_path_d, line, best_line, other_cars)
        
        
        #получаем максимальную скорость в полосе
        line_max_speed=get_max_speed(other_cars, line, end_path_s, car_speed)
        
        print ('line_max_speed:', line_max_speed)
        
        res = get_interpolated_points(new_points, ref_yaw, prev_x, prev_y, 
        	target_speed=min(line_max_speed,max_speed), car_speed=car_speed)
        
        #todo - запушить s,d
        
        send_control(res[0],res[1])
        	
        
    else:
        # NOTE: DON'T EDIT THIS.
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    #send_control([0], 0)


def send_control(next_x, next_y):
    """отправляем ответ"""
    sio.emit(
        "control",
        data={
            'next_x': next_x,
            'next_y': next_y
        },
        skip_sid=True)
        
        


if __name__ == '__main__':
    
    f=open('log.txt','w')
    
    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
    
    f.close()
    
