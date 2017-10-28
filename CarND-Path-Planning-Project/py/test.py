import argparse
import base64
from datetime import datetime
import os
import shutil

import numpy as np
import socketio
import eventlet
import eventlet.wsgi
#from PIL import Image
from flask import Flask
from io import BytesIO

from geo import getXY

#from keras.models import load_model
#import h5py
#from keras import __version__ as keras_version

sio = socketio.Server()
app = Flask(__name__)
model = None
prev_image_array = None

nextx=[]
nexty=[]


@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        print (data)
        x,y=data['x'],data['y']
        s,d=data['s'],data['d']
        last_s,last_d=data['end_path_s'], data['end_path_d']
        prev_x, prev_y=data['previous_path_x'], data['previous_path_y']
        if len(prev_x)==0:
        	last_s,last_d = s, d
        
        
        	
        for i in range(1,50-len(prev_x)):
        	#prev_x.append(last_x+i)
        	#prev_y.append(last_y)
        	tmp=getXY(last_s+i,6)
        	prev_x.append(tmp[0])
        	prev_y.append(tmp[1])
        
        send_control(prev_x, prev_y)
    else:
        # NOTE: DON'T EDIT THIS.
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    #send_control([0], 0)


def send_control(next_x, next_y):


    print (next_x, next_y)
    
    sio.emit(
        "control",
        data={
            'next_x': next_x,
            'next_y': next_y
        },
        skip_sid=True)
        
        


if __name__ == '__main__':
    
    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
