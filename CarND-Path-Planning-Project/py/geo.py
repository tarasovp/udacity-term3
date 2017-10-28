import pandas as pd
from math import atan2, pi, fabs, sin, cos
import numpy as np

highway=pd.read_csv('../data/highway_map.csv',delimiter=' ', names=['x','y','s','dx','dy'])
maps_x=highway['x'].values
maps_y=highway['y'].values
maps_s=highway['s'].values
max_s = 6945.554

def dist(x,y):
    return ((x[0]-y[0])**2+(x[1]-y[1])**2)**(0.5)    
    
def ClosestWaypoint(x,y):
    """
    Define closest point number
    """
    d=[dist((x,y),(x1,y1)) for x1,y1 in highway[['x','y']].values]
    return int(np.argmin(d))

def  NextWaypoint( x,  y,  theta ):
    """
    next waypoint on the way
    """
    n = ClosestWaypoint(x, y)
    closestWaypoint = highway[['x','y']].values[n]
    heading = atan2( (closestWaypoint[1] - y), (closestWaypoint[0] - x) )
    angle = fabs(theta - heading)
    if angle > pi / 4:   n+=1
    return n

def distance(x,y,x1,y1): return dist((x,y),(x1,y1))

def get_frenet(x,y, theta):
    next_wp = NextWaypoint(x, y, theta, highway)
    
    prev_wp =  next_wp - 1 if next_wp>0 else len(highway)-1
    
    maps_x=highway['x'].values
    maps_y=highway['y'].values
    
    
    n_x = maps_x[next_wp] - maps_x[prev_wp]
    n_y = maps_y[next_wp] - maps_y[prev_wp]
    x_x = x - maps_x[prev_wp]
    x_y = y - maps_y[prev_wp]
    
    #find the projection of x onto n
    proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y)
    proj_x = proj_norm * n_x
    proj_y = proj_norm * n_y
    
    frenet_d = distance(x_x, x_y, proj_x, proj_y)
    
    #see if d value is positive or negative by comparing it to a center point
    
    center_x = 1000 - maps_x[prev_wp]
    center_y = 2000 - maps_y[prev_wp]
    centerToPos = distance(center_x, center_y, x_x, x_y)
    centerToRef = distance(center_x, center_y, proj_x, proj_y)
    
    if centerToPos <= centerToRef:
        frenet_d *= -1
    
    #calculate s value
    frenet_s = 0
    for i in range(prev_wp):
        frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1])
    
    frenet_s += distance(0, 0, proj_x, proj_y)
    
    return (frenet_s, frenet_d)

def deg2rad( x): return x * pi / 180 
def rad2deg( x):  return x * 180 / pi

def   getXY( s,  d):
    s=s%max_s
    prev_wp = -1
    
    while (prev_wp < len(maps_s)-1 ) and (s > maps_s[prev_wp + 1] ):
        prev_wp+=1
    
    wp2 = (prev_wp + 1) % len(highway)
    
    heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]))
    # the x,y,s along the segment
    seg_s = (s - maps_s[prev_wp])
    
    seg_x = maps_x[prev_wp] + seg_s * cos(heading)
    seg_y = maps_y[prev_wp] + seg_s * sin(heading)
    
    perp_heading = heading - pi / 2
    
    x = seg_x + d * cos(perp_heading)
    y = seg_y + d * sin(perp_heading)
    
    return (x, y)

