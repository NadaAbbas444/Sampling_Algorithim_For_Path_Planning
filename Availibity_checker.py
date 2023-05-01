# -*- coding: utf-8 -*-
"""FMT.ipynb

Original file is located at
    https://colab.research.google.com/drive/15UZySk2iueEvuoWKXLZLdXWhHYNCyza3
"""

import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import matplotlib.patches as patches
from scipy import interpolate
def is_valid(pose,map): 
        # print (pose)
        map = np.array(map)
        grid_pose = position_to_map(pose)
        grid_pose = (round(grid_pose[0]), round(grid_pose[1]))
        # print("pose",pose,"grip_pose",grid_pose)
        map_value = map[grid_pose[0],grid_pose[1]]
        # print('grid_pose[0],grid_pose[1]',grid_pose[0],grid_pose[1],"=", map_value)

        # Return True if free, False if occupied and self.is_unknown_valid if unknown.
        if map_value == 0:   # If free space, check vicinity as well
            return True
        else:
            return False
    
    # Transform position with respect the map origin to cell coordinates
def position_to_map(p):
    # TODO: convert world position to map coordinates. If position outside map return `[]` or `None`
    origin = [0,0]
    resolution = 1.0
    # print("p",p,"origin",origin,"resolution",resolution)
    mx = (p[0]-origin[0])/resolution 
    my = (p[1]-origin[1])/resolution
    return [mx,my] 

# Given a path, returs true if the path is not in collision and false othewise.
def CheckObstacleCollision(x1,x2, map,step_size=0.5):
    path = (x1,x2)
    waypoints = []  # upsampled points
    # TODO: Discretize the positions between 2 waypoints with step_size
    for i in range(len(path)-1):
        p1, p2 = path[i], path[i+1]
        # print("now checking these two points",p1,'and',p2)
        dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        # print ("the distance in between is","  ",dist)
        num_steps = dist / step_size
        num_steps= int(num_steps)
        # print ("the number of steps in between","  ",num_steps)
        for j in range(num_steps):
            interpolation = float(j) / num_steps  #the interpolation value for each step to find the pt we are checking right now
            #print ('interpolation', interpolation)
            #p = p1 * (1 - interpolation) + p2 * interpolation
            x = p1[0] * (1-interpolation) + p2[0] * interpolation
            y = p1[1] * (1-interpolation) + p2[1] * interpolation
            # print ('the point we are checking', (x, y))
            #print ('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%')
            waypoints.append((x,y))
            
    # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True. 
    for w in waypoints:
        # print(w)
        if is_valid(w,map) == False:
            return True
    return False

