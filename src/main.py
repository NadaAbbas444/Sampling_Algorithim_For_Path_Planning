# -*- coding: utf-8 -*-
import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import matplotlib.patches as patches
from scipy import interpolate
from fmt import FMT
from batch_informed_trees import BITStar
from dubins_path import dupin_smooth
from clean_informed import IRrtStar

def Add_theta(path_x,path_y):
    path = []
    for i in range(len(path_x)-1):
        x0, y0 = path_x[i], path_y[i]
        x1, y1 = path_x[i+1], path_y[i+1]
        dx, dy = x1 - x0, y1 - y0
        angle = math.atan2(dy, dx) * 180 / math.pi
        angle = round(angle / 30) * 30
        path.append((x0, y0, angle))
    path.append((path_x[-1], path_y[-1], angle))
    return path
def Informed_RRT_star(map_matrix,x_start,x_goal):
    step_len= 1
    goal_sample_rate=0.5
    search_radius=60
    iter_max=20000
    rrt_star = IRrtStar(map_matrix,x_start, x_goal, step_len, goal_sample_rate, search_radius, iter_max)
    path_x,path_y= rrt_star.planning()
    return(path_x,path_y)

def FMT_Algorithim(map_matrix,x_start,x_goal):
    sample_numbers = 200
    search_radius = 400
    path=[]
    fmt = FMT(x_start, x_goal, search_radius,map_matrix,sample_numbers)
    path_x,path_y=fmt.Planning()
    return( path_x,path_y)

def BIT_star(map_matrix,x_start,x_goal):
    eta = 200 # control the trade off between exploration and exploitation 
    iter_max = 1800
    bit = BITStar(x_start, x_goal, eta, iter_max,map_matrix)
    path_x,path_y = bit.planning()
    return (path_x,path_y) 

def main():
    # # ----------------------------------- map2_Test ---------
    # map_matrix = np.array(np.loadtxt("maps/np_map0_dilated.txt"))
    # x_start = (140, 10)  # Starting node
    # x_goal = (140, 60)  # Goal node
    # #----------------------------------- map0_Test ---------
    map_matrix = np.array(np.loadtxt("maps/np_map0_dilated.txt"))
    x_start = (80,20)
    x_goal = (75,90)
    # #----------------------------------- map1_Test ---------
    # map_matrix = np.array(np.loadtxt("maps/np_map1_dilated.txt"))
    # x_start = (80, 20)  # Starting node
    # x_goal = (75, 140)  # Goal node

    max_c = 0.3  # max curvature (0.22 for FMT and 0.25 BIT and 0.9 or more for Informed)
    # path_x,path_y = FMT_Algorithim(map_matrix,x_start,x_goal)
    # path_x,path_y = BIT_star(map_matrix,x_start,x_goal)
    path_x,path_y = Informed_RRT_star(map_matrix,x_start,x_goal) #please go to checker and change step_size to 1.5
    path = Add_theta(path_x,path_y)
    new_path=dupin_smooth(path, max_c,map_matrix)


if __name__ == '__main__':
    main()