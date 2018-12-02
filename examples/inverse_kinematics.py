#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Dec  2 01:01:30 2018

@author: huawei
"""
import numpy as np
from numpy import sin, cos, pi
import matplotlib.pyplot as plt
from scipy.optimize import fsolve

def generate_joints(problem, cy_theta, Bike_param, Exo_const, Nnode, interval):

    duration = (Nnode-1)*interval
    time = np.linspace(0, duration, Nnode)
    
    Pedal_A = np.array([Bike_param[2]*cos(cy_theta) + Bike_param[1][0], Bike_param[2]*sin(cy_theta) + Bike_param[1][1]])
    Pedal_B = np.array([Bike_param[2]*cos(cy_theta-pi) + Bike_param[1][0], Bike_param[2]*sin(cy_theta-pi) + Bike_param[1][1]])
    
    Pedal_Av = np.diff(Pedal_A)
    Pedal_Bv = np.diff(Pedal_B)
    
    joints = np.zeros((Nnode, 8))
    joints[0, :] = np.array([pi/2, -pi/2, pi/6, -pi/6, -pi, pi, pi, -pi])
    
    for k in range(Nnode-1):
        
        def func(x):
            P, Pdot, _, _, _ = problem.FootPoint(x[:4], x[4:], Exo_const)
            
            res = np.zeros(8)
            res[:2] = P[:2] - Pedal_A[:, k]
            res[2:4] = P[2:4] - Pedal_B[:, k]
            res[4:6] = Pdot[:2] - Pedal_Av[:, k]
            res[6:8] = Pdot[2:4] - Pedal_Bv[:, k]
            
            return res
        
        if k == 0:
            x0 = joints[k, :]
        else:
            x0 = joints[k-1, :]
        
        xres = fsolve(func, x0)
        
        joints[k, :] = xres
        
    sticks = np.zeros((Nnode, 6))
    
    sticks[:, 0] = 0
    sticks[:, 1] = Exo_const[0]
    
    sticks[:, 2] = Exo_const[1]*sin(joints[:, 0])
    sticks[:, 3] = Exo_const[0] - Exo_const[1]*cos(joints[:, 0])
    
    sticks[:, 4] = Exo_const[1]*sin(joints[:, 0]) + Exo_const[2]*sin(joints[:, 0] + joints[:, 1])
    sticks[:, 5] = Exo_const[0] - Exo_const[1]*cos(joints[:, 0]) - Exo_const[2]*cos(joints[:, 0] + joints[:, 1])
    
    
    Snode = Nnode-1
    plt.figure()
    plt.plot(time[:Snode], joints[:Snode, 0], 'r-', label = 'left hip')
    plt.plot(time[:Snode], joints[:Snode, 1], 'g-', label = 'left knee')
    plt.plot(time[:Snode], joints[:Snode, 2], 'r--', label = 'right hip')
    plt.plot(time[:Snode], joints[:Snode, 3], 'g--', label = 'right knee')
    plt.legend()
    plt.xlabel('time')
    plt.xlabel('Angles (rad)')
    plt.show(block=True)
    
    plt.figure(figsize=(6,12))
    for k in range(Snode):
        plt.plot(sticks[k, [0, 2, 4]], sticks[k, [1, 3, 5]], 'b-')
    plt.plot(Pedal_A[0, :Snode], Pedal_A[1, :Snode], 'r.')
    plt.plot(Pedal_B[0, :Snode], Pedal_B[1, :Snode], 'g.')
    plt.title('Stick plot of inverse kinematics')
    plt.show(block=True)
    
    return joints