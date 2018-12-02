#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 23 18:55:16 2018

This code does forward simulation of cycling model

@author: huawei
"""

import sys
sys.path.append('../')

import numpy as np
from models.virtual_cycling_simulator_system import VirtualCyclingSimulator
from examples.inverse_kinematics import generate_joints
from examples.pedal_force import get_pedal_force
from examples.annimation_human_cycling import animate_cycling
from scipy.optimize import fsolve
from numpy import pi
import os

Nnode = 1001
interval = 0.001
duration = (Nnode-1)*interval
nDoF = 10

lamda = 0.999999
time = np.linspace(0, 1, Nnode)
cy_theta = np.linspace(0.5*pi, -1.5*pi, Nnode)

Human_const = np.array([0.8, 0.4, 0.4, 0.23, 0.18, 6.3, 4.1, 0.122, 0.041, 9.8]) 
                    # [l_St, l_T, l_S, d_T, d_S, m_T, m_S, i_T, i_S, g]
Exo_const = np.array([0.8, 0.4, 0.4, 0.23, 0.18, 0, 0, 0, 0, 9.8])  
                    # [l_St, l_T, l_S, d_T, d_S, m_T, m_S, i_T, i_S, g]
                    
bike_loads = 3

Ip = 0.15*bike_loads
In = 0.15*bike_loads
Cp = 1*bike_loads
Cn = 1*bike_loads

Bike_param = [0.8, np.array([0.15, 0.20]), 0.17, 200000, 10, Ip, In, Cp, Cn]
                #seat_height=1.0, pedal_center = np.array([0.25, 0.25]),
                #pedal_radius=0.17, K=10000, C=10, Ip=0.05, In=0.05,
                #Cp=1, Cn=1
                
K = 10000
D = 50
                
problem = VirtualCyclingSimulator(cy_theta, Nnode, nDoF, interval, lamda, Human_const,
                        Exo_const, Bike_param)

# the joint angles reference from inverse kinematics 
joints_ref = generate_joints(problem, cy_theta, Bike_param, Exo_const, Nnode, interval) 

cy_thetad = np.zeros_like(cy_theta) + (cy_theta[1]-cy_theta[0])/interval

states = np.zeros((Nnode, nDoF))
states[0, :4] = joints_ref[0, :4]
states[0, 4] = cy_theta[0]
states[0, 5:9] = joints_ref[0, 4:8]
states[0, 9] = cy_thetad[0]

tor = np.zeros((Nnode, 4))

Snode = 1000

for k in range(Snode):
    
    def func(x):
        q = x
        qd = (x - states[k, :5])/interval
        qdd = (qd - states[k, 5:])/interval
        
        tor[k+1, :] = K*(joints_ref[k+1, :4] - q[:4]) + D*(joints_ref[k+1, 4:] - qd[:4])
    
        f, _, _, _, _ = problem.dynamic_equations(q, qd, qdd, tor[k+1, :])
                                
        return f
    
    x0 = np.hstack((joints_ref[k+1, :4], cy_theta[k+1]))
    
    xres = fsolve(func, x0)
    
    states[k+1, :5] =  xres
    states[k+1, 5:] = (xres - states[k, :5])/interval
    
    
Pforce = get_pedal_force(Nnode, interval, nDoF, problem, cy_theta, states, tor,
                         Exo_const, Bike_param)
    
newpath = 'example_result' 
if not os.path.exists(newpath):
    os.makedirs(newpath)

animate_cycling(time, states, Pforce, tor, Exo_const, Bike_param, fps = 50, filename = newpath + '/forward_simualtion.mp4')

