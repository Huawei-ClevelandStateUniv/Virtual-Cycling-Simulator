#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 23 04:53:46 2018

This code optimize the motion tractory as well as joint torques

@author: huawei
"""

import sys
sys.path.append('../')

import ipopt
import numpy as np
from models.virtual_cycling_simulator_system import VirtualCyclingSimulator
from examples.inverse_kinematics import generate_joints
from examples.pedal_force import get_pedal_force
from examples.annimation_human_cycling import animate_cycling
from numpy import pi
import os


Nnode = 201
interval = 0.005
duration = (Nnode-1)*interval
time = np.linspace(0, duration, Nnode)
nDoF = 10

lamda = 0.999999
Human_const = np.array([0.8, 0.4, 0.4, 0.23, 0.18, 6.3, 4.1, 0.122, 0.041, 9.8])  # [l_St, l_T, l_S, d_T, d_S, m_T, m_S, i_T, i_S, g]
Exo_const = np.array([0.8, 0.4, 0.4, 0.23, 0.18, 0, 0, 0, 0, 9.8])  # [l_St, l_T, l_S, d_T, d_S, m_T, m_S, i_T, i_S, g]

#for b in range(1, 5):

bike_loads = 3.2

Ip = 0.15*bike_loads
In = 0.15*bike_loads
Cp = 1*bike_loads
Cn = 1*bike_loads

Bike_param = [0.8, np.array([0.15, 0.20]), 0.17, 200000, 10, Ip, In, Cp, Cn]
                #seat_height=1.0, pedal_center = np.array([0.15, 0.20]),
                #pedal_radius=0.17, K=10000, C=10, Ip=0.05, In=0.05,
                #Cp=1, Cn=1

lhip = np.linspace(0, Nnode*nDoF, Nnode, endpoint = False, dtype = np.int32)
lknee = lhip + 1
rhip = lhip + 2
rknee = lhip + 3
cy_x = lhip + 4
lhipd = lhip + 5
lkneed = lhip + 6
rhipd = lhip + 7
rkneed = lhip + 8
cy_xd = lhip + 9

lb_x = np.zeros(Nnode*nDoF)
lb_x[lhip] = -pi/4
lb_x[lknee] = -pi
lb_x[rhip] = -pi/4
lb_x[rknee] = -pi
lb_x[cy_x] = -2*pi

lb_x[lhipd] = -pi/4*10
lb_x[lkneed] = -pi*5
lb_x[rhipd] = -pi/4*10
lb_x[rkneed] = -pi*5
lb_x[cy_xd] = -2*pi*5

ub_x = np.zeros(Nnode*nDoF)
ub_x[lhip] = pi
ub_x[lknee] = 0.1
ub_x[rhip] = pi
ub_x[rknee] = 0.1
ub_x[cy_x] = pi

ub_x[lhipd] = pi/4*10
ub_x[lkneed] = pi*5
ub_x[rhipd] = pi/4*10
ub_x[rkneed] = pi*5
ub_x[cy_xd] = 2*pi*5

lb_u = np.zeros((Nnode-1)*int(nDoF/2-1)) - 500000
ub_u = np.zeros((Nnode-1)*int(nDoF/2-1)) + 500000

lb = np.hstack((lb_x, lb_u))
ub = np.hstack((ub_x, ub_u))

cl = np.zeros(nDoF*(Nnode -1))
cu = np.zeros(nDoF*(Nnode -1))

cy_theta = np.linspace(0.5*pi, -1.5*pi, Nnode)
cy_thetad = np.zeros_like(cy_theta) + (cy_theta[1]-cy_theta[0])/interval

model = VirtualCyclingSimulator(cy_theta, Nnode, nDoF, interval, lamda, Human_const,
                        Exo_const, Bike_param)

# the joint angles reference from inverse kinematics 
joints_ref = generate_joints(model, cy_theta, Bike_param, Exo_const, Nnode, interval) 

nlp = ipopt.problem(
            n=Nnode*nDoF + (Nnode-1)*int(nDoF/2-1),
            m=nDoF*(Nnode -1),
            problem_obj=VirtualCyclingSimulator(cy_theta, Nnode, nDoF, interval, lamda, Human_const,
                        Exo_const, Bike_param),
            lb=lb,
            ub=ub,
            cl=cl,
            cu=cu
            )
               
nlp.addOption(b'linear_solver', b'MA86')
nlp.addOption(b'max_iter', 10000)
nlp.addOption(b'hessian_approximation', b'limited-memory')
nlp.addOption(b'tol', 1e-4)
nlp.addOption(b'acceptable_tol', 1e-3)
nlp.addOption(b'max_cpu_time', 1e+5)

x_init = np.zeros(Nnode*nDoF + (Nnode-1)*int(nDoF/2-1))

x_init[lhip] = joints_ref[:, 0]+ 0.001*np.random.random(Nnode)
x_init[lknee] = joints_ref[:, 1]+ 0.001*np.random.random(Nnode)
x_init[rhip] = joints_ref[:, 2]+ 0.001*np.random.random(Nnode)
x_init[rhip] = joints_ref[:, 3]+ 0.001*np.random.random(Nnode)
x_init[cy_x] = cy_theta + 0.001*np.random.random(Nnode)

x_init[lhipd] = joints_ref[:, 4]+ 0.001*np.random.random(Nnode)
x_init[lkneed] = joints_ref[:, 5]+ 0.001*np.random.random(Nnode)
x_init[rhipd] = joints_ref[:, 6]+ 0.001*np.random.random(Nnode)
x_init[rhipd] = joints_ref[:, 7]+ 0.001*np.random.random(Nnode)
x_init[cy_xd] = cy_thetad + 0.001*np.random.random(Nnode)

x_init[Nnode*nDoF:] = -2 + 4*np.random.random((Nnode-1)*int(nDoF/2-1))
x, info = nlp.solve(x_init)

states = np.zeros((Nnode, nDoF))
tor = np.zeros((Nnode, nDoF))

for m in range(0,Nnode):
    for n in range(0,nDoF):
        states[m, n] = x[m*nDoF + n]

for m in range(0,Nnode-1):
    for n in range(0, 4):
        tor[m, n] = x[Nnode*nDoF + m*4 + n]


Pforce = get_pedal_force(Nnode, interval, nDoF, model, cy_theta, states, tor,
                         Exo_const, Bike_param)
    
newpath = 'example_result' 
if not os.path.exists(newpath):
    os.makedirs(newpath)

animate_cycling(time, states, Pforce, tor, Exo_const, Bike_param, fps = 50, filename = newpath + '/optimization.mp4')