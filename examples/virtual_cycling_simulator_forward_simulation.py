#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 23 18:55:16 2018

This code does forward simulation of cycling model

@author: huawei
"""
import numpy as np
from cycling_model_optimization import CyclingModelOpt
from scipy.optimize import fsolve

Nnode = 1001
interval = 0.001
duration = (Nnode-1)*interval
nDoF = 10

lamda = 0.999999
time = np.linspace(0, 1, Nnode)
time_org = np.linspace(0, 1, 100)
cy_theta = np.linspace(0.5*np.pi, -1.5*np.pi, Nnode)

Human_const = np.array([0.8, 0.4, 0.4, 0.23, 0.18, 6.3, 4.1, 0.122, 0.041, 9.8]) 
                    # [l_St, l_T, l_S, d_T, d_S, m_T, m_S, i_T, i_S, g]
Exo_const = np.array([0.8, 0.4, 0.4, 0.23, 0.18, 0, 0, 0, 0, 9.8])  
                    # [l_St, l_T, l_S, d_T, d_S, m_T, m_S, i_T, i_S, g]
for m in range(5):                
    bike_loads = m + 1
    
    Ip = 0.15*bike_loads
    In = 0.15*bike_loads
    Cp = 1*bike_loads
    Cn = 1*bike_loads
    
    Bike_param = [0.8, np.array([0.15, 0.20]), 0.17, 200000, 10, Ip, In, Cp, Cn]
                    #seat_height=1.0, pedal_center = np.array([0.25, 0.25]),
                    #pedal_radius=0.17, K=10000, C=10, Ip=0.05, In=0.05,
                    #Cp=1, Cn=1
                        
    #Bike_param = [0.8, np.array([0.15, 0.20]), 0.17, 500000, 10, 0.05, 0.05, 1, 1]
    #                #seat_height=1.0, pedal_center = np.array([0.25, 0.25]),
    #                #pedal_radius=0.17, K=10000, C=10, Ip=0.05, In=0.05,
    #                #Cp=1, Cn=1
                    
    K = 10000
    D = 50
                    
    problem = CyclingModelOpt(cy_theta, Nnode, nDoF, interval, lamda, Human_const,
                            Exo_const, Bike_param)
    
    joints_ref_org = np.loadtxt('Joints_Cycling.txt')
    
    joints_ref = np.zeros((Nnode, 8))
    
    for n in range(8):
        joints_ref[:, n] = np.interp(time, time_org, joints_ref_org[:, n])
    
    cy_thetad = np.zeros_like(cy_theta) + (cy_theta[1]-cy_theta[0])/interval
    
    States = np.zeros((Nnode, nDoF))
    States[0, :4] = joints_ref[0, :4]
    States[0, 4] = cy_theta[0]
    States[0, 5:9] = joints_ref[0, 4:8]
    States[0, 9] = cy_thetad[0]
    
    Tor = np.zeros((Nnode, 4))
    
    Snode = 1000
    for k in range(Snode):
        
        def func(x):
            q = x
            qd = (x - States[k, :5])/interval
            qdd = (qd - States[k, 5:])/interval
            
            Tor[k+1, :] = K*(joints_ref[k+1, :4] - q[:4]) + D*(joints_ref[k+1, 4:] - qd[:4])
        
            f, _, _, _, _ = problem.dynamic_equations(q, qd, qdd, Tor[k+1, :])
                                    
            return f
        
        x0 = np.hstack((joints_ref[k+1, :4], cy_theta[k+1]))
        
        xres = fsolve(func, x0)
        
        States[k+1, :5] =  xres
        States[k+1, 5:] = (xres - States[k, :5])/interval
        
    FitName = 'Result_Open/TrajectoryResult_L'+str(lamda)+'_L'+str(bike_loads)+'_simulation.txt'
    with open(FitName,'w') as Outfile:
        StringP = ""
        for m in range(0,Nnode):
            for n in range(0,nDoF):
                StringP += str(States[m, n])
                StringP += " "
            StringP += "\n"
        Outfile.write(StringP)
        
    RcstName = 'Result_Open/Torque_L'+str(lamda)+'_L'+str(bike_loads)+'_simulation.txt'
    with open(RcstName,'w') as Outfile:
        StringP = ""
        for m in range(0,Nnode):
            for n in range(0, 4):
                StringP += str(Tor[m, n])
                StringP += " "
            StringP += "\n"
        Outfile.write(StringP)
    



