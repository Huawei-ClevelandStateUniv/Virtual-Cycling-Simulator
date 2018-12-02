#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 21 16:38:07 2018

This code is to test pedal_model

@author: huawei
"""
import numpy as np
from pedal_model import VirtualPedalModel
from numpy import pi, sin, cos
from scipy.optimize import fsolve
import matplotlib.pyplot as plt


class PedalModelTest(object):
    
    """
    This code is to check the pedal model. Here are two items checking.
    
    1. forward simulation with the rotation of foot points.
    2. derivative check 
    """
    
    def __init__(self):

        self.seat_height=1.0
        self.pedal_center = np.array([0.25, 0.25])
        self.pedal_radius=0.17
        
        bike_param = [1.0, np.array([0.25, 0.25]),
                 0.17, 10000, 10, 0.05, 0.05,
                 1, 1]
        
        self.PM = VirtualPedalModel(bike_param)
        
        self._forward_simulation()
        self._derivative_check()

    # forward simulation
    def _forward_simulation(self):
        
        Nnode = 240
        time = np.linspace(0, 1.2, Nnode)
        theta = np.linspace(0, -2*np.pi, Nnode)
        
        P = np.zeros((Nnode, 4))
        Pdot = np.zeros((Nnode, 4))
        
        states = np.zeros((Nnode, 3))
        
        
        P[:, 0] = self.pedal_center[0] + self.pedal_radius*cos(theta)
        P[:, 1] = self.pedal_center[1] + self.pedal_radius*sin(theta)
        
        P[:, 2] = self.pedal_center[0] + self.pedal_radius*cos(theta+pi)
        P[:, 3] = self.pedal_center[1] + self.pedal_radius*sin(theta+pi)
        
        Pdot[:, 0] = self.pedal_radius*sin(theta)
        Pdot[:, 1] = -self.pedal_radius*cos(theta)
        
        Pdot[:, 2] = self.pedal_radius*sin(theta+pi)
        Pdot[:, 3] = -self.pedal_radius*cos(theta+pi)
        
        states[0, 0] = theta[0]
        states[0, 1] = (theta[1] - theta[0])/time[1]
        
        F = np.zeros((Nnode, 4))
        
        for i in range(Nnode-1):
            
            def func(x):
                states_i = np.hstack((states[i, :2], x))
                
                (f, F[i, :], dfdx, dfdP, dfdPdot, dFdx, dFdP, dFdPdot) =\
                 self.PM.rotor_dynamics(states_i, P[i, :], Pdot[i, :])
                            
                return f
            
            x0 = states[i, 2]
            xres = fsolve(func, x0)
            
            states[i, 2] = xres
            states[i+1, 1] = states[i, 1] + states[i, 2]*(time[1])
            states[i+1, 0] = states[i, 0] + states[i+1, 1]*(time[1])
            
        plt.figure()
        plt.plot(time, theta, 'r-', label='foot rotation')
        plt.plot(time, states[:, 0], 'g--', label='pedal rotation')
        plt.legend()
        
        plt.figure()
        plt.plot(theta, F[:, 0], 'r--', label='left foot force x direction')
        plt.plot(theta, F[:, 1], 'g--', label='left foot force y direction')
        plt.legend()
        
        plt.figure()
        plt.plot(theta, F[:, 2], 'r--', label='right foot force x direction')
        plt.plot(theta, F[:, 3], 'g--', label='right foot force y direction')
        plt.legend()
        
        
    # derivative check
    def _derivative_check(self):
        
        states = np.random.random(3)
        P = np.random.random(4)
        Pdot = np.random.random(4)
        
        delta = 0.00001    
        Unit3 = np.eye(3)
        Unit4 = np.eye(4)
        
        dfdx_pd = np.zeros(3)
        dfdP_pd = np.zeros(4)
        dfdPdot_pd = np.zeros(4)
        dFdx_pd = np.zeros((4, 3))
        dFdP_pd = np.zeros((4, 4))
        dFdPdot_pd = np.zeros((4, 4))
        
        
        (f0, F0, dfdx0, dfdP0, dfdPdot0, dFdx0, dFdP0, dFdPdot0) =\
                                 self.PM.rotor_dynamics(states, P, Pdot)
             
        for i in range(4):
            if i < 3:
                (f_u, F_u, dfdx, dfdP, dfdPdot, dFdx, dFdP, dFdPdot) =\
                 self.PM.rotor_dynamics(states+delta*Unit3[i, :], P, Pdot)
                 
                (f_d, F_d, dfdx, dfdP, dfdPdot, dFdx, dFdP, dFdPdot) =\
                 self.PM.rotor_dynamics(states-delta*Unit3[i, :], P, Pdot)
                 
                dfdx_pd[i] = (f_u - f_d)/(2*delta)
                dFdx_pd[:, i] = (F_u - F_d)/(2*delta)
        
         
            (f_u, F_u, dfdx, dfdP, dfdPdot, dFdx, dFdP, dFdPdot) =\
             self.PM.rotor_dynamics(states, P+delta*Unit4[i, :], Pdot)
             
            (f_d, F_d, dfdx, dfdP, dfdPdot, dFdx, dFdP, dFdPdot) =\
             self.PM.rotor_dynamics(states, P-delta*Unit4[i, :], Pdot)
                 
            dfdP_pd[i] = (f_u - f_d)/(2*delta)
            dFdP_pd[:, i] = (F_u - F_d)/(2*delta)
            
            (f_u, F_u, dfdx, dfdP, dfdPdot, dFdx, dFdP, dFdPdot) =\
             self.PM.rotor_dynamics(states, P, Pdot+delta*Unit4[i, :])
             
            (f_d, F_d, dfdx, dfdP, dfdPdot, dFdx, dFdP, dFdPdot) =\
             self.PM.rotor_dynamics(states, P, Pdot-delta*Unit4[i, :])
        
            dfdPdot_pd[i] = (f_u - f_d)/(2*delta)
            dFdPdot_pd[:, i] = (F_u - F_d)/(2*delta)
            
        diff_dfdx = dfdx_pd - dfdx0
        diff_dFdx = dFdx_pd - dFdx0
        
        diff_dfdP = dfdP_pd - dfdP0
        diff_dFdP = dFdP_pd - dFdP0
        
        diff_dFdPdot = dFdPdot_pd - dFdPdot0
        
        print('The maximum number in dfdx is: ' + str(np.max(np.abs(diff_dfdx))))
        print('The maximum number in dFdx is: ' + str(np.max(np.abs(diff_dFdx))))
        
        print('The maximum number in dfdP is: ' + str(np.max(np.abs(diff_dfdP))))
        
        print('The maximum number in dFdP is: ' + str(np.max(np.abs(diff_dFdP))))
        print('The maximum number in dFdPdot is: ' + str(np.max(np.abs(diff_dFdPdot))))
         
        
        
         
    