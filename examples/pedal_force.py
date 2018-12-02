#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov 24 20:59:17 2018

This code is to calculate joint torques of exoskeletons, forces at pedals, and cycling power.

@author: huawei
"""

import sys
sys.path.append('../')

import numpy as np
from models.bicycle_model import VirtualBicycleModel


def get_pedal_force(Nnode, interval, nDoF, problem, cy_theta, traj, tor,
                         Exo_const, Bike_param):

    F_pedal = np.zeros((Nnode, 4))

    for k in range(Nnode-1):
        
        q = traj[k+1, :int(nDoF/2-1)]
        qd = traj[k+1, int(nDoF/2):-1]
        
        states_cy = np.hstack((traj[k+1, int(nDoF/2-1)], traj[k+1, int(nDoF-1)],
                           (traj[k+1, int(nDoF-1)] -traj[k, int(nDoF-1)])/interval))
        
        P, Pdot, _, _, _ = problem.FootPoint(q, qd, Exo_const)
        
        PM = VirtualBicycleModel(Bike_param)
        
        (f_cy, F_cy, dfdx_cy, dfdP_cy, dfdPdot_cy, dFdx_cy, dFdP_cy, dFdPdot_cy) =\
                                                    PM.rotor_dynamics(states_cy, P, Pdot)
        
        F_pedal[k+1, :] = -F_cy
        
    return F_pedal