#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 22 12:44:38 2018

This code connectted human, exoskeleton, and bicycle models together.
Also contains functions that generate objective, gradient, constraints, and Jacobian
for trajecotry optimization (direct collocation).

@author: huawei
"""

import sys
sys.path.append('../')

import numpy as np
from models.bicycle_model import VirtualBicycleModel
from models.Q_pydy_codegen import eval as func
from models.dQdq_pydy_codegen import eval as dfdq
from models.dQdqd_pydy_codegen import eval as dfdqd
from models.dQdqdd_pydy_codegen import eval as dfdqdd
from models.dQdf_pydy_codegen import eval as dfdf
from numpy import cos, sin
from scipy.sparse import find

class VirtualCyclingSimulator(object):
    
    def __init__(self, cy_theta, Nnode, nDoF, interval, lamda, Human_const, Exo_const, Bike_param):
        
        # general informaiton of the virtual cycling simulator, including following:
        self.cy_theta = cy_theta  # Crank angle trajectory (as reference)
        self.Nnode = Nnode        # Number of nodes in direct collocation
        self.nDoF = nDoF          # Degree of freedom of the virtual cycling system (4 here) 
        self.interval = interval  # Time interval between direct collocation nodes (usually 0.001~0.01 seconds) 
        self.lamda = lamda        # Weight prameter between crank angle tracking and joint torques (between 0 and 1)
        self.Human_const = Human_const # Human model constant information, including segment length, mass, inerial
        self.Exo_const = Exo_const     # Exoskeleton model constant information, including segment length, mass, inerial
        self.Bike_param = Bike_param   # Bicycle model constant information, including segment length, mass, inerial
        
        # index of crank angle index in optiming parameters, including state trajectories and joint torques
        self.x_cy_ind = np.linspace(0, Nnode*nDoF, Nnode, endpoint = False, dtype = np.int32) + 4
        
    def objective(self, x):
        
        # objective contains two parts: tracking of crank angle trajectories and
        # joint torques.
        
        obj = (self.lamda*np.sum((x[self.x_cy_ind] - self.cy_theta)**2) 
                + (1-self.lamda)*np.sum(x[self.Nnode*self.nDoF:]**2))/self.Nnode
        
        return obj
    
    def gradient(self, x):
        
        # gradiant is the derivative of objecitve respect to optimizing parameters
        
        grad = np.zeros_like(x)
        grad[self.x_cy_ind] = 2*self.lamda*(x[self.x_cy_ind]-self.cy_theta)/self.Nnode
        grad[self.Nnode*self.nDoF:] = 2*(1-self.lamda)*x[self.Nnode*self.nDoF:]/self.Nnode
        
        return grad
    
    def constraints(self, x):
        
        # constraints are the dyanmic equations of the virtual cycling simulator system
        # at all direct collocation nodes.
        
        cons = np.array([])
        
        for k in range(self.Nnode-1):
            
            x_p = x[k*self.nDoF:(k+1)*self.nDoF]
            x_d = x[(k+1)*self.nDoF:(k+2)*self.nDoF]
            
            T = x[self.Nnode*self.nDoF+k*int(self.nDoF/2-1):self.Nnode*self.nDoF+(k+1)*int(self.nDoF/2-1)]
            
            f, dfdq, dfdqd, dfdT = self.cycling_model(x_d, (x_d-x_p)/self.interval, T)
            
            cons = np.hstack((cons, f))
            
        return cons
    
    def jacobianstructure(self):
        
        # To find out the non-zero elements in jacobian matrix. This is achieved
        # by randomly generate jacobian matrix three times using random inputs.
        # Then find out the row and col of the non-zero elements of the jacobian
        # matrix.
        
        row = np.array([])
        col = np.array([])
        
        
        for k in range(self.Nnode-1):
            
            jac_part = np.zeros((self.nDoF, 2*self.nDoF+int(self.nDoF/2-1)))
            
            for rep in range(3):
                x_p = np.random.random(self.nDoF)
                x_d = np.random.random(self.nDoF)
                T = np.random.random(int(self.nDoF/2-1))
                
                f, dfdq, dfdqd, dfdT = self.cycling_model(x_d, (x_d-x_p)/self.interval, T)
                
                jac_part[:, :self.nDoF] += -dfdqd/self.interval
                jac_part[:, self.nDoF:2*self.nDoF] += dfdq + dfdqd/self.interval
                jac_part[:, 2*self.nDoF:] += dfdT
                
            for m in range(self.nDoF):
                row_x, col_x, _ = find(jac_part[m, :2*self.nDoF])
                row_T, col_T, _ = find(jac_part[m, 2*self.nDoF:])
                
                row_xf = row_x + k*self.nDoF+m
                row_Tf = row_T + k*self.nDoF+m
                
                col_xf = col_x + k*self.nDoF
                col_Tf = col_T + self.Nnode*self.nDoF + k*(self.nDoF/2-1)
                
                row = np.hstack((row, row_xf, row_Tf))
                col = np.hstack((col, col_xf, col_Tf))
                
        row = row.astype(int)
        col = col.astype(int)
                
        return (row, col)
    
    
    def jacobian(self, x):
        
        # jacobian matrix (only store non-zero elements).
        # this jacobian matrix include the jacobian of all nodes.
        
        jac = np.array([])
        
        for k in range(self.Nnode-1):
            
            jac_part = np.zeros((self.nDoF, 2*self.nDoF+int(self.nDoF/2-1)))
            
            x_p = x[k*self.nDoF:(k+1)*self.nDoF]
            x_d = x[(k+1)*self.nDoF:(k+2)*self.nDoF]
            
            T = x[self.Nnode*self.nDoF+k*int(self.nDoF/2-1):self.Nnode*self.nDoF+(k+1)*int(self.nDoF/2-1)]
            
            f, dfdq, dfdqd, dfdT = self.cycling_model(x_d, (x_d-x_p)/self.interval, T)
            
            jac_part[:, :self.nDoF] = -dfdqd/self.interval
            jac_part[:, self.nDoF:2*self.nDoF] = dfdq + dfdqd/self.interval
            jac_part[:, 2*self.nDoF:] = dfdT
        
            _, _, nele= find(jac_part)
            
            jac = np.hstack((jac, nele))
        
        return jac
    
    def cycling_model(self, q, qd, T):
        
        # generate constriant and jacobian of one point.
        
        q1 = q[:int(self.nDoF/2)]
        q1d = q[int(self.nDoF/2):]
        q1dd = qd[int(self.nDoF/2):]
        
        f, dfdq, dfdqd, dfdqdd, dfdT = self.dynamic_equations(q1, q1d, q1dd, T)
        
        f_full = np.zeros(self.nDoF)
        dfdq_full = np.zeros((self.nDoF, self.nDoF))
        dfdqd_full = np.zeros((self.nDoF, self.nDoF))
        dfdT_full = np.zeros((self.nDoF, int(self.nDoF/2-1)))
        
        f_full[:int(self.nDoF/2)] = q[int(self.nDoF/2):] - qd[:int(self.nDoF/2)]
        f_full[int(self.nDoF/2):] = f
        
        dfdq_full[:int(self.nDoF/2), int(self.nDoF/2):] = np.eye(int(self.nDoF/2))
        dfdq_full[int(self.nDoF/2):, :int(self.nDoF/2)] = dfdq
        dfdq_full[int(self.nDoF/2):, int(self.nDoF/2):] = dfdqd
        
        dfdqd_full[:int(self.nDoF/2), :int(self.nDoF/2)] = -np.eye(int(self.nDoF/2))
        dfdqd_full[int(self.nDoF/2):, int(self.nDoF/2):] = dfdqdd
        
        dfdT_full[int(self.nDoF/2):, :] = dfdT
        
        return f_full, dfdq_full, dfdqd_full, dfdT_full
        
    def dynamic_equations(self, q, qd, qdd, T_input):
        
        # connect all three models together. Find Wiki of this project to see
        # how do them connected.
    
        T0 = np.zeros(4)
        F0 = np.zeros(4)
        res_exo_ini = np.zeros(int(self.nDoF/2-1))  #[residuleT_lh, residuleT_lk, residuleF_rh, residuleF_rk]
        jac_exo_dfdq_ini = np.zeros(int(self.nDoF/2-1)*int(self.nDoF/2-1))
        jac_exo_dfdqd_ini = np.zeros(int(self.nDoF/2-1)*int(self.nDoF/2-1))
        jac_exo_dfdqdd_ini = np.zeros(int(self.nDoF/2-1)*int(self.nDoF/2-1))
        jac_exo_dfdF_ini = np.zeros(int(self.nDoF/2-1)*int(self.nDoF/2-1))
        
        res_human_ini = np.zeros(int(self.nDoF/2-1))  #[residuleT_lh, residuleT_lk, residuleF_rh, residuleF_rk]
        jac_human_dfdq_ini = np.zeros(int(self.nDoF/2-1)*int(self.nDoF/2-1))
        jac_human_dfdqd_ini = np.zeros(int(self.nDoF/2-1)*int(self.nDoF/2-1))
        jac_human_dfdqdd_ini = np.zeros(int(self.nDoF/2-1)*int(self.nDoF/2-1))
        
        (P, Pdot, dPdq, dPdotdq, dPdotdqd) = self.FootPoint(q[:4], qd[:4], self.Exo_const)
        
        states_cy = np.hstack((q[4], qd[4], qdd[4]))
        
        PM = VirtualBicycleModel(self.Bike_param)
        
        (f_cy, F_cy, dfdx_cy, dfdP_cy, dfdPdot_cy, dFdx_cy, dFdP_cy, dFdPdot_cy) =\
                                                    PM.rotor_dynamics(states_cy, P, Pdot)
        
        f_exo = func(q[:4], qd[:4], qdd[:4], T0, F_cy, self.Exo_const, res_exo_ini)
        jac_dfdq_exo = dfdq(q[:4], qd[:4], qdd[:4], T0, F_cy, self.Exo_const, jac_exo_dfdq_ini)
        jac_dfdqd_exo = dfdqd(q[:4], qd[:4], qdd[:4], T0, F_cy, self.Exo_const, jac_exo_dfdqd_ini)
        jac_dfdqdd_exo = dfdqdd(q[:4], qd[:4], qdd[:4], T0, F_cy, self.Exo_const, jac_exo_dfdqdd_ini)
        jac_dfdF_exo = dfdf(q[:4], qd[:4], qdd[:4], T0, F_cy, self.Exo_const, jac_exo_dfdF_ini)
        
        delta_T = T_input + f_exo
        
        f_human = func(q[:4], qd[:4], qdd[:4], delta_T, F0, self.Human_const, res_human_ini)
        jac_dfdq_human = dfdq(q[:4], qd[:4], qdd[:4], delta_T, F0, self.Human_const, jac_human_dfdq_ini)
        jac_dfdqd_human = dfdqd(q[:4], qd[:4], qdd[:4], delta_T, F0, self.Human_const, jac_human_dfdqd_ini)
        jac_dfdqdd_human = dfdqdd(q[:4], qd[:4], qdd[:4], delta_T, F0, self.Human_const, jac_human_dfdqdd_ini)
        #jac_dfdF_human = dfdf(q, qd, qdd, delta_T, F0, constants_human, jac_dfdF_ini)
        
        f_full = np.hstack((f_human, f_cy))
        
        dfdq_full = np.zeros((int(self.nDoF/2), int(self.nDoF/2)))
        dfdqd_full = np.zeros((int(self.nDoF/2), int(self.nDoF/2)))
        dfdqdd_full = np.zeros((int(self.nDoF/2), int(self.nDoF/2)))
        
        dfdT_full = np.zeros((int(self.nDoF/2), int(self.nDoF/2-1)))
        
        dfdq_full[:4, :4] = jac_dfdq_human + (jac_dfdq_exo + np.dot(jac_dfdF_exo,
                                 (np.dot(dFdP_cy, dPdq) + np.dot(dFdPdot_cy, dPdotdq))))
        
        dfdq_full[:4, 4] = np.dot(np.eye(4), np.dot(jac_dfdF_exo, dFdx_cy[:, 0]))
        dfdq_full[4, :4] = np.dot(dfdP_cy, dPdq) + np.dot(dfdPdot_cy, dPdotdq)
        dfdq_full[4, 4] = dfdx_cy[0]
        
        
        dfdqd_full[:4, :4] = jac_dfdqd_human + np.dot(np.eye(4), jac_dfdqd_exo 
                                          + np.dot(jac_dfdF_exo, np.dot(dFdPdot_cy, dPdotdqd)))
        dfdqd_full[:4, 4] = np.dot(np.eye(4), np.dot(jac_dfdF_exo, dFdx_cy[:, 1]))
        dfdqd_full[4, :4] = np.dot(dfdPdot_cy, dPdotdqd)
        dfdqd_full[4, 4] = dfdx_cy[1]
        
        dfdqdd_full[:4, :4] = jac_dfdqdd_human + np.dot(np.eye(4), jac_dfdqdd_exo)
        dfdqdd_full[:4, 4] = np.dot(np.eye(4), np.dot(jac_dfdF_exo, dFdx_cy[:, 2]))
        dfdqdd_full[4, 4] = dfdx_cy[2]
        
        dfdT_full[:4, :4] = np.eye(4)
        
        return f_full, dfdq_full, dfdqd_full, dfdqdd_full, dfdT_full
    
    
    def FootPoint(self, q, qd, constants_exosk):
        
        # kenimatics of human/exokeleton model.
        
        # from joint angle/velocity to foot locaiton and velocity
        
        P = np.zeros(4)
        Pdot = np.zeros(4)
        
        P[0] = constants_exosk[1]*sin(q[0]) + constants_exosk[2]*sin(q[0]+q[1])
        P[1] = constants_exosk[0] - (constants_exosk[1]*cos(q[0]) + constants_exosk[2]*cos(q[0]+q[1]))
        
        P[2] = constants_exosk[1]*sin(q[2]) + constants_exosk[2]*sin(q[2]+q[3])
        P[3] = constants_exosk[0] - (constants_exosk[1]*cos(q[2]) + constants_exosk[2]*cos(q[2]+q[3]))
        
        Pdot[0] = constants_exosk[1]*cos(q[0])*qd[0] + constants_exosk[2]*cos(q[0]+q[1])*(qd[0]+qd[1])
        Pdot[1] = constants_exosk[1]*sin(q[0])*qd[0] + constants_exosk[2]*sin(q[0]+q[1])*(qd[0]+qd[1])
        
        Pdot[2] = constants_exosk[1]*cos(q[2])*qd[2] + constants_exosk[2]*cos(q[2]+q[3])*(qd[2]+qd[3])
        Pdot[3] = constants_exosk[1]*sin(q[2])*qd[2] + constants_exosk[2]*sin(q[2]+q[3])*(qd[2]+qd[3])
        
        
        dPdq = np.zeros((4, 4))
        dPdotdq = np.zeros((4, 4))
        dPdotdqd = np.zeros((4, 4))
        
        dPdq[0, 0] = constants_exosk[1]*cos(q[0]) + constants_exosk[2]*cos(q[0]+q[1])
        dPdq[0, 1] = constants_exosk[2]*cos(q[0]+q[1])
        dPdq[1, 0] = constants_exosk[1]*sin(q[0]) + constants_exosk[2]*sin(q[0]+q[1])
        dPdq[1, 1] = constants_exosk[2]*sin(q[0]+q[1])
        
        dPdq[2, 2] = constants_exosk[1]*cos(q[2]) + constants_exosk[2]*cos(q[2]+q[3])
        dPdq[2, 3] = constants_exosk[2]*cos(q[2]+q[3])
        dPdq[3, 2] = constants_exosk[1]*sin(q[2]) + constants_exosk[2]*sin(q[2]+q[3])
        dPdq[3, 3] = constants_exosk[2]*sin(q[2]+q[3])
        
        dPdotdq[0, 0] = -constants_exosk[1]*sin(q[0])*qd[0] - constants_exosk[2]*sin(q[0]+q[1])*(qd[0]+qd[1])
        dPdotdq[0, 1] = -constants_exosk[2]*sin(q[0]+q[1])*(qd[0]+qd[1])
        dPdotdq[1, 0] = constants_exosk[1]*cos(q[0])*qd[0] + constants_exosk[2]*cos(q[0]+q[1])*(qd[0]+qd[1])
        dPdotdq[1, 1] = constants_exosk[2]*cos(q[0]+q[1])*(qd[0]+qd[1])
        
        dPdotdq[2, 2] = -constants_exosk[1]*sin(q[2])*qd[2] - constants_exosk[2]*sin(q[2]+q[3])*(qd[2]+qd[3])
        dPdotdq[2, 3] = -constants_exosk[2]*sin(q[2]+q[3])*(qd[2]+qd[3])
        dPdotdq[3, 2] = constants_exosk[1]*cos(q[2])*qd[2] + constants_exosk[2]*cos(q[2]+q[3])*(qd[2]+qd[3])
        dPdotdq[3, 3] = constants_exosk[2]*cos(q[2]+q[3])*(qd[2]+qd[3])
        
        dPdotdqd[0, 0] = constants_exosk[1]*cos(q[0]) + constants_exosk[2]*cos(q[0]+q[1])
        dPdotdqd[0, 1] = constants_exosk[2]*cos(q[0]+q[1])
        dPdotdqd[1, 0] = constants_exosk[1]*sin(q[0]) + constants_exosk[2]*sin(q[0]+q[1])
        dPdotdqd[1, 1] = constants_exosk[2]*sin(q[0]+q[1])
        
        dPdotdqd[2, 2] = constants_exosk[1]*cos(q[2]) + constants_exosk[2]*cos(q[2]+q[3])
        dPdotdqd[2, 3] = constants_exosk[2]*cos(q[2]+q[3])
        dPdotdqd[3, 2] = constants_exosk[1]*sin(q[2]) + constants_exosk[2]*sin(q[2]+q[3])
        dPdotdqd[3, 3] = constants_exosk[2]*sin(q[2]+q[3])
        
        return P, Pdot, dPdq, dPdotdq, dPdotdqd