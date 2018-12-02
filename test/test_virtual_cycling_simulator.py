#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 23 03:09:54 2018

This code is to check derivative of cycling model for optimization

@author: huawei
"""

import numpy as np
from cycling_model_optimization import CyclingModelOpt

Nnode = 11
nDoF = 10
interval = 0.01
cy_theta = np.random.random(Nnode)
x = np.random.random(Nnode*nDoF + (Nnode-1)*4)
lamda = 0.9
Human_const = np.array([0.8, 0.4, 0.4, 0.23, 0.18, 6.3, 4.1, 0.122, 0.041, 9.8])  # [l_St, l_T, l_S, d_T, d_S, m_T, m_S, i_T, i_S, g]
Exo_const = np.array([0.8, 0.4, 0.4, 0.23, 0.18, 4.3, 2.1, 0.082, 0.021, 9.8])  # [l_St, l_T, l_S, d_T, d_S, m_T, m_S, i_T, i_S, g]

Bike_param = [0.8, np.array([0.15, 0.20]), 0.17, 10000, 10, 0.05, 0.05, 1, 1]

                #seat_height=1.0, pedal_center = np.array([0.25, 0.25]),
                #pedal_radius=0.17, K=10000, C=10, Ip=0.05, In=0.05,
                #Cp=1, Cn=1

model = CyclingModelOpt(cy_theta, Nnode, nDoF, interval, lamda, Human_const, Exo_const, Bike_param)

obj0 = model.objective(x)
grad0 = model.gradient(x)
cons0 = model.constraints(x)

jac0_ele = model.jacobian(x)
(row, col) = model.jacobianstructure()
jac0 = np.zeros((nDoF*(Nnode-1), len(x)))
jac0[row, col] = jac0_ele

delta = 1e-4
unit = np.eye(len(x))

grad_pd = np.zeros_like(x)
jac_pd = np.zeros((nDoF*(Nnode-1), len(x)))

for m in range(len(x)):
    
    obj_u = model.objective(x+delta*unit[m, :])
    obj_d = model.objective(x-delta*unit[m, :])
    grad_pd[m] = (obj_u - obj_d)/(2*delta)
    
    cons_u = model.constraints(x+delta*unit[m, :])
    cons_d = model.constraints(x-delta*unit[m, :])
    jac_pd[:, m] = (cons_u - cons_d)/(2*delta)
    
    
diff_grad = grad0 - grad_pd
diff_jac = jac0 - jac_pd

max_diff_grad = np.max(np.abs(diff_grad))
max_diff_jac = np.max(np.abs(diff_jac))

print('The maximum number of diff_grad is: ' + str(max_diff_grad))
print('The maximum number of diff_jac is: ' + str(max_diff_jac))