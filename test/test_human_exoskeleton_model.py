#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 20 14:31:52 2018

@author: huawei
"""

import sys
sys.path.append('../')

import numpy as np
import os
from models.Q_pydy_codegen import eval as func
from models.dQdq_pydy_codegen import eval as dfdq
from models.dQdqd_pydy_codegen import eval as dfdqd
from models.dQdqdd_pydy_codegen import eval as dfdqdd
from models.dQdf_pydy_codegen import eval as dfdf
from scipy.optimize import fsolve
from annimation_human_model import animate_human_model


"""
This code is to test the generated and complied planar cycling model (c code)

Two tests are contained here: leg falling and derivative check

-- leg falling:
    both left and right leg are lifted vertically up initially. Then
    simulation the falling process
    
-- derivative check:
    check the derivative of q, u, a, and F

"""

nDoF = 4
        
def leg_fall(ltime= 2):
    
    Nnode = ltime*100
    interval = ltime/Nnode
    time = np.linspace(0, ltime, Nnode)
           
    q = np.zeros((Nnode, nDoF))  # [theta_lh(t), theta_lk(t), theta_rh(t), theta_rk(t)]
    qd = np.zeros((Nnode, nDoF))  # [omega_lh(t), omega_lk(t), omega_rh(t), omega_rk(t)]
    qdd = np.zeros((Nnode, nDoF)) # [beta_lh(t), beta_lk(t), beta_rh(t), beta_rk(t)]
    T = np.zeros((Nnode, nDoF))  # [T_lh(t), T_lk(t), T_rh(t), T_rk(t)]
    F = np.zeros((Nnode, nDoF))  # [F_lx(t), F_ly(t), F_rx(t), F_ry(t)]
    constants = np.array([1.0, 0.5, 0.4, 0.23, 0.18, 6.3, 4.1, 0.122, 0.041, 9.8])  # [l_St, l_T, l_S, d_T, d_S, m_T, m_S, i_T, i_S, g]
    res0 = np.zeros(nDoF)  #[residuleT_lh, residuleT_lk, residuleF_rh, residuleF_rk]
    
    q[0, 0] = np.pi + 0.001
    q[0, 2] = np.pi - 0.001
    
    for k in range(Nnode-1):
        def func_eval(x):
            residule = func(q[k, :], qd[k, :], x, T[k, :], F[k, :], constants, res0)
            return residule
        x0 = qdd[k, :]
        xres = fsolve(func_eval, x0)
        
        qdd[k+1, :] = xres
        qd[k+1, :] = qd[k, :] + interval*qdd[k+1, :]
        q[k+1, :] = q[k, :] + interval*qd[k+1, :]
        
    newpath = 'test_result' 
    if not os.path.exists(newpath):
        os.makedirs(newpath)

    animate_human_model(time, q, constants, filename = newpath + '/leg_fall_test.mp4')
    
    
def derivative_check():
    
    q = np.random.random(4)
    qd = np.random.random(4)
    qdd = np.random.random(4)
    T = np.random.random(4)
    F = np.random.random(4)
    
    constants = np.array([1.0, 0.5, 0.4, 0.23, 0.18, 6.3, 4.1, 0.122, 0.041, 9.8])  # [l_St, l_T, l_S, d_T, d_S, m_T, m_S, i_T, i_S, g]
    #res0 = np.zeros(self.nDoF)  #[residuleT_lh, residuleT_lk, residuleF_rh, residuleF_rk]
    jac_dfdq_ini = np.zeros((nDoF*nDoF))
    jac_dfdqd_ini = np.zeros((nDoF*nDoF))
    jac_dfdqdd_ini = np.zeros((nDoF*nDoF))
    jac_dfdf_ini = np.zeros((nDoF*nDoF))
    
    
    delta = 0.0001
    unit_mx = np.eye(4)
    
    jac_dfdq0 = dfdq(q, qd, qdd, T, F, constants, jac_dfdq_ini)
    jac_dfdqd0 = dfdqd(q, qd, qdd, T, F, constants, jac_dfdqd_ini)
    jac_dfdqdd0 = dfdqdd(q, qd, qdd, T, F, constants, jac_dfdqdd_ini)
    jac_dfdf0 = dfdf(q, qd, qdd, T, F, constants, jac_dfdf_ini)
    
    jac_dfdq_pd = np.zeros((nDoF, nDoF))
    jac_dfdqd_pd = np.zeros((nDoF, nDoF))
    jac_dfdqdd_pd = np.zeros((nDoF, nDoF))
    jac_dfdf_pd = np.zeros((nDoF, nDoF))
    
    res_up = np.zeros(nDoF)
    res_dw = np.zeros(nDoF)
    
    for m in range(nDoF):
        
        res_up = func(q+delta*unit_mx[m, :], qd, qdd, T, F, constants, res_up)
        res_dw = func(q-delta*unit_mx[m, :], qd, qdd, T, F, constants, res_dw)
        jac_dfdq_pd[:, m] = (res_up - res_dw)/(2*delta)
                    
        res_up = func(q, qd+delta*unit_mx[m, :], qdd, T, F, constants, res_up)
        res_dw = func(q, qd-delta*unit_mx[m, :], qdd, T, F, constants, res_dw)
        jac_dfdqd_pd[:, m] = (res_up - res_dw)/(2*delta)
        
        res_up = func(q, qd, qdd+delta*unit_mx[m, :], T, F, constants, res_up)
        res_dw = func(q, qd, qdd-delta*unit_mx[m, :], T, F, constants, res_dw)
        jac_dfdqdd_pd[:, m] = (res_up - res_dw)/(2*delta)
        
        res_up = func(q, qd, qdd, T, F+delta*unit_mx[m, :], constants, res_up)
        res_dw = func(q, qd, qdd, T, F-delta*unit_mx[m, :], constants, res_dw)
        jac_dfdf_pd[:, m] = (res_up - res_dw)/(2*delta)
        
    diff_jac_dfdq = jac_dfdq0 - jac_dfdq_pd
    diff_jac_dfdqd = jac_dfdqd0 - jac_dfdqd_pd
    diff_jac_dfdqdd = jac_dfdqdd0 - jac_dfdqdd_pd
    diff_jac_dfdf = jac_dfdf0 - jac_dfdf_pd
    
    print('The maximum number in dfdq is ' + str(np.max(np.abs(diff_jac_dfdq))))
    print('The maximum number in dfdqd is ' + str(np.max(np.abs(diff_jac_dfdqd))))
    print('The maximum number in dfdqdd is ' + str(np.max(np.abs(diff_jac_dfdqdd))))
    print('The maximum number in dfdf is ' + str(np.max(np.abs(diff_jac_dfdf))))

leg_fall()
derivative_check() 
        
