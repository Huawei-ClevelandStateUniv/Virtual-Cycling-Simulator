#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 20 21:42:19 2018

Pedal model of the cycling machine

@author: huawei
"""

import numpy as np
from numpy import cos, sin, sqrt



class VirtualBicycleModel(object):
    """
    This code model the cycling constraints using contact model.
    
    The origion of the cycling cycle is at the a certain point ' o' ' at the 
    global cooridiante, and the radius of the cycling cycle is 'r'. 

    Clockwise rotation is positive!!!
    
    """
    
    def __init__(self, bike_param):
        
        """ 
        Set up foundamental parameters
        
        seat_height: seat height, in global coordiante
        pedal_center: pedalling cycle center, in global coordinate
        pedal_radius: radius of the rotor (m)
        k: stiffness of contact model
        c: damping parameter of contact model
        Inp: inertia of the positve direction, clockwise (N*m/s**2)
        Inn: inertia of the negative direction, anticlockwise (N*m/s**2)
        Cp: dampping parameter of the rotor when doing positive cycling (Nm*s/rad)
        Cn: dampping parameter of the rotor when doing negative cycling (Nm*s/rad)
        """
        
        self.seat_height = bike_param[0]
        self.pedal_center = bike_param[1]
        self.pedal_radius = bike_param[2]
        
        self.K = bike_param[3]
        self.C = bike_param[4]
        self.Ip = bike_param[5]
        self.In = bike_param[6]
        self.Cp = bike_param[7]
        self.Cn = bike_param[8]
        
    def coordinate_transfer_global_to_local_position(self, P, theta):
        """
        Transfer P from global coordinate to local coordinate with 'O' as original
        and rotate theta rad. Positive of theta is clockwise.
        
        Input:
            P: 1 dim array (2), value in global coordinate
            O: 1 dim array (2), origin of local coordinate, here is the pedal center
            theta: float, rotation rad of the coordinate
        
        Output:
            Po: colum array (2x1), value of P in local coordinate
        
        Method used:
            Tranformation matrix
        """
        O = self.pedal_center
        Pe = np.array([P[0], P[1], 1])
        
        T = np.array([[cos(theta), sin(theta), -O[0]*cos(theta)-O[1]*sin(theta)],
                        [-sin(theta), cos(theta), O[0]*sin(theta)-O[1]*cos(theta)],
                        [0, 0, 1]])
                
        Po = np.dot(T, Pe)
        
        dPo_dP = T[:2, :2]
        dPo_dtheta = np.zeros(2)
        dPo_dtheta[0] = -P[0]*sin(theta) + P[1]*cos(theta) + O[0]*sin(theta) - O[1]*cos(theta)
        dPo_dtheta[1] = -P[0]*cos(theta) - P[1]*sin(theta) + O[0]*cos(theta) + O[1]*sin(theta)
        
        return Po[:2], dPo_dP, dPo_dtheta
    
    def coordinate_transfer_global_to_local_speed(self, P, theta):
        """
        Transfer P from global coordinate to local coordinate with 'O' as original
        and rotate theta rad. Positive of theta is clockwise.
        
        Input:
            P: 1 dim array (2), value in global coordinate
            O: 1 dim array (2), origin of local coordinate, here is the pedal center
            theta: float, rotation rad of the coordinate
        
        Output:
            Po: colum array (3), value of P in local coordinate
        
        Method used:
            Tranformation matrix
        """
        
        Pe = np.array([P[0], P[1]])
        
        T = np.array([[cos(theta), sin(theta)],
                        [-sin(theta), cos(theta)]])
                
        Po = np.dot(T, Pe)
        
        dPo_dP = T
        dPo_dtheta = np.zeros(2)
        dPo_dtheta[0] = -P[0]*sin(theta) + P[1]*cos(theta)
        dPo_dtheta[1] = -P[0]*cos(theta) - P[1]*sin(theta)
        
        return Po, dPo_dP, dPo_dtheta
    
    
    def coordinate_transfer_local_to_global_force(self, F, theta):
        """
        Transfer P from local coordinate to global coordinate with 'O' as original of global system
        and rotate theta rad. Positive of theta is clockwise.
        
        Input:
            F: 1 dim array (2), force vector  in local coordinate
            theta: float, rotation rad of the coordinate
        
        Output:
            Fg: 1 dim array (2), force vector  in global coordinate
        
        Method used:
            Tranformation matrix
        """

        T = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
        Fg = np.dot(T, F)
        
        dFg_dF = T
        dFg_dtheta = np.zeros(2)
        dFg_dtheta[0] = -F[0]*sin(theta) - F[1]*cos(theta)
        dFg_dtheta[1] = F[0]*cos(theta) - F[1]*sin(theta)
        
        return Fg, dFg_dF, dFg_dtheta
    
    def pedal_contact_model(self, Tr, Trdot, Ft, Ftdot):
        """
        Contact Force based on spring-damping model. 
        Positive force point to A (from B)
        
        Input:
            Tr: colum array (2x1), target point and velocity
            Ft: colum array (2x1), actual foot point and velocity

        Output:
            F: colum array (2x1), contact force. positive value means direction
            same as positive coordinate.
        """
        
        F = self.K*(Tr - Ft) + self.C*(Trdot - Ftdot)
        
        dF_dTr = np.array([[self.K, 0], [0, self.K]])
        dF_dTrdot = np.array([[self.C, 0], [0, self.C]])
        dF_dFt = np.array([[-self.K, 0], [0, -self.K]])
        dF_dFtdot = np.array([[-self.C, 0], [0, -self.C]])
        
        return F, dF_dTr, dF_dTrdot, dF_dFt, dF_dFtdot
    
    def smooth_function(self, P1, P2, x, xin):
        """
        Smooth function for two values of one parameter change at point x
        
        Input: 
            P1: float, value level when input value less than x
            P2: float, value level when input value larger than x
            x: float, value change point
            xin: float, input value
        Output:
            P: float, value of smoothed function at input value xin
        """
        
        P = P1 + ((xin-x) + sqrt((xin-x)**2 + 1e-4))/(2*sqrt((xin-x)**2 + 1e-4))*(P2-P1)
        
        dP_dxin =  (x**2 -x*xin + 1e4)/(((xin-x)**2 + 1e-4)*np.sqrt((xin-x)**2 + 1e-4))*(P2-P1)
        
        return P, dP_dxin
    
    
    def rotor_dynamics(self, x, P, Pdot, theta_dot_rel=0):
        """
        Rotor dynamics of the bike, caculate the angluar acceleration based on the bippdal force. Implict equation
        and derivative will be given here.
        There are two inertias: positive and negative. Postive rotation is harder which has larger inertia;
        negative rotation is easier, since no cycling load is driven.
        
        Input:
            Pl: 1 dim array (2x1), left foot point location, in global coordinate
            Pr: 1 dim array (2x1), right foot point location, in global coordinate
            Pl: 1 dim array (2x1), left foot point velocity, in global coordinate
            Pr: 1 dim array (2x1), right foot point velocity, in global coordinate
            x: 1 dim array (1x3), rotor rotation angle/velcity/acceleration at current
            theta_dot_rel: float, virtual rotor rotation velocity, if theta_dot smaller than it, negative inertia will be applied;
                            otherwise, positive interia will be applied (rad)

        
        Output: 
            f: float, residue of the rotor dynamics model, in local coordinate
            Fl: column array (2x1), contact force at left foot point, in global coordinate
            Fr: column array (2x1), contact force at the right foot point, in global coordinate
            dfdx: 2 dim array (2x3), derivative of the rotor dynamics with respect to rotor states
            dfdPl: 2 dim array (2x2), derivative of the rotor dynamics with respect to left foot point location
            dfdPldot: 2 dim array (2x2), derivative of the rotor dynamics with respect to left foot point velocity
            dfdPr: 2 dim array (2x2), derivative of the rotor dynamics with respect to right foot point location
            dfdPrdot: 2 dim array (2x2), derivative of the rotor dynamics with respect to right foot point velocity
            
            dFldx: 2 dim array (2x3),  derivative of the left foot contact force with respect to rotor states
            dFldPl:  2 dim array (2x2), derivative of the left foot contact force with respect to left foot point location
            dFldPldot:  2 dim array (2x2), derivative of the left foot contact force with respect to left foot point velocity
            dFrdx:  2 dim array (2x3), derivative of the right foot contact force with respect to rotor states
            dFrdPl:  2 dim array (2x2), derivative of the right foot contact force with respect to right foot point location
            dFldPldot:  2 dim array (2x2), derivative of the left foot contact force with respect to right foot point velocity
            
        """
        
        Pl = P[:2]
        Pr = P[2:]
        Pldot = Pdot[:2]
        Prdot = Pdot[2:]
        
        Lo, dLo_dPl, dLo_dx = self.coordinate_transfer_global_to_local_position(Pl, x[0])
        Ro, dRo_dPr, dRo_dx = self.coordinate_transfer_global_to_local_position(Pr, x[0])
        
        Lodot, dLodot_dPldot, dLodot_dx = self.coordinate_transfer_global_to_local_speed(Pldot, x[0])
        Rodot, dRodot_dPrdot, dRodot_dx = self.coordinate_transfer_global_to_local_speed(Prdot, x[0])
        
        Tl = np.array([self.pedal_radius, 0])
        Tldot = np.array([0, self.pedal_radius*x[1]])
        Tr = np.array([-self.pedal_radius, 0])
        Trdot = np.array([0, -self.pedal_radius*x[1]])
        
        dTldot_dxd = np.array([0, self.pedal_radius])
        dTrdot_dxd = np.array([0, -self.pedal_radius])
        
        Fl, dFl_dTl, dFl_dTldot, dFl_dLo, dFl_dLodot = self.pedal_contact_model(Tl, Tldot, Lo,  Lodot)
        Fr, dFr_dTr, dFr_dTrdot, dFr_dRo, dFr_dRodot = self.pedal_contact_model(Tr, Trdot, Ro, Rodot)
        
        ISm, dISm_dxd= self.smooth_function(self.In, self.Ip, theta_dot_rel, x[1])
        CSm, dCSm_dxd = self.smooth_function(self.Cn, self.Cp, theta_dot_rel, x[1])
        
        f = -x[2]*ISm - Fl[1]*self.pedal_radius + Fr[1]*self.pedal_radius - CSm*x[1]
        
        dfdx = np.zeros(len(x))
        dfdP = np.zeros(4)
        dfdPdot = np.zeros(4)
        
        dfdx[0] = (-self.pedal_radius*(dFl_dLo[1, 1]*dLo_dx[1] + dFl_dLodot[1, 1]*dLodot_dx[1]) 
                        + self.pedal_radius*(dFr_dRo[1, 1]*dRo_dx[1] + dFr_dRodot[1, 1]*dRodot_dx[1]))
        
        dfdx[1] = (-x[2]*dISm_dxd - self.pedal_radius*dFl_dTldot[1, 1]*dTldot_dxd[1]
                            +self.pedal_radius*dFr_dTrdot[1, 1]*dTrdot_dxd[1] - CSm -x[1]*dCSm_dxd)
        dfdx[2] = -ISm

        dfdP[:2] = -self.pedal_radius*dFl_dLo[1, 1]*dLo_dPl[1, :]
        dfdP[2:] = self.pedal_radius*dFr_dRo[1, 1]*dRo_dPr[1, :]
        
        dfdPdot[:2] = -self.pedal_radius*dFl_dLodot[1, 1]*dLodot_dPldot[1, :]
        dfdPdot[2:] = self.pedal_radius*dFr_dRodot[1, 1]*dRodot_dPrdot[1, :]
        
        Flg, dFlg_dFl, dFlg_dx = self.coordinate_transfer_local_to_global_force(Fl, x[0])
        Frg, dFrg_dFr, dFrg_dx = self.coordinate_transfer_local_to_global_force(Fr, x[0])
        
        F = np.hstack((Flg, Frg))
        
        dFdx = np.zeros((4, len(x)))
        dFdP = np.zeros((4, 4))
        dFdPdot = np.zeros((4, 4))
        
        dFdx[:2, 0] = (dFlg_dx + np.dot(dFlg_dFl, np.dot(dFl_dLo, dLo_dx)) 
                                + np.dot(dFlg_dFl, np.dot(dFl_dLodot, dLodot_dx)))
        dFdx[:2, 1] = np.dot(dFlg_dFl, np.dot(dFl_dTldot, dTldot_dxd))
        
        dFdx[2:, 0] = (dFrg_dx + np.dot(dFrg_dFr, np.dot(dFr_dRo, dRo_dx)) 
                                + np.dot(dFrg_dFr, np.dot(dFr_dRodot, dRodot_dx)))
        dFdx[2:, 1] = np.dot(dFrg_dFr, np.dot(dFr_dTrdot, dTrdot_dxd))

        dFdP[:2, :2] = np.dot(dFlg_dFl, np.dot(dFl_dLo, dLo_dPl))
        dFdP[2:, 2:] = np.dot(dFrg_dFr, np.dot(dFr_dRo, dRo_dPr))
        
        dFdPdot[:2, :2] = np.dot(dFlg_dFl, np.dot(dFl_dLodot, dLodot_dPldot))
        dFdPdot[2:, 2:] = np.dot(dFrg_dFr, np.dot(dFr_dRodot, dRodot_dPrdot))
        
        return (f, F, dfdx, dfdP, dfdPdot, dFdx, dFdP, dFdPdot)
    
    
        
    
    