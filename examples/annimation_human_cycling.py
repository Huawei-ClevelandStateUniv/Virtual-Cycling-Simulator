#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 30 13:34:36 2017

@author: huawei
"""
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

def animate_cycling(t, states, pedal_forces, torques, constants, bike_param, fps = 25, filename=None):
    """Animates the 4-link cycling model and optionally saves it to file.

    Parameters
    ----------
    t : ndarray, shape(m)
        Time array.
    states: ndarray, shape(m,p)
        State time history.
    filename: string or None, optional
        If true a movie file will be saved of the animation. This may take some time.

    Returns
    -------
    fig : matplotlib.Figure
        The figure.
    anim : matplotlib.FuncAnimation
        The animation.

    """

    # first set up the figure, the axis, and the plot elements we want to animate
    fig = plt.figure(figsize=(12, 12))
    
    # create the axes
    ax = plt.axes(xlim=(-0.2, .6), ylim=(-0.3, 1.0), aspect='equal')
    
    # display the current time
    time_text = ax.text(0.04, 0.9, '', transform=ax.transAxes)
    
    # blank line for the pendulum
    line_link_r, = ax.plot([], [], 'r', lw=3)
    line_link_l, = ax.plot([], [], 'b', lw=3)
    line_link_c, = ax.plot([], [], 'k--', lw=1.5)
    line_link_p, = ax.plot([], [], 'k', lw=3)
    
    line_link_flx, = ax.plot([], [], 'g', lw=1.5)
    line_link_fly, = ax.plot([], [], 'c', lw=1.5)
    line_link_frx, = ax.plot([], [], 'g', lw=1.5)
    line_link_fry, = ax.plot([], [], 'c', lw=1.5)
    
    line_link_tlh, = ax.plot([], [], 'm', lw=5)
    line_link_tlk, = ax.plot([], [], 'm', lw=5)
    line_link_trh, = ax.plot([], [], 'm', lw=5)
    line_link_trk, = ax.plot([], [], 'm', lw=5)

    # initialization function: plot the background of each frame
    def init():
        time_text.set_text('')
        
        line_link_l.set_data([], [])
        line_link_r.set_data([], [])
        line_link_c.set_data([], [])
        line_link_p.set_data([], [])
        
        line_link_flx.set_data([], [])
        line_link_fly.set_data([], [])
        line_link_frx.set_data([], [])
        line_link_fry.set_data([], [])
        
        line_link_tlh.set_data([], [])
        line_link_tlk.set_data([], [])
        line_link_trh.set_data([], [])
        line_link_trk.set_data([], [])
        
        return (time_text, line_link_r, line_link_l, line_link_c, line_link_p,
                line_link_flx, line_link_fly, line_link_frx, line_link_fry,
                line_link_tlh, line_link_tlk, line_link_trh, line_link_trk)

    # calcuate sticks based states
    def generate_sticks():
        
        sticks = np.zeros((len(states[:, 0]), 10))
        sticks[:, 0] = 0
        sticks[:, 1] = constants[0]
        sticks[:, 2] = np.sin(states[:, 0])*constants[1]
        sticks[:, 3] = constants[0] - np.cos(states[:, 0])*constants[1]
        sticks[:, 4] = np.sin(states[:, 0])*constants[1] + np.sin(states[:, 0]+states[:, 1])*constants[2]
        sticks[:, 5] = constants[0] - np.cos(states[:, 0])*constants[1] - np.cos(states[:, 0]+states[:, 1])*constants[2]
        sticks[:, 6] = np.sin(states[:, 2])*constants[1]
        sticks[:, 7] = constants[0] - np.cos(states[:, 2])*constants[1]
        sticks[:, 8] = np.sin(states[:, 2])*constants[1] + np.sin(states[:, 2]+states[:, 3])*constants[2]
        sticks[:, 9] = constants[0] - np.cos(states[:, 2])*constants[1] - np.cos(states[:, 2]+states[:, 3])*constants[2]

        return sticks
    
    def generate_cycling():
        
        cycle = np.zeros((100, 2))
        theta = np.linspace(0, 2*np.pi, 100)
        cycle[:, 0] = bike_param[1][0] + bike_param[2]*np.sin(theta)
        cycle[:, 1] = bike_param[1][1] + bike_param[2]*np.cos(theta)
        
        return cycle
    
    def generate_crank():
        
        crank = np.zeros((len(states[:, 0]), 4))
        
        crank[:, 0] = bike_param[1][0] + bike_param[2]*np.cos(states[:, 4])
        crank[:, 1] = bike_param[1][1] + bike_param[2]*np.sin(states[:, 4])
        
        crank[:, 2] = bike_param[1][0] + bike_param[2]*np.cos(states[:, 4]+np.pi)
        crank[:, 3] = bike_param[1][1] + bike_param[2]*np.sin(states[:, 4]+np.pi)
        
        return crank
    
    def generate_pedal_force():
        
        force_vec = np.zeros((len(states[:, 0]), 8))
        
        force_vec[:, 0] = bike_param[1][0] + bike_param[2]*np.cos(states[:, 4])
        force_vec[:, 1] = bike_param[1][1] + bike_param[2]*np.sin(states[:, 4])
        force_vec[:, 2] = bike_param[1][0] + bike_param[2]*np.cos(states[:, 4]) + pedal_forces[:, 0]*0.001
        force_vec[:, 3] = bike_param[1][1] + bike_param[2]*np.sin(states[:, 4]) + pedal_forces[:, 1]*0.001
        
        force_vec[:, 4] = bike_param[1][0] + bike_param[2]*np.cos(states[:, 4]+np.pi)
        force_vec[:, 5] = bike_param[1][1] + bike_param[2]*np.sin(states[:, 4]+np.pi)
        force_vec[:, 6] = bike_param[1][0] + bike_param[2]*np.cos(states[:, 4]+np.pi) + pedal_forces[:, 2]*0.001
        force_vec[:, 7] = bike_param[1][1] + bike_param[2]*np.sin(states[:, 4]+np.pi) + pedal_forces[:, 3]*0.001
        
        return force_vec
    
    def generate_torque():
        
        torque_vec_lh = np.zeros((len(states[:, 0]), 20))
        torque_vec_lk = np.zeros((len(states[:, 0]), 20))
        torque_vec_rh = np.zeros((len(states[:, 0]), 20))
        torque_vec_rk = np.zeros((len(states[:, 0]), 20))
        torque_q_lh = torques[:, 0]*0.1
        torque_q_lk = torques[:, 1]*0.1
        torque_q_rh = torques[:, 2]*0.1
        torque_q_rk = torques[:, 3]*0.1
        
        x_index = np.linspace(0, 18, 10, endpoint = True, dtype = np.int32)
        y_index = x_index + 1
        
        for n in range(10):  
            torque_vec_lh[:, 2*n] = np.sin(states[:, 0]+torque_q_lh*0.1*n)*constants[1]*0.2
            torque_vec_lh[:, 2*n+1] = constants[0] - np.cos(states[:, 0]+torque_q_lh*0.1*n)*constants[1]*0.2
            
            torque_vec_lk[:, 2*n] = np.sin(states[:, 0])*constants[1] + np.sin(states[:, 0]+states[:, 1]+torque_q_lk*0.1*n)*constants[2]*0.2
            torque_vec_lk[:, 2*n+1] = constants[0] - np.cos(states[:, 0])*constants[1] - np.cos(states[:, 0]+states[:, 1]+torque_q_lk*0.1*n)*constants[2]*0.2
            
            torque_vec_rh[:, 2*n] = np.sin(states[:, 2]+torque_q_rh*0.1*n)*constants[1]*0.3
            torque_vec_rh[:, 2*n+1] = constants[0] - np.cos(states[:, 2]+torque_q_rh*0.1*n)*constants[1]*0.3
            
            torque_vec_rk[:, 2*n] = np.sin(states[:, 2])*constants[1] + np.sin(states[:, 2]+states[:, 3]+torque_q_rk*0.1*n)*constants[2]*0.2
            torque_vec_rk[:, 2*n+1] = constants[0] - np.cos(states[:, 2])*constants[1] - np.cos(states[:, 2]+states[:, 3]+torque_q_rk*0.1*n)*constants[2]*0.2
        
        return torque_vec_lh, torque_vec_lk, torque_vec_rh, torque_vec_rk, x_index, y_index

    # animation function: update the objects
    def animate(i):
        time_text.set_text('time = {:2.2f}'.format(t[i]))
                       
        x_l = sticks[:, 2*np.array([0,1,2])]
        y_l = sticks[:, 2*np.array([0,1,2])+1]
        x_r = sticks[:, 2*np.array([0,3,4])]
        y_r = sticks[:, 2*np.array([0,3,4])+1]
        
        c_x = cycle[:, 0]
        c_y = cycle[:, 1]
        
        x_p = crank[:, [0, 2]]
        y_p = crank[:, [1, 3]]

        line_link_l.set_data(x_l[i, :], y_l[i, :])
        line_link_r.set_data(x_r[i, :], y_r[i, :])
        line_link_c.set_data(c_x, c_y)
        line_link_p.set_data(x_p[i, :], y_p[i, :])
        
        line_link_flx.set_data(force_vec[i, [0, 2]], force_vec[i, [1, 1]])
        line_link_fly.set_data(force_vec[i, [0, 0]], force_vec[i, [1, 3]])
        line_link_frx.set_data(force_vec[i, [4, 6]], force_vec[i, [5, 5]])
        line_link_fry.set_data(force_vec[i, [4, 4]], force_vec[i, [5, 7]])
        
        line_link_tlh.set_data(torque_vec_lh[i, x_index], torque_vec_lh[i, y_index])
        line_link_tlk.set_data(torque_vec_lk[i, x_index], torque_vec_lk[i, y_index])
        line_link_trh.set_data(torque_vec_rh[i, x_index], torque_vec_rh[i, y_index])
        line_link_trk.set_data(torque_vec_rk[i, x_index], torque_vec_rk[i, y_index])
        
        return (time_text, line_link_l, line_link_r, line_link_c, line_link_p,
                line_link_flx, line_link_fly, line_link_frx, line_link_fry,
                line_link_tlh, line_link_tlk, line_link_trh, line_link_trk)

    # call the animator function
    cycle = generate_cycling()
    crank = generate_crank()
    sticks = generate_sticks()
    force_vec = generate_pedal_force()
    torque_vec_lh, torque_vec_lk, torque_vec_rh, torque_vec_rk, x_index, y_index = generate_torque()
    anim = animation.FuncAnimation(fig, animate, frames=len(t), init_func=init,
            interval=t[-1] / len(t), blit=True, repeat=False)
    
    # save the animation if a filename is given
    if filename is not None:
        anim.save(filename, fps=fps, codec='libx264', bitrate=-1)
    
