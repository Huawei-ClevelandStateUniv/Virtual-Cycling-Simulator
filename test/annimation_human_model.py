#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 30 13:34:36 2017

@author: huawei
"""
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

def animate_human_model(t, states, constants, filename=None):
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
    ax = plt.axes(xlim=(-1.0, 1.0), ylim=(0.0, 2.0), aspect='equal')
    
    # display the current time
    time_text = ax.text(0.04, 0.9, '', transform=ax.transAxes)
    
    # blank line for the pendulum
    line_link_r, = ax.plot([], [], 'r', lw=3)
    line_link_l, = ax.plot([], [], 'b', lw=3)

    # initialization function: plot the background of each frame
    def init():
        time_text.set_text('')
        
        line_link_r.set_data([], [])
        line_link_l.set_data([], [])
        return (time_text, line_link_r, line_link_l)

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

    # animation function: update the objects
    def animate(i):
        time_text.set_text('time = {:2.2f}'.format(t[i]))
                       
        x_l = sticks[:, 2*np.array([0,1,2])]
        y_l = sticks[:, 2*np.array([0,1,2])+1]
        x_r = sticks[:, 2*np.array([0,3,4])]
        y_r = sticks[:, 2*np.array([0,3,4])+1]

        line_link_l.set_data(x_l[i, :], y_l[i, :])
        line_link_r.set_data(x_r[i, :], y_r[i, :])
        
        return (time_text, line_link_l, line_link_r)

    # call the animator function
    sticks = generate_sticks()
    anim = animation.FuncAnimation(fig, animate, frames=len(t), init_func=init,
            interval=t[-1] / len(t), blit=True, repeat=False)
    
    # save the animation if a filename is given
    if filename is not None:
        anim.save(filename, fps=25, codec='libx264', bitrate=-1)
    
