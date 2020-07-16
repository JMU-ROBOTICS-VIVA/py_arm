#!/usr/bin/env python
"""Arm animation script showing the correspondence between C_space
and the robot for a two-link arm.

Author: Nathan Sprague

"""


import shapely
from shapely.geometry import Point

import numpy as np
import matplotlib.pyplot as plt

from py_arm.arm import Arm
from py_arm.arm_plotting import draw_c_space, draw_world

# Set arm characteristics and obstacle information.
L1 = 100.0
L2 = 100.0
LINK_WIDTH = 6
OBSTACLES = []
OBSTACLES.append(shapely.geometry.box(-150, 60, -90, 120))
OBSTACLES.append(shapely.geometry.Point(90, -40).buffer(40)) # circle
OBSTACLE_COLORS = [(1., .5, .5), (.5, .5, 1), (.5, 1, .5)]

MOUSE_DOWN = False

def animation():
    """ Show an arm animation. """

    def on_press(event):
        global MOUSE_DOWN
        MOUSE_DOWN = True

    def on_release(event):
        global MOUSE_DOWN
        MOUSE_DOWN = False

    def on_motion(event):
        if MOUSE_DOWN and event.xdata is not None:
            #print("[{:.2f}, {:.2f}],".format(event.xdata, event.ydata))
            arm.set_angles((event.xdata, event.ydata))
            links = list(arm.get_geometry())
            xs, ys = links[0].exterior.xy
            patch1.set_xy(np.append([xs], [ys], axis=0).T)
            xs, ys = links[1].exterior.xy
            patch2.set_xy(np.append([xs], [ys], axis=0).T)
            p_zero_seg2 = arm.forward_kinematics(1, Point(0, 0))
            hinge_dot.set_data([p_zero_seg2.x], [p_zero_seg2.y])

            q_dot.set_data([event.xdata], [event.ydata])

            plt.draw()

    fig = plt.figure()

    theta1 = 45
    theta2 = 90
    arm = Arm((L1, L2), (LINK_WIDTH, LINK_WIDTH))

    # Draw c_obs
    ax1 = fig.add_subplot(2, 1, 1)
    draw_c_space(arm, ax1, OBSTACLES, OBSTACLE_COLORS)
    q_dot, = ax1.plot([theta1], [theta2], 'o')

    # Draw W
    ax2 = fig.add_subplot(2, 1, 2)
    patch1, patch2, hinge_dot = draw_world(arm, ax2, theta1, theta2,
                                           OBSTACLES, OBSTACLE_COLORS)
    lim = (L1 + L2) * 1.1
    ax2.set_xlim(-lim, lim)
    ax2.set_ylim(-lim, lim)

    # Set up the callbacks.
    fig.canvas.mpl_connect('button_press_event', on_press)
    fig.canvas.mpl_connect('button_release_event', on_release)
    fig.canvas.mpl_connect('motion_notify_event', on_motion)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    animation()

