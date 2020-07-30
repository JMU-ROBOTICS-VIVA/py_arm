""" Live-updating arm simulator.

Author: Nathan Sprague

"""

import multiprocessing
from shapely.geometry import Point
import numpy as np
import matplotlib.pyplot as plt

from . import arm_plotting

LOCK = multiprocessing.Lock()


def simulation_main(q):
    arm = q.get()
    obstacles = q.get()
    interval = q.get()
    ArmAnimator(arm, obstacles, interval, q)


class ArmAnimator(object):

    def __init__(self, arm, obstacles, interval, queue):
        self.arm = arm
        self.obstacles = obstacles
        self.interval = interval
        self.queue = queue

        fig = plt.figure()
        ax = fig.gca()

        for obs in self.obstacles:
            xs, ys = obs.exterior.xy
            ax.fill(xs, ys, edgecolor=(0, 0, 0),
                    facecolor=(.5, .5, 1.0))

        self.handles = arm_plotting.draw_arm(self.arm, ax)
        timer = fig.canvas.new_timer(interval=self.interval)
        timer.add_callback(self._update_animation)
        timer.start()
        ax.axis('scaled')
        ax.set_xlim(-self.arm.length(), self.arm.length())
        ax.set_ylim(-self.arm.length(), self.arm.length())
        ax.xaxis.set_major_locator(plt.NullLocator())
        ax.yaxis.set_major_locator(plt.NullLocator())

        plt.show()

    def _update_animation(self):
        angles = None
        LOCK.acquire()
        while not self.queue.empty():
            angles = self.queue.get()
        LOCK.release()

        if angles is not None:
            self.arm.set_angles(self.queue.get(), False)
            links = self.arm.get_geometry()
            for i, link in enumerate(links):
                xs, ys = link.exterior.xy
                self.handles[i][0].set_xy(np.append([xs], [ys], axis=0).T)
                p_zero_link = self.arm.forward_kinematics(i, Point(0, 0))
                self.handles[i][1].set_data([p_zero_link.x], [p_zero_link.y])
        plt.draw()


class ArmSim(object):

    def __init__(self, arm, objects):
        self.arm = arm
        self.objects = objects
        self.queue = multiprocessing.Queue()
        self.queue.put(arm)
        self.queue.put(objects)
        self.queue.put(33)

        p = multiprocessing.Process(target=simulation_main, args=(self.queue,))
        p.start()

    def set_angles(self, angles):
        ok = self.arm.set_angles(angles)
        LOCK.acquire()
        self.queue.put(self.arm.angles)
        LOCK.release()
        return ok
