""" Utility code for drawing arms and arm configuration spaces.

Author: Nathan Sprague

"""

import shapely
import shapely.geometry
from shapely.geometry import Point, MultiPoint, MultiPolygon
import shapely.affinity
import shapely.ops

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches
import matplotlib.transforms

from . import geom_util

def draw_arm(arm, ax=None, alpha=1.0):
    """Draw the arm onto the provided axes with the given alpha.
    Arguments:
       arm - Arm object
       ax - Matplotlib axis or None if a new axis should be created
       alpha - transparency value

    Returns: A list of tuples where the first entry in each tuple is a
             matplotlib patch object representing the link and the
             second is a handle representing the dot at the joint.
             These are needed if we want to change the arm
             visualization without re-drawing the entire figure.

    """
    if not ax:
        ax = plt.gca()
    links = arm.get_geometry()
    handles = []
    for i, link in enumerate(links):
        xs, ys = link.exterior.xy
        patch, = ax.fill(xs, ys, edgecolor=(0, 0, 0),
                         facecolor=(.7, .7, .7), alpha=alpha)
        pivot = arm.forward_kinematics(i, Point(0, 0))
        dot, = ax.plot(pivot.x, pivot.y, '.k', alpha=alpha)
        handles.append((patch, dot))
    return handles


def torus_to_cartesian(theta1, theta2, r=.33, R=1.):
    """ Convert from coordinates on a torus to cartesian coordinates suitable
    for 3d plotting.

    Args:
       theta1, theta2 - arrays of torus coordinates (created using np.meshgrid)
       r - Radius of the circle that we spin to create the torus.
       R - Radius of the overall torus

    Returns:
      X, Y, Z coordinates

    """
    X = (R + r * np.cos(theta2 * np.pi / 180)) * np.cos(theta1 * np.pi / 180)
    Y = (R + r * np.cos(theta2 * np.pi / 180)) * np.sin(theta1 * np.pi / 180)
    Z = r * np.sin(theta2 * np.pi / 180)
    return X, Y, Z


def draw_c_space_torus(ax, arm, obstacles, obstacle_colors):
    """ Draw a torus painted with the collision regions.

    """

    theta1 = np.linspace(-180.0, 180.0, 200)
    theta2 = np.linspace(-180.0, 180.0, 100)
    colors = np.ones((theta1.size, theta2.size, 3)) * .9  # light gray
    # background.
    for i, t1 in enumerate(theta1):
        for j, t2 in enumerate(theta2):
            arm.set_angles((t1, t2))
            for obs, color in zip(obstacles, obstacle_colors):
                if arm.check_collision([obs]):
                    colors[i, j, :] = color
            if ((t1 < -179 or t1 > 179) or
                    (t2 < -178 or t2 > 178)):  # Draw black lines at
                # theta1=0 and theta2=0.
                colors[i, j, :] = np.array([0, 0, 0])
    theta2, theta1 = np.meshgrid(theta2, theta1)

    X, Y, Z = torus_to_cartesian(theta1, theta2)

    ax.set_xlim3d(-1, 1)
    ax.set_ylim3d(-1, 1)
    ax.set_zlim3d(-1, 1)
    ax.plot_surface(X, Y, Z, facecolors=colors, rstride=1, cstride=1,
                    linewidth=.5)

    ax.view_init(35, -225)
    ax.axis('off')


def collision_thetas(arm, obstacle):
    """Check a grid of (theta1, theta2) values and return a sequence
    contining all points that result in a collision with the provided
    obstacle.

    """
    points = []
    for theta1 in np.linspace(-180., 180.0, 100):
        for theta2 in np.linspace(-180.0, 180.0, 100):
            arm.set_angles((theta1, theta2), respect_limits=False)
            if arm.check_collision([obstacle]):
                points.append(Point(theta1, theta2))
    return MultiPoint(points)


def find_polygons(points):
    """This function takes a dense collection of points from the inside of
    some number of shapes and creates a collection of polygons
    containing those points.  It works by performing a Delauny
    triangulation, throwing out the big triangles, and merging the
    remaining small triangles.

    """
    shapes = shapely.ops.triangulate(points)
    med = np.median([shape.area for shape in shapes])
    small_shapes = [shape for shape in shapes if shape.area <= med * 1.05]
    result = shapely.ops.unary_union(MultiPolygon(small_shapes))
    if not hasattr(result, '__iter__'):
        result = MultiPolygon([result])
    return result


def draw_c_space(arm, ax, obstacles, obstacle_colors):
    """Draw the configuration space for the two-degree of freedom arm
        showing C_obs.

    """
    for i, obstacle in enumerate(obstacles):
        coll_points = collision_thetas(arm, obstacle)
        shapes = find_polygons(coll_points)
        for shape in shapes.geoms:
            xs, ys = shape.exterior.xy
            ax.fill(xs, ys, facecolor=obstacle_colors[i])

    ax.set_xlabel(r'$\Theta_1$')
    ax.set_ylabel(r'$\Theta_2$')
    ax.set_xticks([-180, -90, 0, 90, 180])
    ax.set_yticks([-180, -90, 0, 90, 180])
    ax.axis('scaled')
    ax.set_xlim(-180, 180)
    ax.set_ylim(-180, 180)


def draw_world(arm, ax, theta1, theta2, obstacles, obstacle_colors,
               show_obstacles=True, show_angles=False):
    """
    Returns a tuple with the three handles necessary to update the graph
    dynamically.

    """
    if show_obstacles:
        for i, obstacle in enumerate(obstacles):
            xs, ys = obstacle.exterior.xy
            ax.fill(xs, ys, edgecolor=(0, 0, 0),
                    facecolor=obstacle_colors[i])

    if isinstance(theta1, np.ndarray):
        for i in range(len(theta1)):
            alpha = float(i) / len(theta1) * .7 + .1
            arm.set_angles((theta1[i], theta2[i]))
            draw_arm(arm, ax, alpha=alpha)
        arm.set_angles((theta1[-1], theta2[-1]))
        handles = draw_arm(arm, ax, alpha=1)

    else:
        arm.set_angles((theta1, theta2))
        handles = draw_arm(arm, ax)

    patch1 = handles[0][0]
    patch2 = handles[1][0]

    hinge_dot = handles[1][1]

    if show_angles:
        # theta1
        arc1 = matplotlib.patches.Arc((0, 0), arm.lengths[0] * .6,
                                      arm.lengths[0] * .6, theta1=0,
                                      theta2=theta1, linewidth=.5)
        ax.add_patch(arc1)

        ax.arrow(0, 0, arm.lengths[0] * .5, 0, linewidth=.5,
                 head_width=5, facecolor='k')
        ax.arrow(0, 0, 0, arm.lengths[0] * .5, linewidth=.5,
                 head_width=5, facecolor='k')
        arc_line_end = arm.forward_kinematics(0, Point(.5 *
                                                       arm.lengths[0],
                                                       0))
        ax.plot([0, arc_line_end.x], [0, arc_line_end.y],
                '--k', linewidth=.5)

        # theta2
        arc2 = matplotlib.patches.Arc((0, 0), arm.lengths[1] * .6,
                                      arm.lengths[1] * .6, theta1=0,
                                      theta2=theta2, linewidth=.5)

        t_start = ax.transData
        transform = matplotlib.transforms.Affine2D().translate(arm.lengths[0],
                                                               0)
        transform.rotate_deg(theta1)
        t_end = transform + t_start
        arc2.set_transform(t_end)
        ax.add_patch(arc2)
        arrow1 = ax.arrow(0, 0, arm.lengths[0] * .5, 0, linewidth=.5,
                          head_width=5, facecolor='k')
        arrow2 = ax.arrow(0, 0, 0, arm.lengths[0] * .5, linewidth=.5,
                          head_width=5, facecolor='k')
        arrow1.set_transform(t_end)
        arrow2.set_transform(t_end)
        ax.text(41, 14, r"$\Theta_1$")
        ax.text(70, 117, r"$\Theta_2$")

    # Remove ticks
    ax.xaxis.set_major_locator(plt.NullLocator())
    ax.yaxis.set_major_locator(plt.NullLocator())

    ax.axis('scaled')
    if not show_obstacles:
        xlim = ax.get_xlim()
        ax.set_xlim(xlim[0] * 1.05, xlim[1] * 1.05)  # hack
        ax.axis('off')

    return patch1, patch2, hinge_dot

class PlanAnimator(object):

    def __init__(self, arm, obstacles, interval=100, max_delta=1.0):
        self.arm = arm
        self.obstacles = obstacles
        self.interval = interval
        self.max_delta = max_delta

    def _update_animation(self, plan, handles):
        if 0 <= self._frame_number < len(plan):
            self.arm.set_angles(plan[self._frame_number], False)
            links = self.arm.get_geometry()
            for i, link in enumerate(links):
                xs, ys = link.exterior.xy
                handles[i][0].set_xy(np.append([xs], [ys], axis=0).T)
                p_zero_link = self.arm.forward_kinematics(i, Point(0, 0))
                handles[i][1].set_data([p_zero_link.x], [p_zero_link.y])
        elif self._frame_number == len(plan):
            self._frame_number = -10  # create a pause at the end.
        self._frame_number += 1
        plt.draw()

    def animate_plan(self, start, goal, plan):
        fig = plt.figure()
        ax = fig.gca()

        steps = []
        for i in range(len(plan) - 1):
            steps.extend(list(geom_util.angle_sequence(plan[i], plan[i + 1],
                                                       self.max_delta)))

        for obs in self.obstacles:
            xs, ys = obs.exterior.xy
            ax.fill(xs, ys, edgecolor=(0, 0, 0),
                    facecolor=(.5, .5, 1.0))

        self.arm.set_angles(start, False)
        draw_arm(self.arm, ax, alpha=.2)
        self.arm.set_angles(goal, False)
        draw_arm(self.arm, ax, alpha=.2)

        self.arm.set_angles(plan[0], False)
        handles = draw_arm(self.arm, ax)
        timer = fig.canvas.new_timer(interval=self.interval)
        timer.add_callback(self._update_animation, steps, handles)
        self._frame_number = 1
        timer.start()
        ax.axis('scaled')
        ax.set_xlim(-self.arm.length(), self.arm.length())
        ax.set_ylim(-self.arm.length(), self.arm.length())
        ax.xaxis.set_major_locator(plt.NullLocator())
        ax.yaxis.set_major_locator(plt.NullLocator())

        plt.show()
