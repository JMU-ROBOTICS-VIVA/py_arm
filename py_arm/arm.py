""" Simulated 2D multi-link arm.

Author: Nathan Sprague
"""

from itertools import product
import numpy as np
import shapely.geometry
import shapely.speedups
from . import geom_util

shapely.speedups.enable()

class Arm(object):
    """ Class representing a 2D multi-link arm. """

    def __init__(self, lengths, widths, limits=None, init_angles=None):
        """Create an arm object.

        Argments:
            lengths - Iterable sequence of link lengths.
            widths - Iterable sequence of link widths.
            limits - Iterable sequence of angle limits.  Each angle
                     limit should take the form (min_angle, max_angle)
                     with angles in the range -180.0 - 180.0.  Joints
                     without angle limits should have NaN values.
            init_angles - Initial angle values.  All will be 0 if not
                          specified.

        """
        self.lengths = np.array(lengths)
        self.widths = np.array(widths)
        if limits is not None:
            self.limits = np.array(limits)
        else:
            self.limits = None
        if init_angles is None:
            self.angles = np.zeros(self.lengths.shape)
        else:
            self.angles = np.array(init_angles)

        # Cache these points for use in the link forward kinematics
        self._link_points = []
        for i in range(self.num_links()):
            cur_points = np.ones((4, 3))
            # ll point
            cur_points[0, 0] = -self.widths[i]
            cur_points[0, 1] = -self.widths[i]
            # ul point
            cur_points[1, 0] = -self.widths[i]
            cur_points[1, 1] = self.widths[i]
            # ur point
            cur_points[2, 0] = self.widths[i] + self.lengths[i]
            cur_points[2, 1] = self.widths[i]
            # lr point
            cur_points[3, 0] = self.widths[i] + self.lengths[i]
            cur_points[3, 1] = -self.widths[i]
            self._link_points.append(cur_points)
        self._links = None

    def num_links(self):
        """ Total number of links """
        return self.lengths.size

    def length(self):
        """ Total length of all segments when fully extended. """
        return np.sum(self.lengths)

    def check_collision(self, obstacles):
        """Check for collisions between arm and obstacles.
        Args:
           obstacles - iterable collection of obstacles

        Returns: True if there is a collision
        """

        for link, obs in product(self.get_geometry(), obstacles):
            if link.intersects(obs):
                return True

        return False

    def angles_ok(self, angles):
        """ Check if all angles are within limits. """
        if self.limits is not None:
            angles = np.fmod(angles, 180.0)
            if (np.any(angles < self.limits[:, 0]) or
                    np.any(angles > self.limits[:, 1])):
                return False
        return True

    def self_collision(self):
        """Return true if the current arm configuration results in a
        self-collision.

        """
        links = self.get_geometry()
        for i in range(len(links)):
            # We don't check links against themselves or adjacent
            # links
            for j in range(i + 2, len(links), 1):
                if links[i].intersects(links[j]):
                    return True
        return False

    def set_angles(self, angles, respect_limits=True):
        """Set all joint angles.

        Args:
           angles - sequence containing angles

           respect_limits - True if joint limits should be checked
                            before angles are updated.

        Returns: True if the joint angles were updated successfully.

        """
        angles = np.array(angles)
        if not respect_limits or self.angles_ok(angles):
            self.angles = angles
            # Computing the link geometry is expensive, so we only update
            # it when it is actually needed.
            self._links = None
            return True
        else:
            return False

    def forward_kinematics(self, link, geom):
        """Transform the provided shapely geometry from the coordinate frame
        of the indicated link.

        Args:
           link - 0-indexed link number
           geom - Any transformable shapely geometry

        Returns:
           The geometry in the world coordinate frame.

        """
        cur_mat = np.eye(3)
        for i in range(link + 1):
            if i == 0:
                trans_mat = geom_util.trans_matrix(0, 0)
            else:
                trans_mat = geom_util.trans_matrix(self.lengths[i - 1], 0)
            rot_mat = geom_util.rot_matrix(self.angles[i])
            cur_mat = cur_mat.dot(trans_mat).dot(rot_mat)
        return geom_util.affine_transform(geom, cur_mat)

    def get_geometry(self):
        """Return a list containing all link polygons.

        """
        if self._links is None:
            self._compute_geometry()
        return self._links

    def _compute_geometry(self):
        """ Update the link geometry based on the current joint angles """
        self._links = []
        cur_mat = np.eye(3)
        for i in range(self.num_links()):
            if i == 0:
                trans_mat = geom_util.trans_matrix(0, 0)
            else:
                trans_mat = geom_util.trans_matrix(self.lengths[i - 1], 0)
            rot_mat = geom_util.rot_matrix(self.angles[i])
            cur_mat = cur_mat.dot(trans_mat).dot(rot_mat)
            trans_points = cur_mat.dot(self._link_points[i].T).T
            self._links.append(shapely.geometry.Polygon(trans_points[:, 0:2]))


def draw_arm():
    """Draw an arm using matplotlib.  Not fancy, just for sanity check.

    """
    import matplotlib.pyplot as plt

    arm = Arm([50, 40, 100], [10, 5, 2])
    arm.set_angles([10, 90, 90])

    links = arm.get_geometry()
    for link in links:
        xs, ys = link.exterior.xy
        plt.fill(xs, ys, edgecolor=(0, 0, 0), facecolor=(.7, .7, .7))
    plt.axis('scaled')
    plt.show()


if __name__ == "__main__":
    draw_arm()
