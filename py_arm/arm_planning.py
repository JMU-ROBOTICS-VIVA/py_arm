"""

"""

from shapely.geometry import Point
import numpy as np
from . import arm
from . import geom_util


class Problem(object):

    def start(self):
        raise NotImplementedError()

    def goal(self):
        raise NotImplementedError()

    def config_ok(self, q):
        raise NotImplementedError()

    def step_ok(self, q1, q2):
        raise NotImplementedError()

    def plan_ok(self, plan):
        raise NotImplementedError()


class ArmProblem(Problem):
    ARM_LENGTH = 100.0
    ARM_WIDTH = 3.0
    ANGLE_LIMITS = 270.0
    MAX_ANGLE_DELTA = 6.0

    def __init__(self, start_config, goal_config, goal_tolerance=1.0,
                 obstacles=[]):
        self.start_config = np.array(start_config)
        self.goal_config = np.array(goal_config)
        num_links = self.start_config.size
        self.obstacles = obstacles
        self.goal_tolerance = goal_tolerance

        lengths = [self.ARM_LENGTH / num_links for _ in range(num_links)]
        widths = [self.ARM_WIDTH for _ in range(num_links)]
        angle_limits = [[-self.ANGLE_LIMITS / 2.0,
                         self.ANGLE_LIMITS / 2.0] for _ in range(num_links)]
        angle_limits[0][0] = float('nan')
        angle_limits[0][1] = float('nan')

        self.arm = arm.Arm(lengths, widths, angle_limits)

    def start(self):
        return self.start_config

    def goal(self):
        return self.goal_config

    def at_goal(self, q):
        return (np.max(np.abs(geom_util.angle_diff(q, self.goal_config))) <
                self.goal_tolerance)

    def config_ok(self, q):
        if not self.arm.set_angles(q):
            return False
        if self.arm.self_collision():
            return False
        if self.arm.check_collision(self.obstacles):
            return False
        else:
            return True

    def step_ok(self, q1, q2):
        steps = geom_util.angle_sequence(q1, q2, self.MAX_ANGLE_DELTA)
        for step in range(steps.shape[0] - 1, -1, -1):
            q = steps[step]
            if not self.config_ok(q):
                return False

        return True

    def plan_ok(self, plan):
        if len(plan) == 0:
            print("Zero length plan!")
            return False
        if not np.allclose(plan[0], self.start_config):
            print("Plan does not start at the start configuration!")
            return False
        for i in range(len(plan) - 1):
            if not self.step_ok(plan[i], plan[i + 1]):
                print("Plan includes an invalid step!", plan[i], plan[i + 1])
                return False
        if not self.at_goal(plan[-1]):
            print("Plan does not terminate at the goal state!")
            return False
        return True


def plan_demo():
    from . import arm_plotting
    obs = Point(150, 150).buffer(10)
    p = ArmProblem([0, 0, 0], [45, 45, 45],
                   obstacles=[obs])

    plan = np.zeros((100, 3))
    plan[:, 0] = np.linspace(0, 45, 100)
    plan[:, 1] = np.linspace(0, 45, 100)
    plan[:, 2] = np.linspace(0, 45, 100)

    print(p.plan_ok(plan))
    plan_animator = arm_plotting.PlanAnimator(p.arm, p.obstacles, 10)
    plan_animator.animate_plan(p.start(), p.goal(), plan)


if __name__ == "__main__":
    plan_demo()
