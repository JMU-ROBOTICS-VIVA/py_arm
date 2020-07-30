import time
import py_arm.arm
import py_arm.arm_sim
import shapely.geometry

if __name__ == "__main__":

    arm = py_arm.arm.Arm((100, 100), (5, 5))

    obstacles = [shapely.geometry.box(-150, 60, -90, 120),
                 shapely.geometry.Point(90, -40).buffer(40)]

    sim = py_arm.arm_sim.ArmSim(arm, obstacles)

    first = 0
    second = 0
    while True:
        first += 1
        second += 1
        sim.set_angles([first, second])
        time.sleep(.05)
