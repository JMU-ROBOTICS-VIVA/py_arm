import unittest
import shapely
import shapely.geometry
from arm import Arm

class TestArm(unittest.TestCase):

    def setUp(self):
        self.arm = Arm([50, 40, 100], [10, 5, 2])
        
    def test_angle_bounds(self):
        arm =  Arm([50, 40, 100], [10, 5, 2], [[-45., 45.],
                                               [-90., 90.],
                                               [-180., 180.]])
        self.assertTrue(arm.angles_ok([-45., -90., -180]))
        self.assertTrue(arm.angles_ok([45., 90., 180]))
        self.assertTrue(arm.angles_ok([0., 0., 0.]))
        self.assertTrue(arm.angles_ok([-225., -270., 7898.]))
        self.assertTrue(arm.angles_ok([225., 270., 7898.]))
        
        self.assertFalse(arm.angles_ok([-45.1, -90.1, -180]))
        self.assertFalse(arm.angles_ok([45.1, 90.1, 180]))
        self.assertFalse(arm.angles_ok([-225.1, -270.1, 7898.]))
        self.assertFalse(arm.angles_ok([225.1, 270.1, 7898.]))
    
        
    def test_self_collision(self):
        self.arm.set_angles([10., 90., 90.])
        self.assertFalse(self.arm.self_collision())
        self.arm.set_angles([10., 90., 180.])
        self.assertTrue(self.arm.self_collision())

    def test_object_collision(self):
        obstacles = [shapely.geometry.Point(180, 0)]
        self.arm.set_angles([0., 0., 0.])
        self.assertTrue(self.arm.check_collision(obstacles))
        self.arm.set_angles([90., 0., 0.])
        self.assertFalse(self.arm.check_collision(obstacles))



if __name__ == '__main__':
    unittest.main()
