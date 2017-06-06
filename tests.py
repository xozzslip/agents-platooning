import unittest
import numpy as np
import matplotlib.pyplot as plt
import math
from base import RV, V, angle, to_degree, norm
from models import PlatoonStruct, TargetAgent, TrajectoryAgent, TrajectoryPlatoon


class TestBase(unittest.TestCase):
    def test_angle(self):
        angle_1 = angle(V(0, 1), V(1, 0))
        self.assertAlmostEqual(angle_1, 1.57, 2)


class TestPlatoonStruct(unittest.TestCase):
    def test_initializing(self):
        points = [V(0, 0), V(-1, -1), V(1, 1), V(-2, -2), V(2, 2)]
        orientation = V(0, 1)
        relations = [None, 0, 0, 1, 2]

        ps = PlatoonStruct(points, orientation, relations)
        self.assertEqual(ps.relative_positions[0], None)
        self.assertAlmostEqual(ps.relative_positions[1].phi, -2.36, 2)

    def test_relative_positions_correctness(self):
        points = [V(0, 0), V(-1, -1)]
        orientation = V(0, 1)
        relations = [None, 0]
        ps = PlatoonStruct(points, orientation, relations)

        relative_position = ps.relative_positions[1]
        # print(relative_position)
        sin = math.sin(relative_position.phi)
        cos = math.cos(relative_position.phi)
        # print(points[0] + V(ps.orientation.x * cos  - ps.orientation.y * sin, 
        #                   ps.orientation.x * sin  + ps.orientation.y * cos) * relative_position.r)


class TestTrajectoryPlatoon(unittest.TestCase):
    def test_1(self):
        points = [V(0, 0), V(-0.1, -0.1)]
        orientation = V(0, 1)
        relations = [None, 0]
        ps = PlatoonStruct(points, orientation, relations)
        trajectory = [V(math.cos(x) ** 3, math.sin(x) ** 3) for x in np.arange(0, 30, 0.1)]

        master = TrajectoryAgent(trajectory)
        minions = [TargetAgent(V(-1, -1))]
        tp = TrajectoryPlatoon(master=master, minions=minions, platoon_struct=ps)
        for _ in range(30):
            tp.master.update()
    
        self.assertLess(to_degree(tp.current_angle), -75)
        self.assertNotEqual(tp.master.story.positions[0], tp.master.story.positions[-1])

    def test_2(self):
        points = [V(0, 0), V(-1, -1)]
        orientation = V(0, 1)
        relations = [None, 0]
        ps = PlatoonStruct(points, orientation, relations)

        self.assertAlmostEqual(to_degree(ps.relative_positions[1].phi), -135, 2)







