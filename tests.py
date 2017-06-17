import unittest
import numpy as np
import matplotlib.pyplot as plt
import math
from base import RV, V, angle, to_degree, norm
from models import TargetAgent, TrajectoryAgent, FlexAgent
from structs import PlatoonStruct, PlatoonFullStruct
from platoons import TrajectoryPlatoon


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

    def test_initializing_full_ps(self):
        """
        0  o
            \
        1    o
              \
        2      o
        """
        points = [V(0, 0), V(1, -1), V(2, -2)]
        orientation = V(0, 1)
        ps = PlatoonFullStruct(points, orientation)

        self.assertEqual(len(ps.relative_positions), 3)
        self.assertEqual(len(ps.relative_positions[0]), 3)

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

        self.assertNotEqual(tp.master.story.positions[0], tp.master.story.positions[-1])

    def test_2(self):
        points = [V(0, 0), V(-1, -1)]
        orientation = V(0, 1)
        relations = [None, 0]
        ps = PlatoonStruct(points, orientation, relations)

        self.assertAlmostEqual(to_degree(ps.relative_positions[1].phi), -135, 2)


class TestFlexPlatoon(unittest.TestCase):
    def test_flex_agent_switch_to_master(self):
        trajectory = [V(math.cos(0.01 * x) * 100, math.sin(0.01 * x) * 100) for x in np.arange(0, 300, 10)]
        fa = FlexAgent(
            trajectory=trajectory,
            desired_velocity=5,
            position=V(100, 150)
        )
        fa.switch_to_master()

        for _ in range(400):
            fa.update()

        traj,  = plt.plot([p.x for p in trajectory], [p.y for p in trajectory], 'bo', alpha=0.2, label='Траектория')
        track, = plt.plot([p.x for p in fa.story.positions], [p.y for p in fa.story.positions], 'black', label='Мастер')
        plt.axis('equal')
        plt.show()

        # Расстояние от текущей позиции до конечной точки траейтории мало
        # self.assertLess(abs(fa.position - trajectory[-1]), 1)

       