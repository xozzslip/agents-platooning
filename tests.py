import unittest
import numpy as np
import matplotlib.pyplot as plt
import math
from base import RV, V, angle, to_degree, norm
from models import TargetAgent, TrajectoryAgent, FlexAgent
from structs import PlatoonStruct, PlatoonFullStruct
from platoons import TrajectoryPlatoon, FlexTrajectoryPlatoon


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
        print(ps.relative_positions)

        self.assertEqual(len(ps.relative_positions), 3)
        self.assertEqual(len(ps.relative_positions[0]), 3)

    def test_relative_positions_correctness(self):
        points = [V(0, 0), V(1, -1)]
        orientation = V(0, 1)
        relations = [None, 0]
        ps = PlatoonStruct(points, orientation, relations)

        relative_position = ps.relative_positions[1]
        sin = math.sin(relative_position.phi)
        cos = math.cos(relative_position.phi)


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
            desired_velocity=12,
            position=V(100, 150)
        )
        fa.switch_to_master()

        for _ in range(350):
            fa.update()

        # traj,  = plt.plot([p.x for p in trajectory], [p.y for p in trajectory], 'bo', alpha=0.2, label='Траектория')
        # track, = plt.plot([p.x for p in fa.story.positions], [p.y for p in fa.story.positions], 'black', label='Мастер')
        # plt.axis('equal')
        # plt.show()

        # Расстояние от текущей позиции до конечной точки траейтории мало
        self.assertLess(abs(fa.position - trajectory[-1]), 10)

    def test_flex_agent_switch_to_minion(self):
        trajectory = [V(0, 0), V(1, 1)]
        fa = FlexAgent(
            trajectory=trajectory,
            desired_velocity=12,
            position=V(10, 10)
        )
        target = V(30, 30)
        fa.switch_to_minion(target=target)

        for i in range(150):
            fa.update_target(target=target)
            fa.update()
       
        self.assertLess(abs(fa.position - target), 1)

    def test_flex_agent_twice_switched(self):
        trajectory = [V(math.cos(0.01 * x) * 100, math.sin(0.01 * x) * 100) for x in np.arange(0, 300, 10)]
        fa = FlexAgent(trajectory=trajectory, desired_velocity=12, position=V(100, 150))
        
        fa.switch_to_master()
        for _ in range(200):
            fa.update()

        target = V(30, 30)
        fa.switch_to_minion(target)
        for i in range(150):
            fa.update_target(target=target)
            fa.update()

        self.assertLess(abs(fa.position - target), 5)


class TestFlexTrajPlatoon(unittest.TestCase):
    def setUp(self):
        """
        Схема расположения агентов (ai) и точек структуры (pj)

               a2     a0

           p0
             \
             p1
               \
               p2

        a1
        """
        points = [V(10, 10), V(22, 0), V(-22, -2)]
        orientation = V(0, 1)
        ps = PlatoonFullStruct(points, orientation)

        # Траектория — полукруг
        self.tr = [V(math.cos(0.01 * x) * 100, math.sin(0.01 * x) * 100) for x in np.arange(0, 300, 10)]
        des_v = 5
        agents_positions = [V(-15, 3), V(-10, -10), V(1, 3)]
        agents = [FlexAgent(self.tr, des_v, p) for p in agents_positions]

        self.ftp = FlexTrajectoryPlatoon(agents, ps)

    def test_enumeration(self):
        self.ftp.reenumerate_agents()
        self.assertEqual(self.ftp.agents[0].id, 1)
        self.assertEqual(self.ftp.agents[1].id, 2)
        self.assertEqual(self.ftp.agents[2].id, 0)

    def test_influence_list(self):
        self.ftp.reenumerate_agents()
        infl_l = self.ftp.influence_list(self.ftp.agents[0])
        self.assertIs(infl_l[0], self.ftp.agents[2])
        self.assertEqual(len(infl_l), 1)

    def test_calculate_target_for_minion(self):
        self.ftp.reenumerate_agents()
        self.ftp.sorted_agents = sorted(self.ftp.agents, key=lambda x: x.id)
        self.ftp.master.switch_to_master()
        t = self.ftp.calculate_target_for_minion(self.ftp.agents[0])
        # Нулевой агент попадает в p1, следовательно должен быть
        # правее и ниже, чем a2, который является его единственным
        # отношением

    def test_switch(self):
        self.ftp.switch()

        agents_track = [[] for _ in range(len(self.ftp.agents))]
        for _ in range(500):
            self.ftp.update()
            for i in range(len(self.ftp.agents)):
                agents_track[i].append(self.ftp.agents[i].position)


        traj,  = plt.plot([p.x for p in self.tr], [p.y for p in self.tr], 'bo', alpha=0.2, label='Траектория')
        track, = plt.plot([p.x for p in agents_track[0]], [p.y for p in agents_track[0]], 'black', label='Мастер')
        track, = plt.plot([p.x for p in agents_track[1]], [p.y for p in agents_track[1]], 'black', label='Мастер')
        track, = plt.plot([p.x for p in agents_track[2]], [p.y for p in agents_track[2]], 'black', label='Мастер')
        plt.axis('equal')
        plt.show()

    def test_split_groups(self):
        ag1 = [V(700, 800), V(0, 0), V(300, 300), V(305, 305), V(1, 1), V(2, 2)]
        agents = [FlexAgent(self.tr, 5, p) for p in ag1]
        self.ftp.agents = agents
        groups = self.ftp.split_to_groups()
        self.assertEqual(len(groups[0]), 1)
        self.assertEqual(len(groups[1]), 3)
        self.assertEqual(len(groups[2]), 2)
