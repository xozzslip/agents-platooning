from base import V, RV, Story, angle, point_vector_distance, index_of_closest_position, norm
import math
from collections import defaultdict
from structs import PlatoonStruct


class TrajectoryPlatoon:
    def __init__(self, master, minions, platoon_struct):
        """
        master имеет класс TrajectoryAgent
        minion имеет класс TargetAgent
        """
        self.master = master
        self.minions = minions
        self.ps = platoon_struct

        self.minions_errors = [[] for _ in range(len(self.minions))]
        self.targets = [[] for _ in range(len(self.minions))]
        assert 1 + len(minions) == len(platoon_struct)

        # Агенты изначально должны находиться в позициях target
        for i in range(len(self.minions)):
            target = self.calculate_target_for_minion(minion_order=i + 1)
            self.minions[i].target = target
            self.minions[i].position = target

    @property
    def current_orientation(self):
        return self.master.current_orientation

    @property
    def current_angle(self):
        return angle(self.ps.orientation, self.current_orientation)

    def calculate_target_for_minion(self, minion_order):
        relative_position = self.ps.relative_positions[minion_order]
        ps_agents = [self.master] + self.minions
        local_master = ps_agents[self.ps.relations[minion_order]]
        full_phi = self.current_angle + relative_position.phi
        sin = math.sin(full_phi)
        cos = math.cos(full_phi)
        target = local_master.position + \
                 norm(V(self.ps.orientation.x * cos  + self.ps.orientation.y * sin, 
                        -self.ps.orientation.x * sin  + self.ps.orientation.y * cos)) * relative_position.r
        return target

    def update(self):
        self.master.update()
        for i in range(len(self.minions)):
            self.minions_errors[i].append(abs(self.minions[i].target - self.minions[i].position))
            self.targets[i].append(self.minions[i].target)
            target = self.calculate_target_for_minion(minion_order=i + 1)
            self.minions[i].update_target(target)
            self.minions[i].update()


class FlexTrajectoryPlatoon(TrajectoryPlatoon):
    def __init__(self, agents, ps):
        """
        Агенты — flex
        """
        self.agents = self.enumerate_agents(agents)
        self.ps = ps
        self.sensetivity_r = 100

    def enumerate_agents(self, agents):
        for i in range(len(agents)):
            agents[i].id = i
        return agents
