from base import (V, RV, Story, angle, point_vector_distance, index_of_closest_position, 
    norm, abstract_index_of_closest_position, to_degree, centrize_plist)
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
        self.agents = agents
        self.sorted_agents = agents
        self.ps = ps

    def reenumerate_agents(self):
        """
        Каждая точка структуры выдаётся какому-то агенту.
        Структура должна быть заполнена в том порядке, в каком
        желательно её заполнение.

        Если точек больше, чем количесво агентов в данный момент, то
        то будут заняты не все точки.
        Если точек меньше, чем количество агентов, то не у каждого
        агента будет ID.
        """
        for agent in self.agents:
            agent.id = None
        for i in range(min(len(self.ps.points), len(self.agents))):
            point = self.ps.points[i]
            # К какому агенту данная точка ближе всего?
            free_agents = [agent for agent in self.agents if agent.id is None]
            free_agents_positions = centrize_plist([agent.position for agent in free_agents])
            ic = abstract_index_of_closest_position(point, free_agents_positions)
            # ID этого агента будет номер точки в списке ps.points
            free_agents[ic].id = i

    def switch(self):
        self.reenumerate_agents()
        self.sorted_agents = sorted(self.agents, key=lambda x: x.id)
        self.master.switch_to_master()
        for minion in self.minions:
            target = self.calculate_target_for_minion(minion)
            minion.switch_to_minion(target)

    def influence_list(self, agent):
        other_agents = []
        for a in self.agents:
            if (abs(a.position - agent.position) < agent.sensetivity_r
                and a is not agent and agent.id > a.id):
                other_agents.append(a)
        return other_agents     

    def calculate_target_for_minion(self, agent):
        # Если агент находится в списке sorted_agents 
        # на i-ом месте, то он должен находится в точке
        # ps.points[i]. Target будет отсчитан относительно
        # всех агентов в поле sensetivity_r.
        other_agents = self.influence_list(agent)
        relations = self.ps.relative_positions[agent.id]
        targets = []
        for infl_a in other_agents:
            rel = relations[infl_a.id]
            full_phi = self.current_angle + rel.phi
            sin = math.sin(full_phi)
            cos = math.cos(full_phi)
            target = infl_a.position + \
                     norm(V(self.ps.orientation.x * cos  + self.ps.orientation.y * sin, 
                            -self.ps.orientation.x * sin  + self.ps.orientation.y * cos)) * rel.r
            targets.append(target)
        return sum(targets, V(0, 0)) / len(targets)

    @property
    def master(self):
        return self.sorted_agents[0]

    @property
    def minions(self):
        return self.sorted_agents[1:]

    @property
    def current_orientation(self):
        return self.master.trajectory_agent.current_orientation

    @property
    def current_angle(self):
        return angle(self.ps.orientation, self.current_orientation)

    def update(self):
        self.master.update()
        for i in range(len(self.minions)):
            minion = self.minions[i]
            target = self.calculate_target_for_minion(minion)
            minion.update_target(target)
            minion.update()
