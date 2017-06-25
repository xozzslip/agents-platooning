from base import (V, RV, Story, angle, point_vector_distance, index_of_closest_position, 
    norm, abstract_index_of_closest_position, to_degree, centrize_plist, rotate_struct)
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
        self.agents = self.enumerate_angents(agents)
        self.ps = ps
        sort_agents = self.initsort_agents()
        self.groups = [[sort_agents[i]] for i in range(len(self.agents))]
        self.groups.sort(key=lambda x: min([abs(agent.position - agent.trajectory[0]) for agent in x]))
        

    def enumerate_angents(self, agents):
        for i in range(len(agents)):
            agents[i].id = 0
        return agents

    def initsort_agents(self):
        result = [None for _ in range(len(self.agents))]
        points = self.ps.points.copy()
        for i in range(len(self.agents)):
            agent = self.agents[i]
            free_points = [point for point in points if point is not None]
            ic = abstract_index_of_closest_position(agent.position, points)
            points[ic] = None
            result[ic] = agent
        return result 

    def split_to_groups(self):
        was_groups = self.groups
        agents = [agent for group in self.groups for agent in group if agent.is_active]
        new_groups = [[] for i in range(len(self.agents))]
        for agent in agents:
            for group in new_groups:
                if self.is_close_agent_exists(agent, group):
                    agent.id = len(group)
                    group.append(agent)
                    break
        self.groups = sorted(new_groups, key=lambda x: len(x), reverse=True)

    def is_close_agent_exists(self, agent, group):
        for group_agent in group:
            if group_agent is agent:
                continue
            if abs(agent.position - group_agent.position) < agent.sensetivity_r:
                return True
        if not group:
            return True
        return False

    def switch(self):
        self.split_to_groups()
        for group in self.groups:
            if len(group) > 0:
                master = group[0]
                if not master.is_master:
                    master.switch_to_master()

                for i in range(1, len(group)):
                    minion = group[i]
                    if not minion.is_minion:
                        minion.switch_to_minion()
                    minion.master = group[0]


    def merge_groups(self, groups_was, groups_now):
        main_group_was = self.what_is_main_group(groups_was)
        main_group_now = self.what_is_main_group(groups_now)

    def what_is_main_group(self, group):
        return min(group, key=lambda x: min([abs(agent.position - agent.trajectory[-1]) for agent in x]))


    def influence_list(self, agent):
        other_agents = []
        for a in [agent for agent in self.agents if agent.is_active]:
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
            current_angle = angle(self.ps.orientation, agent.master.trajectory_agent.current_orientation)
            full_phi = current_angle + rel.phi
            sin = math.sin(full_phi)
            cos = math.cos(full_phi)
            target = infl_a.position + \
                     norm(V(self.ps.orientation.x * cos  + self.ps.orientation.y * sin, 
                            -self.ps.orientation.x * sin  + self.ps.orientation.y * cos)) * rel.r
            targets.append(target)
        if not targets:
            return agent.position
        else:
            return sum(targets, V(0, 0)) / len(targets)

    def update(self):
        for group in self.groups:
            if len(group) > 0:
                master = group[0]
                master.update()
                for i in range(1, len(group)):
                    minion = group[i]
                    target = self.calculate_target_for_minion(minion)
                    minion.update_target(target)
                    minion.update()
