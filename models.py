from base import V, RV, Story, angle, point_vector_distance, index_of_closest_position, norm
import math
from collections import defaultdict


DELTA_T = 0.1


class Agent:
    PID = (0.4, 0.4, 0.01)
    MAX_FORCE = 40

    def __init__(self, position=None):
        self.position = position or V(0, 0)
        self.velocity = V(0, 0)
        self.acceleration = V(0, 0)

        self.mass = 1
        self.story = Story()
        self.d_story = defaultdict(list)

    def force(self):
        raise Exception.NotImplementedError

    def friction_force(self):
        return (self.velocity * (-0.1))

    def update(self):
        force = self.force()

        self.story.add(self.position, self.velocity, self.acceleration, force   )

        self.acceleration = force / self.mass
        self.velocity = self.velocity + self.acceleration * DELTA_T
        self.position = self.position + self.velocity * DELTA_T


class TrajectoryAgent(Agent):
    def __init__(self, trajectory, velocity=None):
        # Условие необходимое для корректного 
        # определения current_orientation у строя
        assert len(trajectory) >= 2 
        self.trajectory = trajectory
        self.current_position = 0
        self.desired_velocity = velocity or 60
        super().__init__(position=trajectory[0])

        self.prev = V(0, 0)


    def force(self):
        cur_point = self.trajectory[self.current_position]
        if self.current_position + 1 < len(self.trajectory):
            next_point = self.trajectory[self.current_position + 1]
            vector_to_traj = point_vector_distance(next_point, cur_point, self.position)
            self.d_story['vector_to_traj'].append(abs(vector_to_traj) * (1 if (next_point.x - cur_point.x) * (vector_to_traj.y - cur_point.y) - (next_point.y - cur_point.y) * (vector_to_traj.x - cur_point.x) >= 0 else -1))
            to_next_point_force = next_point - self.position
            velocity_diff = self.desired_velocity - abs(self.velocity)
            full_force = V(0, 0)
            full_force += vector_to_traj * 1.5
            full_force += (vector_to_traj - self.prev) * 30
            self.prev = vector_to_traj
            full_force += norm(to_next_point_force) * velocity_diff * self.PID[0]
            full_force += self.acceleration * self.PID[1] * (-1)
        else:
            to_final = self.trajectory[-1] - self.position
            full_force = to_final * self.PID[0] / 2.2 - self.velocity * self.PID[1] 
        if abs(full_force) > self.MAX_FORCE:
            full_force = norm(full_force) * self.MAX_FORCE
        return full_force

    def update_current_position(self):
        if self.current_position >= len(self.trajectory):
            return
        closest_ind = index_of_closest_position(
            self, self.trajectory[:self.current_position+3])
        if closest_ind < self.current_position:
            return
        self.current_position = closest_ind 

    @property
    def current_orientation(self):
        cur_pos = self.current_position
        tr = self.trajectory
        if cur_pos >= len(tr) - 1:
            return tr[cur_pos] - tr[cur_pos - 1]
        if abs(self.velocity) < 0.001:
            return tr[cur_pos + 1] - tr[cur_pos]
        return self.velocity
        

    def update(self):
        super().update()
        self.update_current_position()



class PlatoonStruct:
    def __init__(self, points, orientation, relations):
        """
            0 o 
             / \
          1 o   o 2
           /     \
        3 o       o 4

        points = [V(0, 0), ...]
        orientation = V(0, 1)
        relations = [None, 0, 0, 1, 2]
        """
        assert len(points) == len(relations)
        assert relations.count(None) == 1
        self.points = points
        self.orientation = orientation
        self.relations = relations
        self.relative_positions = self.build_relative_positions()

    def build_relative_positions(self):
        relative_positions = [None] * len(self.relations)
        for i in range(len(self.relations)):
            if self.relations[i] is None:
                continue
            master_point = self.points[self.relations[i]]
            point = self.points[i]
            r = abs(point - master_point)
            phi = angle(point - master_point, self.orientation)
            relative_positions[i] = RV(r, phi)
        return relative_positions

    def __len__(self):
        assert len(self.points) == len(self.relations)
        return len(self.points)


class TargetAgent(Agent):
    """Агент, напрямую следующий к точке"""
    PID = (0.6, 1.3, 0.1)

    def __init__(self, position=None):
        super().__init__(position)
        self.target = self.position
        self.dist_sum = V(0, 0)
        self.vel_sum = V(0, 0)

    def update_target(self, target):
        self.d_story['target'].append(self.target)
        self.target = target

    def force(self):
        full_force = V(0, 0)
        dist = self.target - self.position
        full_force += dist * self.PID[0]

        target_velocity = (self.target - self.d_story['target'][-1]) / DELTA_T
        velocity_diff = target_velocity - self.velocity

        self.dist_sum += dist * DELTA_T
        full_force += self.dist_sum * self.PID[2]
        full_force += velocity_diff * self.PID[1]
        return full_force


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
