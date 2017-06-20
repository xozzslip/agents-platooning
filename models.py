from base import (V, RV, Story, angle, point_vector_distance, index_of_closest_position, norm,
    noneg)
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
    def __init__(self, trajectory, velocity=None, position=None):
        # Условие необходимое для корректного 
        # определения current_orientation у строя
        assert len(trajectory) >= 2 
        self.trajectory = trajectory
        self.current_position = 0
        self.desired_velocity = velocity or 60
        position = position or trajectory[0]
        super().__init__(position=position)

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
            full_force += norm(to_next_point_force) * noneg(velocity_diff) * self.PID[0]
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
            return tr[-1] - tr[-2]
        return self.velocity

    def update(self):
        super().update()
        self.update_current_position()


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
        if abs(full_force) > self.MAX_FORCE:
            full_force = norm(full_force) * self.MAX_FORCE
        return full_force


class FlexAgent(Agent):
    """Агент, который при необходимости может становиться мастером и миньоном"""
    def __init__(self, trajectory, desired_velocity, position=None):
        super().__init__(position)

        # Идентефикатор: будет учавствовать в разрешении
        # спорных ситуаций между агентами
        self.id = None

        self.trajectory = trajectory
        self.desired_velocity = desired_velocity

        self.trajectory_agent = None
        self.target_agent = None
        self.is_master = False

        self.sensetivity_r = 100
        self._external_force = V(0, 0)

    def switch_to_master(self):
        self.is_master = True
        self.trajectory_agent = TrajectoryAgent(
            trajectory=self.trajectory,
            velocity=self.desired_velocity,
            position=self.position
        )
        closest_tr_pos = index_of_closest_position(self.trajectory_agent, self.trajectory_agent.trajectory)
        self.trajectory_agent.current_position = closest_tr_pos

    def update_under_agent(self):
        """Приводит состояние подагента в тоже состояние, что и реальный агент"""
        if self.is_master:
            self.trajectory_agent.position = self.position
            self.trajectory_agent.velocity = self.velocity
            self.trajectory_agent.acceleration = self.acceleration
            self.trajectory_agent.update_current_position()
        else:
            self.target_agent.position = self.position
            self.target_agent.velocity = self.velocity
            self.target_agent.acceleration = self.acceleration

    def switch_to_minion(self, target):
        self.is_master = False
        self.target_agent = TargetAgent(self.position)
        self.target_agent.target = target

    def update_target(self, target):
        if self.is_master:
            raise Exception("cant update target for master")
        self.target_agent.update_target(target)

    def internal_force(self):
        self.update_under_agent()
        if self.is_master:
            return self.trajectory_agent.force()
        else:
            return self.target_agent.force()

    def set_external_force(self, force):
        self._external_force = force 

    def external_force(self):
        ext_f = self._external_force
        self._external_force = V(0, 0)
        return ext_f

    def force(self):
        return self.internal_force() + self.external_force()

    def __repr__(self):
        return "<A{}({}, {})>".format(self.id, self.position.x, self.position.y)
