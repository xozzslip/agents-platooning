from models import Agent, TargetAgent
from base import V, RV, Story, angle, point_vector_distance, index_of_closest_position, norm
import math


class DiracAgent(Agent):
    """Агент, на которого действует дельта функция"""
    def __init__(self):
        super().__init__()
        self.initial_orientation = V(1, 0)

    def force(self):
        if len(self.story.positions) == 1:
            return V(100, 0)
        else:
            return V(0, 0)


class HeavisideAgent(Agent):
    def __init__(self, position=None):
        super().__init__(position)
        self.initial_orientation = V(1, 0)
        self.x = 0

    def force(self):
        if len(self.story.positions) < 50:
            return V(0, 0)
        else:
            return V(1, 0)

    @property
    def current_orientation(self):
        return self.initial_orientation


class Target1DAgent(TargetAgent):
    def update(self):
        # Хак для того, чтобы определять, обгоняет миньон таргет или нет
        dist = self.target - self.position
        sign = 1 if dist.x >= 0 else -1
        self.d_story['signed_target_error'].append(dist.x)
        super().update()