from models import Agent
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
	def __init__(self):
		super().__init__()
		self.initial_orientation = V(1, 0)

	def force(self):
		if len(self.story.positions) == 1:
			return V(0, 0)
		else:
			return V(1, 0)
