from base import V, RV, Story, angle, point_vector_distance, index_of_closest_position, norm
import math


DELTA_T = 0.1


class Agent:
	PID = (0.7, 0.9, 1)
	MAX_FORCE = 0.05

	def __init__(self, position=None):
		self.position = position or V(0, 0)
		self.velocity = V(0, 0)
		self.acceleration = V(0, 0)

		self.mass = 1
		self.story = Story()

	def force(self):
		raise Exception.NotImplementedError

	def friction_force(self):
		return (self.velocity * (-0.1))

	def update(self):
		self.story.add(self.position, self.velocity, self.acceleration)

		self.acceleration = (self.force()) / self.mass
		self.velocity = self.velocity + self.acceleration * DELTA_T
		self.position = self.position + self.velocity * DELTA_T


class TrajectoryAgent(Agent):
	def __init__(self, trajectory, velocity=None):
		# Условие необходимое для корректного 
		# определения current_orientation у строя
		assert len(trajectory) >= 2 
		self.trajectory = trajectory
		self.current_position = 0
		self.desired_velocity = velocity or 0.08
		super().__init__(position=trajectory[0])

	def force(self):
		cur_point = self.trajectory[self.current_position]
		if self.current_position + 1 < len(self.trajectory):
			next_point = self.trajectory[self.current_position + 1]
		else:
			to_final = self.trajectory[-1] - self.position
			if abs(to_final) > abs(self.acceleration * self.mass):
				to_final = norm(to_final) * abs(self.acceleration * self.mass)
			return self.velocity * (-1) * self.PID[1] + to_final * self.PID[0]
		vector_to_traj = point_vector_distance(next_point, cur_point, self.position)
		to_next_point_force = next_point - self.position
		velocity_diff = self.desired_velocity - abs(self.velocity)
		full_force = V(0, 0)
		full_force += vector_to_traj * 1.5
		full_force += norm(to_next_point_force) * velocity_diff * self.PID[0]
		full_force += self.acceleration * self.PID[1] * (-1)
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
	def __init__(self, position=None):
		super().__init__(position)
		self.target = self.position

	def update_target(self, target):
		self.target = target

	def force(self):
		full_force = V(0, 0)
		full_force += (self.target - self.position) * self.PID[0]
		full_force += self.velocity * self.PID[1] * (-1)
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
