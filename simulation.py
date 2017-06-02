import numpy as np
import matplotlib.pyplot as plt
import math
from base import RV, V, angle, to_degree, norm
from models import PlatoonStruct, TargetAgent, TrajectoryAgent, TrajectoryPlatoon
from agents import DiracAgent, HeavisideAgent
from matplotlib.legend_handler import HandlerLine2D


if __name__ == '__main__':
	points = [V(0, 0), V(-0.1, -0.1)]
	orientation = V(0, 1)
	relations = [None, 0]
	ps = PlatoonStruct(points, orientation, relations)
	# Симпотная траектория V(math.sin(x) ** 3, x)
	trajectory = [V(math.sin(x) ** 7, x ** (2/3)) for x in np.arange(0, 6, 0.1)]

	master = TrajectoryAgent(trajectory) # Находится в V(0, 0)
	minions = [TargetAgent(V(0, 0))]
	tp = TrajectoryPlatoon(master=master, minions=minions, platoon_struct=ps)

	times = np.arange(0, 1000, 1)
	for t in times:
		tp.update()
		# print("angle={0:.2f}, target={1}, minion={2}, master={3}".format(
		# 	to_degree(tp.current_angle), tp.minions[0].target, tp.minions[0].position, tp.master.position)
		# )
	# traj,  = plt.plot([p.x for p in trajectory], [p.y for p in trajectory], 'bo', alpha=0.2, label='Trajectory')
	plt.plot([p.x for p in tp.master.story.positions], [p.y for p in tp.master.story.positions], 'black')
	# plt.plot(tp.master.story.positions[0].x, tp.master.story.positions[0].y, 'black', marker='^')
	# plt.plot(tp.master.story.positions[-1].x, tp.master.story.positions[-1].y, 'black', marker='o')
	# plt.plot([p.x for p in tp.minions[0].story.positions], [p.y for p in tp.minions[0].story.positions])
	# plt.plot([p.x for p in tp.targets[0]], [p.y for p in tp.targets[0]])
	# plt.plot(times, [e for e in tp.minions_errors[0]])
	# print(tp.minions_errors[0])
	# plt.plot(times, [abs(p) for p in tp.master.story.velocities], 'black')

	print(list(map(abs, tp.master.story.accelerations)))
	# plt.legend(handler_map={traj: HandlerLine2D(numpoints=4)})
	plt.axis('equal')
	plt.ylabel('velocity')
	plt.xlabel('time')
	plt.show()
