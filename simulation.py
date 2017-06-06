import numpy as np

import matplotlib


import math
import matplotlib.pyplot as plt
from base import RV, V, angle, to_degree, norm
from models import PlatoonStruct, TargetAgent, TrajectoryAgent, TrajectoryPlatoon
from agents import DiracAgent, HeavisideAgent
from matplotlib.legend_handler import HandlerLine2D


if __name__ == '__main__':
    points = [V(0, 0), V(-100, -100), V(100, -100)]
    orientation = V(0, 1)
    relations = [None, 0, 0]
    ps = PlatoonStruct(points, orientation, relations)

    trajectory = [V(300 * math.cos(x / 300), x) for x in np.arange(0, 5000, 100)]

    master = TrajectoryAgent(trajectory, 60) # Находится в V(0, 0)
    minions = [TargetAgent(), TargetAgent()]
    tp = TrajectoryPlatoon(master=master, minions=minions, platoon_struct=ps)

    times = np.arange(0, 1100, 1)
    for t in times:
        tp.update()

    traj,  = plt.plot([p.x for p in trajectory], [p.y for p in trajectory], 'bo', alpha=0.2, label='Траектория')
    track, = plt.plot([p.x for p in tp.master.story.positions], [p.y for p in tp.master.story.positions], 'black', label='След движения')
    s, = plt.plot(tp.master.story.positions[0].x, tp.master.story.positions[0].y, 'k^', label='Исходная точка')
    f, = plt.plot(tp.master.story.positions[-1].x, tp.master.story.positions[-1].y, 'ko', label='Конечная точка')
    # plt.plot([p.x for p in tp.minions[0].story.positions], [p.y for p in tp.minions[0].story.positions])
    # plt.plot([p.x for p in tp.minions[1].story.positions], [p.y for p in tp.minions[1].story.positions])
    plt.axis('equal')

    # plt.plot([p.x for p in tp.targets[0]], [p.y for p in tp.targets[0]])
    # plt.plot(times, [e for e in tp.minions_errors[0]])
    # print(tp.minions_errors[0])
    

    # print(list(map(abs, tp.master.story.velocitie)))
    plt.legend(handler_map={traj: HandlerLine2D(numpoints=4)}, handles=[traj, track, s, f], loc=1)
    # plt.plot(times, [abs(p) for p in tp.master.story.accelerations], 'black')
    # err, = plt.plot(times, [p for p in tp.master.d_story['vector_to_traj']], 'black', label='Отколонение')
    # plt.plot([i for i in range(-50, 1050)], [0 for _ in range(1100)], 'b--')
    # p, = plt.plot(200, 0, 'r.', label='Точка перепада')
    # plt.plot(404, 0, 'red', marker='.')
    # plt.plot(601, 0, 'red', marker='.'), plt.plot(799, 0, 'red', marker='.')
    plt.ylabel('Координата $y$')
    plt.xlabel('Координата $x$')
    # plt.legend(handles=[err, p], loc=1)
    plt.savefig('text/master-trajectory-0.png', bbox_inches='tight')

    plt.show()
