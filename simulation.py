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

    trajectory = [V(x, 2000 * math.sin(x / 2000) ** 3) for x in np.arange(0, 3000, 100)]

    master = TrajectoryAgent(trajectory, 20) # Находится в V(0, 0)
    minions = [TargetAgent(), TargetAgent()]
    tp = TrajectoryPlatoon(master=master, minions=minions, platoon_struct=ps)

    times = np.arange(0, 200, 0.1)
    for t in times:
        tp.update()



    # # Траектория
    # traj,  = plt.plot([p.x for p in trajectory], [p.y for p in trajectory], 'bo', alpha=0.2, label='Траектория')
    # track, = plt.plot([p.x for p in tp.master.story.positions], [p.y for p in tp.master.story.positions], 'black', label='След движения')
    # s, = plt.plot(tp.master.story.positions[0].x, tp.master.story.positions[0].y, 'k^', label='Исходная точка')
    # f, = plt.plot(tp.master.story.positions[-1].x, tp.master.story.positions[-1].y, 'ko', label='Конечная точка')
    # plt.legend(handler_map={traj: HandlerLine2D(numpoints=4)}, handles=[traj, track, f, s], loc=1)
    # plt.ylabel('Координата $y$, м')
    # plt.xlabel('Координата $x$, м')
    # plt.axis('equal')
    # plt.show()
    # plt.savefig('text/master-trajectory-0.png', bbox_inches='tight')
    # plt.clf()

    # # Отклонение от траектории
    # err, = plt.plot(times[0:len(tp.master.d_story['vector_to_traj'])], [p for p in tp.master.d_story['vector_to_traj']], 'black', label='Отколонение')
    # plt.plot([i for i in range(-50, 1100)], [0 for _ in range(1150)], 'b--')
    # p, = plt.plot(50, 0, 'r.', label='Точки перепада')
    # plt.plot(111, 0, 'red', marker='.')
    # plt.plot(172, 0, 'red', marker='.'), plt.plot(226, 0, 'red', marker='.'), plt.plot(289, 0, 'red', marker='.')
    # plt.ylabel('Отклонение, м')
    # plt.xlabel('Время, с')
    # plt.legend(handles=[err, p], loc=2)
    # plt.xlim(0, 300)
    # plt.savefig('text/master-trajectory-error.png', bbox_inches='tight')
    # plt.clf()

    # # Скорость
    # plt.plot([i for i in range(-100, 1550)], [0 for _ in range(1650)], 'b--')
    # vel, = plt.plot(times, [abs(p) for p in tp.master.story.velocities], 'black', label='Скорость')
    # s, = plt.plot(0, abs(tp.master.story.velocities[0]), 'kp')
    # f, = plt.plot(times[-1], abs(tp.master.story.velocities[-1]), 'kp')
    # bord, = plt.plot([i for i in range(-100, 1550)], [18.8 for _ in range(1650)], 'r--', label='Зона стац.\nдвижения')
    # plt.plot([i for i in range(-100, 1550)], [21.5 for _ in range(1650)], 'r--')
    # plt.ylabel('Скорость, м/c')
    # plt.xlabel('Время, с')
    # plt.xlim(-20, 450)
    # plt.legend(handles=[vel, bord], loc=1)
    # plt.savefig('text/master-trajectory-0-velocity.png', bbox_inches='tight')
    # plt.clf()

    # # Скорость в пределах стац. движения
    # av, = plt.plot([i for i in range(-100, 1550)], [20 for _ in range(1650)], 'b--', label='Желаемая скорость')
    # vel, = plt.plot(times, [abs(p) for p in tp.master.story.velocities], 'black', label='Скорость')
    # plt.ylabel('Скорость, м/c')
    # plt.xlabel('Время, с')
    # plt.xlim(20, 200)
    # plt.ylim(18, 22)
    # plt.legend(handles=[vel, av], loc=4)
    # plt.savefig('text/master-trajectory-1-velocity.png', bbox_inches='tight')


    # # Точки изменения стороны следования относительно траектории
    # traj,  = plt.plot([p.x for p in trajectory], [p.y for p in trajectory], 'bo', alpha=0.2,  label='Траектория')
    # for t in [500, 1100, 1720, 2260, 2890]:
    #     pch1, = plt.plot(master.story.positions[t].x, master.story.positions[t].y, 'ro', label='Точки изменения\nстороны следования')
    # for t in [610, 1180, 1770, 2340, 2920]:
    #     pch2, = plt.plot(master.story.positions[t].x, master.story.positions[t].y, 'ko', label='Точки изменения\nпервой производной')
    # plt.legend(handler_map={traj: HandlerLine2D(numpoints=4)}, handles=[traj, pch1, pch2], loc=1)
    # plt.ylabel('Координата $y$, м')
    # plt.xlabel('Координата $x$, м')
    # plt.axis('equal')
    # plt.savefig('text/master-trajectory-changes-2.png', bbox_inches='tight')
    # plt.clf()

    # # Траектория с миньонами
    # traj,  = plt.plot([p.x for p in trajectory], [p.y for p in trajectory], 'bo', alpha=0.2, label='Траектория')
    # track, = plt.plot([p.x for p in tp.master.story.positions], [p.y for p in tp.master.story.positions], 'black', label='Мастер')
    # track2, = plt.plot([p.x for p in tp.minions[0].story.positions], [p.y for p in tp.minions[0].story.positions], 'green', label='Миньоны')
    # track3, = plt.plot([p.x for p in tp.minions[1].story.positions], [p.y for p in tp.minions[1].story.positions], 'green')
    # s, = plt.plot(tp.master.story.positions[0].x, tp.master.story.positions[0].y, 'k>', label='Исходная точка')
    # f, = plt.plot(tp.master.story.positions[-1].x, tp.master.story.positions[-1].y, 'ko', label='Конечная точка')
    # s, = plt.plot(tp.minions[0].story.positions[0].x, tp.minions[0].story.positions[0].y, 'k>', label='Исходная точка')
    # f, = plt.plot(tp.minions[0].story.positions[-1].x, tp.minions[0].story.positions[-1].y, 'ko', label='Конечная точка')
    # s, = plt.plot(tp.minions[1].story.positions[0].x, tp.minions[1].story.positions[0].y, 'k>', label='Исходная точка')
    # f, = plt.plot(tp.minions[1].story.positions[-1].x, tp.minions[1].story.positions[-1].y, 'ko', label='Конечная точка')
    # plt.legend(handler_map={traj: HandlerLine2D(numpoints=4)}, handles=[traj, track, track2], loc=2)
    # plt.ylabel('Координата $y$, м')
    # plt.xlabel('Координата $x$, м')
    # plt.axis('equal')
    # plt.savefig('text/platoon-trajectory-0.png', bbox_inches='tight')
    # plt.clf()