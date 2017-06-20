import numpy as np

import matplotlib


import math
import matplotlib.pyplot as plt
from base import RV, V, angle, to_degree, norm
from models import TargetAgent, TrajectoryAgent, FlexAgent
from structs import PlatoonStruct, PlatoonFullStruct
from agents import DiracAgent, HeavisideAgent, Target1DAgent
from matplotlib.legend_handler import HandlerLine2D
from platoons import TrajectoryPlatoon, FlexTrajectoryPlatoon


def minion_sim():
    points = [V(0, 0), V(-1, 0)]
    orientation = V(1, 0)
    relations = [None, 0]
    ps = PlatoonStruct(points, orientation, relations)

    master = HeavisideAgent(position=V(0, 0))
    minions = [Target1DAgent(position=V(-1, 0))]
    tp = TrajectoryPlatoon(master=master, minions=minions, platoon_struct=ps)
    times = np.arange(0, 35, 0.1)
    for t in times:
        tp.update()

    plt.axhline(0, color='grey', linestyle='--', alpha=0.8)
    tr_er, = plt.plot(times, tp.minions[0].d_story['signed_target_error'], 'k', label='Отклонение миньона от\nвиртуального лидера')
    plt.ylabel('Отклонение, м')
    plt.xlabel('Время, с')
    plt.legend(handles=[tr_er], loc=1)
    plt.savefig('text/minion/heviside_targerr.png', bbox_inches='tight')
    plt.clf()
    
    min_a, = plt.plot(times, [abs(p) for p in tp.minions[0].story.accelerations], 'k', label='Ускорение миньона')
    mas_a, = plt.plot(times, [abs(p) for p in master.story.accelerations], 'b--', label='Ускорение виртуального лидера')
    plt.ylabel('Ускорение, м/c^2')
    plt.xlabel('Время, с')
    plt.legend(handles=[min_a, mas_a], loc=4)
    plt.savefig('text/minion/heviside_accelerations.png', bbox_inches='tight')
    plt.clf()

    min_v, = plt.plot(times[40:130], [abs(p) for p in tp.minions[0].story.velocities[40:130]], 'k', label='Скорость миньона')
    mas_v, = plt.plot(times[40:130], [abs(p) for p in master.story.velocities[40:130]], 'b--', label='Скорость виртуального лидера')
    plt.ylabel('Скорость, м/с')
    plt.xlabel('Время, с')
    plt.legend(handles=[min_v, mas_v], loc=2)
    plt.savefig('text/minion/heviside_velocity.png', bbox_inches='tight')
    plt.clf()

    plt.axhline(0, color='grey', linestyle='--', alpha=0.8)
    velocities_diff = [(minion - mast).x for mast, minion in zip(tp.minions[0].story.velocities[:-1], master.story.velocities[1:])]
    v_err, = plt.plot(times[:-1], velocities_diff, 'k', label='Отклонение скорости\nминьона от скорости\nвиртуального лидера')
    plt.ylabel('Отклонение скорости, м/с')
    plt.xlabel('Время, с')
    plt.legend(handles=[v_err], loc=1)
    plt.savefig('text/minion/heviside_velerr.png', bbox_inches='tight')
    plt.clf()

    plt.axhline(0, color='grey', linestyle='--', alpha=0.8)
    velocities_diff = [(minion - mast).x for mast, minion in zip(tp.minions[0].story.accelerations, master.story.accelerations)]
    ac_err, = plt.plot(times, velocities_diff, 'k', label='Отклонение ускорения\nминьона от ускорения\nвиртуального лидера')
    plt.ylabel('Отклонение ускорения, м/с^2')
    plt.xlabel('Время, с')
    plt.legend(handles=[ac_err], loc=1)
    plt.savefig('text/minion/heviside_axerr.png', bbox_inches='tight')
    plt.clf()


def platoon_sim():
    points = [V(0, 0), V(-100, -100), V(100, -100)]
    orientation = V(0, 1)
    relations = [None, 0, 0]
    ps = PlatoonStruct(points, orientation, relations)

    trajectory = [V(300 * math.cos(x / 300), x) for x in np.arange(0, 5000, 100)]

    master = TrajectoryAgent(trajectory, 20) # Находится в V(0, 0)
    minions = [TargetAgent(), TargetAgent()]
    tp = TrajectoryPlatoon(master=master, minions=minions, platoon_struct=ps)

    times = np.arange(0, 450, 0.1)
    for t in times:
        tp.update()

    # Траектория с миньонами
    traj,  = plt.plot([p.x for p in trajectory], [p.y for p in trajectory], 'bo', alpha=0.2, label='Траектория')
    track, = plt.plot([p.x for p in tp.master.story.positions], [p.y for p in tp.master.story.positions], 'black', label='Мастер')
    track2, = plt.plot([p.x for p in tp.minions[0].story.positions], [p.y for p in tp.minions[0].story.positions], 'green', label='Миньоны')
    track3, = plt.plot([p.x for p in tp.minions[1].story.positions], [p.y for p in tp.minions[1].story.positions], 'green')
    s, = plt.plot(tp.master.story.positions[0].x, tp.master.story.positions[0].y, 'k>', label='Исходная точка')
    f, = plt.plot(tp.master.story.positions[-1].x, tp.master.story.positions[-1].y, 'ko', label='Конечная точка')
    s, = plt.plot(tp.minions[0].story.positions[0].x, tp.minions[0].story.positions[0].y, 'k>', label='Исходная точка')
    f, = plt.plot(tp.minions[0].story.positions[-1].x, tp.minions[0].story.positions[-1].y, 'ko', label='Конечная точка')
    s, = plt.plot(tp.minions[1].story.positions[0].x, tp.minions[1].story.positions[0].y, 'k>', label='Исходная точка')
    f, = plt.plot(tp.minions[1].story.positions[-1].x, tp.minions[1].story.positions[-1].y, 'ko', label='Конечная точка')
    plt.legend(handler_map={traj: HandlerLine2D(numpoints=4)}, handles=[traj, track, track2], loc=2)
    plt.ylabel('Координата $y$, м')
    plt.xlabel('Координата $x$, м')
    plt.axis('equal')
    plt.savefig('text/platoon-trajectory-0.png', bbox_inches='tight')
    plt.clf()


def master_sim():
    points = [V(0, 0), V(-100, -100), V(100, -100)]
    orientation = V(0, 1)
    relations = [None, 0, 0]
    ps = PlatoonStruct(points, orientation, relations)

    trajectory = [V(300 * math.cos(x / 300), x) for x in np.arange(0, 5000, 100)]

    master = TrajectoryAgent(trajectory, 20) # Находится в V(0, 0)
    minions = [TargetAgent(), TargetAgent()]
    tp = TrajectoryPlatoon(master=master, minions=minions, platoon_struct=ps)

    times = np.arange(0, 450, 0.1)
    for t in times:
        tp.update()

    # Траектория
    traj,  = plt.plot([p.x for p in trajectory], [p.y for p in trajectory], 'bo', alpha=0.2, label='Траектория')
    track, = plt.plot([p.x for p in tp.master.story.positions], [p.y for p in tp.master.story.positions], 'black', label='След движения')
    s, = plt.plot(tp.master.story.positions[0].x, tp.master.story.positions[0].y, 'k^', label='Исходная точка')
    f, = plt.plot(tp.master.story.positions[-1].x, tp.master.story.positions[-1].y, 'ko', label='Конечная точка')
    plt.legend(handler_map={traj: HandlerLine2D(numpoints=4)}, handles=[traj, track, f, s], loc=1)
    plt.ylabel('Координата $y$, м')
    plt.xlabel('Координата $x$, м')
    plt.axis('equal')
    plt.savefig('text/master-trajectory-0.png', bbox_inches='tight')
    plt.show()
    plt.clf()

    # Отклонение от траектории
    err, = plt.plot(times[0:len(tp.master.d_story['vector_to_traj'])], [p for p in tp.master.d_story['vector_to_traj']], 'black', label='Отколонение')
    plt.plot([i for i in range(-50, 1100)], [0 for _ in range(1150)], 'b--')
    p, = plt.plot(50, 0, 'r.', label='Точки перепада')
    plt.plot(111, 0, 'red', marker='.')
    plt.plot(172, 0, 'red', marker='.'), plt.plot(226, 0, 'red', marker='.'), plt.plot(289, 0, 'red', marker='.')
    plt.ylabel('Отклонение, м')
    plt.xlabel('Время, с')
    plt.legend(handles=[err, p], loc=2)
    plt.xlim(0, 300)
    plt.savefig('text/master-trajectory-error.png', bbox_inches='tight')
    plt.clf()

    # Скорость
    plt.plot([i for i in range(-100, 1550)], [0 for _ in range(1650)], 'b--')
    vel, = plt.plot(times, [abs(p) for p in tp.master.story.velocities], 'black', label='Скорость')
    s, = plt.plot(0, abs(tp.master.story.velocities[0]), 'kp')
    f, = plt.plot(times[-1], abs(tp.master.story.velocities[-1]), 'kp')
    bord, = plt.plot([i for i in range(-100, 1550)], [18.8 for _ in range(1650)], 'r--', label='Зона стац.\nдвижения')
    plt.plot([i for i in range(-100, 1550)], [21.5 for _ in range(1650)], 'r--')
    plt.ylabel('Скорость, м/c')
    plt.xlabel('Время, с')
    plt.xlim(-20, 450)
    plt.legend(handles=[vel, bord], loc=1)
    plt.savefig('text/master-trajectory-0-velocity.png', bbox_inches='tight')
    plt.clf()

    # Скорость в пределах стац. движения
    av, = plt.plot([i for i in range(-100, 1550)], [20 for _ in range(1650)], 'b--', label='Желаемая скорость')
    vel, = plt.plot(times, [abs(p) for p in tp.master.story.velocities], 'black', label='Скорость')
    plt.ylabel('Скорость, м/c')
    plt.xlabel('Время, с')
    plt.xlim(20, 200)
    plt.ylim(18, 22)
    plt.legend(handles=[vel, av], loc=4)
    plt.savefig('text/master-trajectory-1-velocity.png', bbox_inches='tight')


    # Точки изменения стороны следования относительно траектории
    traj,  = plt.plot([p.x for p in trajectory], [p.y for p in trajectory], 'bo', alpha=0.2,  label='Траектория')
    for t in [500, 1100, 1720, 2260, 2890]:
        pch1, = plt.plot(master.story.positions[t].x, master.story.positions[t].y, 'ro', label='Точки изменения\nстороны следования')
    for t in [610, 1180, 1770, 2340, 2920]:
        pch2, = plt.plot(master.story.positions[t].x, master.story.positions[t].y, 'ko', label='Точки изменения\nпервой производной')
    plt.legend(handler_map={traj: HandlerLine2D(numpoints=4)}, handles=[traj, pch1, pch2], loc=1)
    plt.ylabel('Координата $y$, м')
    plt.xlabel('Координата $x$, м')
    plt.axis('equal')
    plt.savefig('text/master-trajectory-changes-2.png', bbox_inches='tight')
    plt.clf()


def flex_platoon_sim():
    points = [V(0, 0), V(5, 0), V(-5, 0), V(-10, 0), V(10, 0), V(0, -15)]
    orientation = V(0, 1)
    ps = PlatoonFullStruct(points, orientation)

    # Траектория — полукруг
    tr = [V(math.sin(0.01 * x) * 100, math.cos(0.01 * x) * 100) for x in np.arange(0, 300, 10)]
    des_v = 5
    agents_positions = [V(-15, 95), V(-9, 107), V(-22, 90), V(-25, 89), V(-30, 110), V(-15, 100)]
    agents = [FlexAgent(tr, des_v, p) for p in agents_positions]

    ftp = FlexTrajectoryPlatoon(agents, ps)

    ftp.switch()

    agents_track = [[] for _ in range(len(ftp.agents))]
    for _ in range(300):
        ftp.update()

    for _ in range(4):
        ftp.agents[2].set_external_force(V(230, 0))
        ftp.switch()
        ftp.update()

    ftp.switch()
    for _ in range(500):
        ftp.update()


    traj,  = plt.plot([p.x for p in tr], [p.y for p in tr], 'bo', alpha=0.2, label='Траектория')
    for i in range(len(ftp.agents)):
        track, = plt.plot([p.x for p in ftp.agents[i].story.positions], [p.y for p in ftp.agents[i].story.positions])
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    flex_platoon_sim()