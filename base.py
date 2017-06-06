import math
import random


class V:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __iter__(self):
        yield self.x
        yield self.y

    def __mul__(self, b):
        return V(self.x * b, self.y * b)

    def __truediv__(a, b):
        return V(a.x / b, a.y / b)

    def __floordiv__(a, b):
        return V(a.x // b, a.y // b)

    def __add__(a, b):
        return V(a.x + b.x, a.y + b.y)

    def __sub__(a, b):
        return a + b * (-1)

    def __abs__(a):
        return math.sqrt(a.x ** 2 + a.y ** 2)

    def __repr__(self):
        return "<{0}({1:.2f}, {2:.2f})>".format(self.__class__.__name__, self.x, self.y, abs(self))

    def norm(self):
        if abs(self) > 0:
            return self / abs(self)
        return self


def norm(v):
    if abs(v) > 0:
        return v / abs(v)
    return V(0, 0)  


def angle(v1, v2):
    dot = v1.x * v2.x + v1.y * v2.y     # dot product
    det = v1.x * v2.y + v1.y * v2.x     # determinant
    angle = math.atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
    return angle

def to_degree(angle):
    return angle / 2 / math.pi * 360

def mult(v1, v2):
    return v1.x * v2.x + v1.y * v2.y


class Entity:
    def __init__(self, position):
        self.position = position


class Target(Entity):
    pass


class Force:
    @staticmethod
    def gravity(v1, v2):
        v = v2 - v1
        if abs(v) == 0:
            return V(0, 0)
        return v / abs(v)

    @staticmethod
    def repulsion(v1, v2):
        v = v2 - v1
        if abs(v) == 0:
            return V(0, 0)
        return 1 / (v / abs(v)) ** 3


class Story:
    def __init__(self):
        self.positions = []
        self.velocities = []
        self.accelerations = []

    def add(self, position, velocity, acceleration):
        self.positions.append(position)
        self.velocities.append(velocity)
        self.accelerations.append(acceleration)


class RV:
    def __init__(self, r, phi):
        self.r = r
        self.phi = phi

    @property
    def phi_d(self):
        return self.phi / 2 / math.pi * 360

    def __repr__(self):
        return "<{0}(R={1:.2f}, phi={2:.2f}({3:.1f}))>".format(self.__class__.__name__, self.r, self.phi, self.phi_d)


def point_vector_distance(OA, OB, OC):
    """
    A               D         B
    o---------------o---------o
                    |
                    |
                    |
                    o
                    C
    What is CD?
    """
    AB = OB - OA
    AC = OC - OA
    AD = AB * mult(AB, AC) / mult(AB, AB)
    CD = AD - AC
    return CD


def index_of_closest_position(agent, desired_struct):
    min_dist, min_indx = math.inf, 0
    if not desired_struct:
        return None
    for i in range(len(desired_struct)):
        if abs(desired_struct[i] - agent.position) < min_dist:
            min_dist = abs(desired_struct[i] - agent.position)
            min_indx = i
    return min_indx
