from base import V, RV, Story, angle, point_vector_distance, index_of_closest_position, norm
import math
from collections import defaultdict


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


class PlatoonFullStruct:
    """Структура содержащая все связи"""
    def __init__(self, points, orientation):
        self.points = points
        self.orientation = orientation
        self.relative_positions = self.build_relative_positions()

    def build_relative_positions(self):
        relative_positions = [[None for _ in range(len(self.points))] for _ in range(len(self.points))]
        for i in range(len(relative_positions)):
            for j in range(len(relative_positions[i])):
                if i == j:
                    continue
                master_point = self.points[j]
                point = self.points[i]
                r = abs(point - master_point)
                phi = angle(point - master_point, self.orientation)
                relative_positions[i][j] = RV(r, phi)
        return relative_positions

    def __len__(self):
        return len(self.points)