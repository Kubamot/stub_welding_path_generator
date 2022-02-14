import math
from utils import Point
import numpy as np


class Stub():
    def __init__(self, radius, axis_offset, axis_rotation=0, number_of_path_points=12):
        self._radius = radius
        self._axis_offset = axis_offset
        self._axis_rotation = axis_rotation
        if (number_of_path_points % 4 != 0):
            raise ValueError("Path points must be a multiple of four !")
        self._number_of_path_points = number_of_path_points
        self._points_for_quarter = number_of_path_points // 4
        self._path_points = []

    def create_path_points(self, pipe_radius):
        """
        Counting cross section path by formula from :
        https://www.obliczeniowo.com.pl/267
        """
        max_arc_radius = self._calc_hypotenuse(pipe_radius, self._radius)
        min_arc_radius = pipe_radius
        # Using 1/4 of point because stub has two plane of symmetry
        delta_arc_radius = (max_arc_radius - min_arc_radius) / \
            self._points_for_quarter
        for i in range(0, self._points_for_quarter + 1):
            arc_radius = min_arc_radius + (i * delta_arc_radius)
            x = self._calc_cathetus(
                arc_radius, pipe_radius) + self._axis_offset
            z = self._calc_cathetus(arc_radius, self._radius)
            y = self._calc_cathetus(pipe_radius, z)
            self._path_points.append(Point([x, y, z]))
        self._mirror_points_by_YZ_plane()
        self._mirror_points_by_XZ_plane()

    def _calc_hypotenuse(self, cathetus_one, cathetus_two):
        return math.sqrt(cathetus_one ** 2 + cathetus_two ** 2)

    def _calc_cathetus(self, hypotenuse, cathetus):
        return math.sqrt(hypotenuse ** 2 - cathetus ** 2)

    def _mirror_points_by_YZ_plane(self):
        for point in reversed(self._path_points):
            if point.y == 0:
                continue
            self._path_points.append(Point([point.x, -point.y, point.z]))

    def _mirror_points_by_XZ_plane(self):
        for point in reversed(self._path_points):
            if point.x == 0:
                continue
            self._path_points.append(Point([-point.x, point.y, point.z]))

    def print_path(self):
        for point in self._path_points:
            print(point.to_string())
