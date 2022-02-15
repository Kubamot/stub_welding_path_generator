import math
import numpy as np
from scipy import interpolate

from utils import unit_vector
from utils import angle_between_vectors
from utils import vectors_bisector
from utils import Point


class Stub():
    def __init__(self, radius, pipe_radius, number_of_path_points=12):
        self._radius = radius
        self._pipe_radius = pipe_radius
        self._axis_offset = axis_offset
        self._axis_rotation = axis_rotation
        if (number_of_path_points % 4 != 0):
            raise ValueError("Path points must be a multiple of four !")
        self._number_of_path_points = number_of_path_points
        self._points_for_quarter = number_of_path_points // 4
        self._path_points = []

    def create_path_points(self):
        """
        Counting cross section path by formula from :
        https://www.obliczeniowo.com.pl/267
        """
        max_arc_radius = self._calc_hypotenuse(self._pipe_radius, self._radius)
        min_arc_radius = self._pipe_radius
        # Using 1/4 of point because stub has two plane of symmetry
        delta_arc_radius = (max_arc_radius - min_arc_radius) / \
            self._points_for_quarter
        for i in range(0, self._points_for_quarter + 1):
            arc_radius = min_arc_radius + (i * delta_arc_radius)
            x = self._calc_cathetus(arc_radius, self._pipe_radius)
            z = self._calc_cathetus(arc_radius, self._radius)
            y = self._calc_cathetus(self._pipe_radius, z)
            self._path_points.append(Point(x, y, z))
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
            self._path_points.append(Point(point.x, -point.y, point.z))

    def _mirror_points_by_XZ_plane(self):
        for point in reversed(self._path_points):
            if point.x == 0:
                continue
            self._path_points.append(Point(-point.x, point.y, point.z))

    def count_rot(self):
        path_data = [point.pose for point in self._path_points]

        leading_vector = get_leading_vectors(path_data)
        torch_ax_vectors = get_torch_ax_vectors(path_data)

    def get_torch_ax_vectors(self, path_data):
        torch_ax_vectors = []
        for vector in path_data:
            horizontal_vector = np.array([0, 0, 1])
            vector_tangled_to_pipe = _get_vector_tangled_to_YZ_pipe_crossection(
                vector)
            torch_ax_vectors.append(unit_vector(vectors_bisector(
                vector_tangled_to_pipe, horizontal_vector)))

        return torch_ax_vectors

    def _get_vector_tangled_to_YZ_pipe_crossection(self, vector):

        plane_angle = self._get_section_plane_angle_around_Z_axis(vector)
        plane_cos = math.cos(section_plane_angle)
        plane_sin = math.sin(section_plane_angle)

        temp_z = -100
        y_origin_shift = -self._calc_hypotenuse(vector[0], vector[1])
        z_origin_shift = -vector[2]

        # y coordinate correspond to above z which create line tangent to
        # Ellipse created by cross plane on cylinder equation
        y_on_ellipse = (
            (-z_origin_shift ** 2 + self._pipe_radius **
             2 + z_origin_shift * temp_z)
            / (-y_origin_shift * cos ** 2)) - y_origin_shift

        return unit_vector(np.array([(y_on_ellipse * sin), (y_on_ellipse * cos), temp_z]))

    def _get_section_plane_angle_around_Z_axis(self, vector):

        vertical_vector = np.array([0, 1])
        vector_on_XY = unit_vector(vector)[0:2]
        return angle_between_vectors(vertical_vector, vector_on_XY)

    def get_leading_vectors(self, path_data):

        vectors_heads = np.array(path_data)
        path_data.insert(0, path_data.pop())
        vectors_tails = np.array(path_data)

        path_vectors = vectors_tails-vectors_heads

        np.insert(path_vectors, 0, path_vectors[-1])
        np.append(path_vectors, path_vectors[1])

        leading_vecotrs = []

        for i in range(1, len(path_vectors)):
            norm_vec_from_last_point = unit_vector(path_vectors[i-1])
            norm_vec_to_next_point = unit_vector(path_vectors[i])
            leading_vector = np.divide(
                norm_vec_from_last_point + norm_vec_to_next_point, 2)
            leading_vecotrs.append(unit_vector(leading_vector))
        return leading_vecotrs
