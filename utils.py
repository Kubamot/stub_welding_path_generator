import math
import numpy as np


class Point():
    def __init__(self, x, y, z, rotation_matrix=None):
        self._pose = np.array([x, y, z])
        self.rotation_matrix = rotation_matrix

    def to_string(self):
        return f'X = {self._pose[0]}\nY = {self.y}\nZ = {self.z}/n'

    @property
    def pose(self):
        return self._pose

    @property
    def x(self):
        return self._pose[0]

    @property
    def y(self):
        return self._pose[1]

    @property
    def z(self):
        return self._pose[2]


def unit_vector(vector):
    vector_rounded = vector.round(3)
    return np.divide(vector_rounded, np.abs(np.linalg.norm(vector_rounded)))

def angle_between_vectors(vector_1, vector_2):

    dot_product = vector_1[0] * vector_2[0] + vector_1[1] * vector_2[1]
    vector_det = vector_1[0] * vector_2[1] - vector_1[1] * vector_2[0]
    x = math.atan2(vector_det, dot_product)
    
    y = np.dot(unit_vector(vector_1), unit_vector(vector_2))
    return math.atan2(vector_det, dot_product)

def vectors_bisector(vector_1, vector_2):
    vector_1 = unit_vector(vector_1)
    vector_2 = unit_vector(vector_2)
    return unit_vector(np.sum(vector_1, vector_2))

def project_vector_to_plane(vector, plane):
    plane = perpendicular_plane(ax1)
    w = np.linalg.inv(plane.T @ plane) @ plane.T @ vector
    return unit_vector(plane @ w)

def plane_normal_to_vector(vec):

    t1 = np.random.rand(3, 1)
    t2 = np.random.rand(3, 1)

    # Making vector perpendicular to Vector vec
    if vec[2] != 0:
        t1[2] = -(vec[0] * t1[0] + vec[1] * t1[1]) / vec[2]
        t2[2] = -(vec[0] * t2[0] + vec[1] * t2[1]) / vec[2]
    elif vec[1] != 0:
        t1[1] = -(vec[0] * t1[0] + vec[2] * t1[2]) / vec[1]
        t2[1] = -(vec[0] * t2[0] + vec[2] * t2[2]) / vec[1]
    else:
        t1[0] = -(vec[2] * t1[2] + vec[1] * t1[1]) / vec[0]
        t2[0] = -(vec[2] * t2[2] + vec[1] * t2[1]) / vec[0]

    # Matrix witch have to vector which create 2D plane i 3D
    return np.concatenate((t1, t2), axis=1)
