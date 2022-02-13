# coding=utf-8
# Import
import numpy as np
import math as m
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# CHANGE kat_obrotu -> rot_ang


# --------------------------------------------------------------
# GLOBAL FUNCTION
# ---------------------------
def rot_mat(w=0, p=0, r=0):
    """
    Change angles of rotation on rotation matrix by euler rotations
    :param w: Angle over X axis in deg default 0 deg
    :param p: Angle over Y axis in deg default 0 deg
    :param r: Angle over Z axis in deg default 0 deg
    :return: Euler rotation matrix
    """
    w = m.radians(w)
    p = m.radians(p)
    r = m.radians(r)
    RX = np.array([[1, 0, 0], [0, m.cos(w), -m.sin(w)], [0, m.sin(w), m.cos(w)]])
    RY = np.array([[m.cos(p), 0, m.sin(p)], [0, 1, 0], [-m.sin(p), 0, m.cos(p)]])
    RZ = np.array([[m.cos(r), -m.sin(r), 0], [m.sin(r), m.cos(r), 0], [0, 0, 1]])
    RXYZ = RX @ RY @ RZ
    return RXYZ


# ---------------------------
def rot_ang(RXYZ):
    """
    Change rotation matrix to rotation angles in deg
    :param RXYZ: Rotation matrix -> Matrix[3,3]
    :return: Tuple with rotation angles (W,P,R) in deg
    """
    Pp = m.degrees(m.asin(RXYZ[0, 2]))
    Rp = m.degrees(m.acos(round(RXYZ[0, 0] / m.cos(m.radians(Pp)), 5)))
    Wp = m.degrees(m.acos(round(RXYZ[2, 2] / m.cos(m.radians(Pp)), 5)))
    return (Wp, Pp, Rp)


# ---------------------------
def vec_bis(V1, V2):
    """
    Bisector between two vectors
    :param V1: Vector one
    :param V2: Vector two
    :return: Bisector between V1 and V2
    """
    V1 = np.reshape(V1, [V1.size, 1])
    V2 = np.reshape(V2, [V2.size, 1])
    V1 = vec_norm(V1)
    V2 = vec_norm(V2)
    return (vec_norm(V1 + V2))


# ---------------------------
def vec_norm(V):
    """
    Return normalized vector V
    :param V: vector to normalization
    :return: normalized vector V
    """
    V = vec_round(V)
    return np.divide(V, np.abs(np.linalg.norm(V)))


# ---------------------------
def vec_round(V):
    """
    Rounded vector V -> reduce calculation confidence
    All elements  <-0.0001,0.0001> ale set to 0.000
    :param V: vector to be rounded
    :return: rounded vector
    """
    for i in range(V.size):
        if 0.0001 >= V[i] >= -0.0001:
            V[i] = 0.0
    return V


# ---------------------------
def perpendicular_plane(v):
    """
    Creating matrix with two basic vector of plane perpendicular to vector v
    :param v: vector for
    :return: plane perpendicular to vector v
    """
    # Two random vectors
    t1 = np.random.rand(3, 1)
    t2 = np.random.rand(3, 1)

    # Making vector perpendicular to Vector v
    if v[2] != 0:
        t1[2] = -(v[0] * t1[0] + v[1] * t1[1]) / v[2]
        t2[2] = -(v[0] * t2[0] + v[1] * t2[1]) / v[2]
    elif v[1] != 0:
        t1[1] = -(v[0] * t1[0] + v[2] * t1[2]) / v[1]
        t2[1] = -(v[0] * t2[0] + v[2] * t2[2]) / v[1]
    else:
        t1[0] = -(v[2] * t1[2] + v[1] * t1[1]) / v[0]
        t2[0] = -(v[2] * t2[2] + v[1] * t2[1]) / v[0]

    # Matrix witch have to vector which create 2D plane i 3D
    A = np.concatenate((t1, t2), axis=1)
    return A


# ---------------------------
def vec_ang(v1, v2):
    """
    Returns angle between two vector
    :param v1: Vector one
    :param v2: Vector two
    :return: angle in RAD
    """
    dot = v1[0] * v2[0] + v1[1] * v2[1]
    det = v1[0] * v2[1] - v1[1] * v2[0]
    kat = m.atan2(det, dot)
    return kat


# small_pipe_radius = 100
# Coordination of pipe axis cross section [ X, Y, Z ]
# pipe_axis_coordination = np.array([ -1700, 1095 , -330])

class GlobalVars:
    def __init__(self):
        # Start vector i^
        self.i_start = np.reshape([1, 0, 0], [3, 1])
        # Start vector j^
        self.j_start = np.reshape([0, 1, 0], [3, 1])
        # Start vector k^
        self.k_start = np.reshape([0, 0, 1], [3, 1])
        # Init system of coordination
        self.init_coord = np.concatenate((self.i_start, self.j_start, self.k_start), axis=1)


class PipeInit(GlobalVars):

    def __init__(self, pipe_radius, pipe_coordination, points, direction):
        super().__init__()
        # Pipe radius
        self.pipe_radius = pipe_radius
        # Pipe axis coordination
        self.pipe_coordination = pipe_coordination
        # Total point in path
        self.total_points = points * 4
        # Direction of torch
        self.direction = direction

        # Create sign matrix
        self.sign_matrix = np.zeros([self.total_points, 2])
        self.sign_calc()

        # Create position matrix
        self.position_matrix = np.zeros([self.total_points, 3])
        self.pos_calc()

    def sign_calc(self, ):
        # Create sign matrix
        for i in range(0, self.total_points):
            self.sign_matrix[i][0] = m.pow(-1, m.floor(i / ((self.total_points / 4) * 2)))
            self.sign_matrix[i][1] = m.pow(-1, m.floor(i / (self.total_points / 4))) * self.sign_matrix[i][0]

    def pos_calc(self, ):
        """
        Counting cross section path by formula from :
        https://www.obliczeniowo.com.pl/267
        :return:
        """

        # Maximum radius of cross section circle
        max_radius = m.sqrt(m.pow(big_pipe_radius, 2) + m.pow(self.pipe_radius, 2))
        # Delta of crossing circle radius
        delta = (max_radius - big_pipe_radius) / (self.total_points / 4)
        # Crossing circles
        circles = np.zeros([self.total_points, 1])
        circles[0, 0] = big_pipe_radius
        for i in range(1, self.total_points):
            circles[i, 0] = circles[i - 1, 0] + delta * self.sign_matrix[i - 1, 0] * self.sign_matrix[i - 1, 1]

        # Position counting
        x = np.reshape(list(
            map(lambda s, w: (m.sqrt(abs(m.pow(w, 2) - m.pow(big_pipe_radius, 2)))) * s, self.sign_matrix[:, 0],
                circles)), [self.total_points, 1])
        z = np.reshape(list(map(lambda w: (m.sqrt(abs(m.pow(w, 2) - m.pow(self.pipe_radius, 2)))), circles)),
                       [self.total_points, 1])
        y = np.reshape(list(
            map(lambda s, w: (m.sqrt(abs(m.pow(big_pipe_radius, 2) - m.pow(w, 2)))) * s, self.sign_matrix[:, 1], z)),
            [self.total_points, 1])

        self.position_matrix = np.concatenate((x, y, z), axis=1)


class Pipe(PipeInit):
    def __init__(self, pipe_radius, pipe_coordination, points, direction):
        super().__init__(pipe_radius, pipe_coordination, points, direction)
        self.XYZ_matrix = self.position_matrix
        self.WPR_matrix = self.orient_calc()
        self.XYZ_matrix_export = self.XYZ_matrix
        self.export_prepare()

    def change_start(self, shift):
        """
        Shifts points in position matrix
        :param shift:
        :return:
        """
        self.XYZ_matrix = np.roll(self.position_matrix, -shift, axis=0)

    def \
            vec_tangent(self, i):
        """
        Return vector tangent to path
        :param i: item of path
        :return: vector tangent to i-th item of path
        """
        # indicator of point before and point after i point ip - i plus , im - i minus
        ip = i + 1 if i + 1 < self.total_points else 0
        im = i - 1 if i - 1 < self.total_points else self.total_points

        # Vector plus
        vp = np.reshape(self.XYZ_matrix[i] - self.XYZ_matrix[ip], [3, 1])
        # Vector minus
        vm = np.reshape(self.XYZ_matrix[im] - self.XYZ_matrix[i], [3, 1])
        vp = vec_norm(vp)
        vm = vec_norm(vm)
        v = vec_norm(np.divide(vp + vm, 2))
        return v if self.direction[1] == '-' else -v

    def vec_tangent_2_ellipse(self, i):
        """
        Function return vector tangent to ellipse created by
        intersection of the cylinder with a plane at the i-th point of path
        :param i: number of path element
        :return: vector tangent to i-th element of patch
        """
        # Vector from origin to i-th point of the PATH
        v0 = np.reshape(self.XYZ_matrix[i], [3, 1])

        # Shift of the ellipse origin on 2D plane a -> horizontal, b -> vertical
        a = -m.sqrt(m.pow(v0[0], 2) + m.pow(v0[1], 2))
        b = float(-v0[2])

        # Normalize vector v0
        v0 = vec_norm(v0)

        # Vector with 0 angle rotation
        v1 = np.array([0, 1])
        # Vector v0 on plane XY
        v2 = np.array([v0[0], v0[1]])

        # Angle of section plane
        alpha = vec_ang(v2, v1)
        # Cosines and Sinus of plane rotation angle
        cos = m.cos(alpha)
        sin = m.sin(alpha)

        # Random Z coordinate to calculate y coordinate
        z = -100

        # y coordinate correspond to above z which create line tangent to
        # Ellipse created by cross plane on cylinder equation
        y = ((-m.pow(b, 2) + m.pow(big_pipe_radius, 2) + b * z) / (-a * m.pow(cos, 2))) - a

        # Created vector by distribution y coordinate on plane XY
        temp = np.array([(y * sin), (y * cos), z])

        # Normalize temp vector
        temp = vec_norm(temp)

        # Count bisector of k^ and temp vector and normalize it
        return vec_norm(vec_bis(self.k_start, temp))

    def export_prepare(self):
        #TODO: OPIS
        self.XYZ_matrix_export =  self.XYZ_matrix + self.pipe_coordination

    def orient_calc(self):
        #TODO: OPIS + komentarze

        # Prepare matrix for matrix orientation
        WPR = np.zeros([self.total_points, 3])
        for i in range(0, self.total_points):
            # Vector ax 1
            ax1 = self.vec_tangent(i)
            # Vector k^
            k_end = self.vec_tangent_2_ellipse(i)
            A = perpendicular_plane(ax1)
            # TODO: change names
            w = np.linalg.inv(A.T @ A) @ A.T @ k_end
            k_end = vec_norm(A @ w)

            ax2 = np.cross(np.reshape(ax1, [1, 3]), np.reshape(k_end, [1, 3]))
            ax2 = ax2[0]

            k_end = np.reshape(k_end, [3, 1])
            ax1 = np.reshape(ax1, [3, 1])
            ax2 = np.reshape(vec_norm(ax2), [3, 1])

            # Stworzenie Macierzy z wersorami i^,j^,k^
            R1 = np.concatenate((self.i_start, self.j_start, self.k_start), axis=1)

            R2 = np.concatenate((ax1, -ax2, k_end), axis=1) if self.direction[0] == "X" \
                else np.concatenate((ax2, ax1, k_end), axis=1)
            # Stworzenie macierzy rotacji
            R = R1 @ np.linalg.inv(R2)

            # -------------------------------------------------------------------------
            # WYŚWIETLANIE WYNIKÓW KONTROLNE
            if i == 121120:
                T = R
                fig = plt.figure()
                ax = fig.gca(projection='3d')
                ax.plot([0, k_end[0]], [0, k_end[1]], [0, k_end[2]], 'b--')
                ax.plot([0, ax1[0]], [0, ax1[1]], [0, ax1[2]], 'g--')
                ax.plot([0, ax2[0]], [0, ax2[1]], [0, ax2[2]], 'r--')
                ax.plot([0, R1[0, 0]], [0, R1[0, 1]], [0, R1[0, 2]], 'k-.')
                ax.plot([0, R1[1, 0]], [0, R1[1, 1]], [0, R1[1, 2]], 'k--')
                ax.plot([0, R1[2, 0]], [0, R1[2, 1]], [0, R1[2, 2]], 'k')
                ax.view_init(90, 90)
                plt.show()
                # print(p12)
            # ----------------------------------------------------------------------------

            # Obliczenie poczegolnych katow obrotu
            p = m.degrees(m.asin(R[0, 2]))
            r = m.degrees(m.atan2(-R[0][1], R[0][0]))
            w = m.degrees(m.atan2(-R[1][2], R[2][2]))
            # wpisanie ich do macierzy obrotów
            WPR[i] = np.array([-w, -p, -r])
        return WPR


# --------------------------------------------------------------
# Input data
# Radius of bigger pipe which have ax in positioner rotation ax.


big_pipe_radius = 37.3

p_1 = Pipe(12.5, [-2670.666, -144.210, 1022.408], 3, "X+")
#p_2 = Pipe(50, [1300, 1000, 1000], 4, "Y-")
#p_3 = Pipe(20, [1100, 1000, 1000], 4, "X+")
exp_data1 = np.concatenate((p_1.XYZ_matrix_export, p_1.WPR_matrix), axis=1)
#exp_data2 = np.concatenate((p_2.XYZ_matrix_export, p_2.WPR_matrix), axis=1)
#exp_data3 = np.concatenate((p_3.XYZ_matrix_export, p_3.WPR_matrix), axis=1)
#exp_data = np.concatenate((exp_data1, exp_data2, exp_data3), axis=0)
np.savetxt("XP_BOLEC.csv", exp_data1, delimiter=",")
#np.savetxt("C:\\Users\\J.Motak\\Desktop\\foo.txt",p_1.XYZ_matrix_export , delimiter=",")
# TODO: DOROBIC KOORDYNACJE i DOSTOSOWYWANIE KĄTA