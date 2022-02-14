class Point():
    def __init__(self, x, y, z, rotation_matrix = None):
        self.x = x
        self.y = y
        self.z = z
        self.rotation_matrix = rotation_matrix
    def to_string(self):
        return f'X = {self.x}\nY = {self.y}\nZ = {self.z}/n'