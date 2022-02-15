from stub import Stub

class Pipe():

    def __init__(self, radius, length):

        self._radius = radius
        self._length = length
        self._stubs = []
    
    def add_stub(self, radius, axis_offset, axis_rotation, path_points):
        self._stubs.append(Stub(radius, axis_offset, axis_rotation, path_points))
    
    def print_path_points(self):
        for stub in self._stubs:
            stub.create_path_points(self._radius)
            stub.count_rot()


    