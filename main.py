from pipe import Pipe

Pipe_1 = Pipe(100, 500)
Pipe_1.add_stub(80, 0, 0, 20)
Pipe_1.add_stub(80, 0, 0, 16)
Pipe_1.add_stub(30, 0, 0, 12)
Pipe_1.print_path_points()