from point2d import Point2D
import math

TRAJ_CENTER = Point2D(300, 200)


TRAJ_STEP_MIN = 0.1
TRAJ_STEP_MAX = 30.
TRAJ_STEP_INC = 0.1


MIN_STEP = 0.1
MAX_STEP = 30.0

DT_VAL = 5
MIN_DT = math.radians(-DT_VAL)
MAX_DT = math.radians(DT_VAL)


class Trajectory:
    def __init__(self, step_length=0.1, radius=100, points_count=1000, center=TRAJ_CENTER, restart=None):
        self.points  = []
        self.counter = 0
        self.center  = center
        self.radius  = radius
        self.step_length  = step_length
        self.points_count = points_count
        self.restart_function = restart

        self.generate()

    def generate(self):
        p = Point2D(self.radius, 0)
        angle_step = 2*math.asin(self.step_length/(2.*self.radius))
        while len(self.points) < self.points_count:
            self.points.append(Point2D(self.center+p))
            p.a += angle_step

    def current(self):
        return self.points[self.counter]

    def curr_idx(self):
        return self.counter

    def next(self):
        point = self.points[self.counter]
        new_counter = self.counter+1
        if new_counter > len(self.points)-1:
            new_counter = 0
            if self.restart_function:
                self.restart_function()

        self.counter = new_counter
        return point

if __name__ == '__main__':
    print(Trajectory(points_count=5).points)