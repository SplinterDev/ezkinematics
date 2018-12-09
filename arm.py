from point2d import Point2D
import math

PI = math.pi
MAX_THETA = PI
MIN_THETA = -PI

class Arm:
    def __init__(self, lengths=[200, 200], thetas=[0, PI/2.]):
        """
        Receives:
            lengths: list of links' lengths (default: 2 links of 200mm)
            thetas: list of initial angles of joints (default: 2 joints -> 0 and pi/2)

        lengths and thetas should be lists of the same size
        """
        if not len(lengths) == len(thetas):
            raise Exception("Arm(): lengths and thetas should be lists of same size.")
        self.lengths = lengths
        self.thetas = thetas
        self.num_joints = len(thetas)

    def get_num_joints(self):
        return self.num_joints

    def move(self, dt):
        """
        Receives:
            dt: list of delta theta changes in joint values
        """
        if not len(dt) == self.num_joints:
            raise Exception("Arm.move(dt): dt, i.e, Delta theta, should have {} elements to move this arm.".format(self.num_joints))
        for i, delta in enumerate(dt):
            # moves it
            self.thetas[i] += delta
            # keeps its value between -pi and pi
            if self.thetas[i] > MAX_THETA:
                self.thetas[i] -= 2*PI
            elif self.thetas[i] < MIN_THETA:
                self.thetas[i] += 2*PI

    def endeffector(self):
        """
        Finds end-effector position through forward kinematics and returns it
        """
        
        return self.joints_pos()[-1]

    def joints_pos(self):
        """
        Returns the position of every joint in the space (useful for drawing them)
        """
        points = [Point2D() for i in self.thetas]

        for i, p in enumerate(points):
            p.r = self.lengths[i]
            p.a = sum(self.thetas[:i+1])
            if i > 0:
                p += points[i-1]
        return points
        
    def error(self, target):
        """
        Returns the euclidian distance between the endeffector and the target
        """
        return (self.endeffector() - target).r

if __name__ == '__main__':
    a = Arm()
    print(a.endeffector())



