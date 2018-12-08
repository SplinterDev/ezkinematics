from point2d import Point2D
import math

PI = math.pi
MAX_THETA = PI
MIN_THETA = -PI

class Arm:
    def __init__(self, l1=200, l2=200, l3=200, t1=0.0, t2=0., t3=PI/2.):
        self.t1 = t1
        self.t2 = t2
        self.t3 = t3
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    def move(self, dt1, dt2, dt3):
        self.t1 += dt1
        self.t2 += dt2
        self.t3 += dt3

        if self.t1 > MAX_THETA:
            self.t1 -= 2*PI
        if self.t2 > MAX_THETA:
            self.t2 -= 2*PI
        if self.t3 > MAX_THETA:
            self.t3 -= 2*PI

        if self.t1 < MIN_THETA:
            self.t1 += 2*PI
        if self.t2 < MIN_THETA:
            self.t2 += 2*PI
        if self.t3 < MIN_THETA:
            self.t3 += 2*PI

    def endeffector(self):
        p1 = Point2D(r=self.l1, a=self.t1)
        p2 = Point2D(r=self.l2, a=self.t2+self.t1) + p1
        return Point2D(r=self.l3, a=self.t3+self.t2+self.t1) + p2

if __name__ == '__main__':
    a = Arm()
    print(a.endeffector())


    # THIS CODE WAS TAKEN FROM INSIDE THE CLASS
    # IM GONNA JUDGE IF IT'S NEEDED AT ANOTHER TIME
    # def set_random_pose(self):
    #     t1 = scale(0.0, 1.0, MIN_THETA, MAX_THETA, np.random.random())
    #     t2 = scale(0.0, 1.0, MIN_THETA, MAX_THETA, np.random.random())
    #     t3 = scale(0.0, 1.0, MIN_THETA, MAX_THETA, np.random.random())
    #     # check if the values are between MIN_THETA and MAX_THETA
    #     assert t1 > MIN_THETA and t1 < MAX_THETA
    #     assert t2 > MIN_THETA and t2 < MAX_THETA
    #     assert t3 > MIN_THETA and t3 < MAX_THETA
    #     # if they are, set joints to the values
    #     self.t1 = t1
    #     self.t2 = t2
    #     self.t3 = t3

    # def get_normalized_pose(self):
    #     t1 = scale(MIN_THETA, MAX_THETA, 0.0, 1.0, self.t1)
    #     t2 = scale(MIN_THETA, MAX_THETA, 0.0, 1.0, self.t2)
    #     t3 = scale(MIN_THETA, MAX_THETA, 0.0, 1.0, self.t3)
    #     # check if the values are between 0.0 and 1.0
    #     assert t1 > 0.0 and t1 < 1.0
    #     assert t2 > 0.0 and t2 < 1.0
    #     assert t3 > 0.0 and t3 < 1.0
    #     return t1, t2, t3

    # def set_normalized_pose(self, t1, t2, t3):
    #     t1 = scale(0.0, 1.0, MIN_THETA, MAX_THETA, t1)
    #     t2 = scale(0.0, 1.0, MIN_THETA, MAX_THETA, t2)
    #     t3 = scale(0.0, 1.0, MIN_THETA, MAX_THETA, t3)
    #     # check if the values are between MIN_THETA and MAX_THETA
    #     assert t1 > MIN_THETA and t1 < MAX_THETA
    #     assert t2 > MIN_THETA and t2 < MAX_THETA
    #     assert t3 > MIN_THETA and t3 < MAX_THETA
    #     # if they are, set joints to the values
    #     self.t1 = t1
    #     self.t2 = t2
    #     self.t3 = t3


    # def error(self, target):
    #     # returns the euclidian distance between the endeffector and the target
    #     return (self.endeffector() - target).r
