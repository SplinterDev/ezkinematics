from ezgame import Ezgame
from point2d import Point2D

from arm import *
from incrementer import IncrementerController

import time

FPS = 30


# APPLICATION MODES
JACOBIAN    = 0
INCREMENTER = 1

CURRENT_MODE = INCREMENTER
# to avoid creating a new Point2D as (0,0) every loop, reuse this one:
ORIGIN = Point2D()

class Application:
    def __init__(self):
        # creates an ezgame application with size (800x800)
        # and sets its fps to FPS
        self.ez = Ezgame(800, 800)
        self.ez.fps = FPS
        # self.loop identifies what loop function will be used
        self.loop = CURRENT_MODE

        # for starters, creates 3 links and 3 joints
        lengths = [200]*3
        thetas  = [0, 0, math.pi/2.]
        # and uses them to create an arm
        self.arm = Arm(lengths, thetas)
        self.init()

    def init(self):
        # for each mode, add one if statement here and create a loop method
        # using self.loop, the program initializes a bunch of stuff
        if self.loop == INCREMENTER:
            self.ez.init(self.incrementer_loop)
            # self.ez.gui = True
            self.controller = IncrementerController(self.arm)
        else:
            self.exit()

    def incrementer_loop(self):
        # in this mode, we draw the arm
        self.draw_arm(self.arm)
        # paint the end-effector blue
        self.ez.point(self.arm.endeffector(), color='blue')
        # and let the controller control the arm
        self.controller.control(ORIGIN)

    def draw_arm(self, arm, color='black'):
        # to draw the arm we get the position of each joint
        points = arm.joints_pos()
        for i, p in enumerate(points):
            # links joints with lines
            if i == 0:
                self.ez.line(ORIGIN, p, color=color)
            else:
                self.ez.line(points[i-1], p, color=color)
            # draws the joint as a point
            self.ez.point(p)

    def run(self):
        # this only calls the ezgame run() method, dont worry about it
        self.ez.run()

    def exit(self):
        exit()


if __name__ == '__main__':
    app = Application()
    app.run()