import math

class IncrementerController:
    def __init__(self, arm):
        # the controller should save a reference to the arm it's controlling
        self.arm = arm

    def control(self, target):
        # the control method receives a target

        # This specific controller ignores the target
        # It only increments each joint in 1 degree

        # the arm.move method receives a list of joint values increments
        # the list should have as many elements as the arm has joints
        # you can find out how many joints the arm has via arm.get_num_joints()
        # for this example:
        # pi/180 = 1 degree in radians
        # [element] * 3 => [element, element, element]
        self.arm.move( [math.pi/180.]*self.arm.get_num_joints() )