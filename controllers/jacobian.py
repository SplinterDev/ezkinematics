import numpy as np

MAX_STEP = 100000000

class JacobianController:
    def __init__(self, arm):
        # the controller should save a reference to the arm it's controlling
        self.arm = arm

    def control(self, target):
        if self.arm.get_num_joints() == 2:
            # if arm has 2 joints
            self.control2J2D(target)
        elif self.arm.get_num_joints() == 3:
            # if arm has 3 joints
            self.control3J2D(target)
        else:
            raise Exception("JacobianController.control(target): Can't control an arm with this amount joints.")

    def control2J2D(self, target):
        # the control method receives a target
        curr_end  = self.arm.endeffector()
        delta_pos = target - curr_end
        # we limit the size of the delta_pos vector as MAX_STEP
        # to avoid making large steps
        if delta_pos.r > MAX_STEP:
            delta_pos.r = MAX_STEP

        # calculate the inverse Jacobian matrix
        iJ = 1/(self.arm.lengths[0]*self.arm.lengths[1]*np.sin(self.arm.thetas[1])) * np.array([
        [self.arm.lengths[1]*np.cos(self.arm.thetas[0] + self.arm.thetas[1]),
         self.arm.lengths[1]*np.sin(self.arm.thetas[0] + self.arm.thetas[1])],
        [-self.arm.lengths[0]*np.cos(self.arm.thetas[0])-self.arm.lengths[1]*np.cos(self.arm.thetas[0]+self.arm.thetas[1]),
         -self.arm.lengths[0]*np.sin(self.arm.thetas[0])-self.arm.lengths[1]*np.sin(self.arm.thetas[0]+self.arm.thetas[1])]
        ])

        # the dot product between iJ and the desired displacement
        # in the end-effector gives us the change necessary in joint values
        delta_joints = iJ.dot(np.array([delta_pos.x, delta_pos.y]))

        # with delta_joints in hand, we move the arm
        self.arm.move( delta_joints )


    def control3J2D(self, target):
        # the control method receives a target
        curr_end  = self.arm.endeffector()
        delta_pos = target - curr_end
        # we limit the size of the delta_pos vector as MAX_STEP
        # to avoid making large steps
        delta_pos.r = MAX_STEP

        l1 = self.arm.lengths[0]
        l2 = self.arm.lengths[1]
        l3 = self.arm.lengths[2]

        theta1 = self.arm.thetas[0]
        theta2 = self.arm.thetas[1]
        theta3 = self.arm.thetas[2]

        a_0_0 = -l1*np.sin(theta1) - l2*np.sin(theta1)*np.cos(theta2) - l2*np.sin(theta2)*np.cos(theta1) + l3*(np.sin(theta1)*np.sin(theta2) - np.cos(theta1)*np.cos(theta2))*np.sin(theta3) + l3*(-np.sin(theta1)*np.cos(theta2) - np.sin(theta2)*np.cos(theta1))*np.cos(theta3)
        a_0_1 = -l2*np.sin(theta1)*np.cos(theta2) - l2*np.sin(theta2)*np.cos(theta1) + l3*(np.sin(theta1)*np.sin(theta2) - np.cos(theta1)*np.cos(theta2))*np.sin(theta3) + l3*(-np.sin(theta1)*np.cos(theta2) - np.sin(theta2)*np.cos(theta1))*np.cos(theta3)
        a_0_2 = -l3*(-np.sin(theta1)*np.sin(theta2) + np.cos(theta1)*np.cos(theta2))*np.sin(theta3) + l3*(-np.sin(theta1)*np.cos(theta2) - np.sin(theta2)*np.cos(theta1))*np.cos(theta3)

        a_1_0 = l1*np.cos(theta1) - l2*np.sin(theta1)*np.sin(theta2) + l2*np.cos(theta1)*np.cos(theta2) + l3*(-np.sin(theta1)*np.sin(theta2) + np.cos(theta1)*np.cos(theta2))*np.cos(theta3) + l3*(-np.sin(theta1)*np.cos(theta2) - np.sin(theta2)*np.cos(theta1))*np.sin(theta3)
        a_1_1 = -l2*np.sin(theta1)*np.sin(theta2) + l2*np.cos(theta1)*np.cos(theta2) + l3*(-np.sin(theta1)*np.sin(theta2) + np.cos(theta1)*np.cos(theta2))*np.cos(theta3) + l3*(-np.sin(theta1)*np.cos(theta2) - np.sin(theta2)*np.cos(theta1))*np.sin(theta3)
        a_1_2 = l3*(-np.sin(theta1)*np.sin(theta2) + np.cos(theta1)*np.cos(theta2))*np.cos(theta3) - l3*(np.sin(theta1)*np.cos(theta2) + np.sin(theta2)*np.cos(theta1))*np.sin(theta3)

        J = np.array([[a_0_0, a_0_1, a_0_2], [a_1_0, a_1_1, a_1_2]])

        iJ = np.linalg.pinv(J)

        #print iJ

        # iJ = 1/(self.arm.l1*self.arm.l2*np.np.np.np.sin(self.arm.t2)) * np.array([
        # [self.arm.l2*np.np.np.np.cos(self.arm.t1 + self.arm.t2),
        #  self.arm.l2*np.np.np.np.sin(self.arm.t1 + self.arm.t2)],
        # [-self.arm.l1*np.np.np.np.cos(self.arm.t1)-self.arm.l2*np.np.np.np.cos(self.arm.t1+self.arm.t2),
        #  -self.arm.l1*np.np.np.np.sin(self.arm.t1)-self.arm.l2*np.np.np.np.sin(self.arm.t1+self.arm.t2)]
        # ])
       
        delta_joints = iJ.dot(np.array([delta_pos.x, delta_pos.y]))
        #print delta_joints

        # self.arm.t1 += delta_joints[0]
        # self.arm.t2 += delta_joints[1]
        self.arm.move(delta_joints)
