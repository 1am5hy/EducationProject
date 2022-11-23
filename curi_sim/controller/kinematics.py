from . import core
import numpy as np


class Kinematics(object):
    def __init__(self, robot):
        '''
        :param robot (robot_mujoco.Robot): instance of class Robot
        '''
        self.robot = robot

    def fk(self, joint_pos):
        return core.FKinSpace(self.robot.initial_pose, self.robot.screw, joint_pos)

    def JacobianSpace(self):
        return core.JacobianSpace(self.robot.screw,self.robot.joint_pos)

    def ik(self, target):
        thetalist = np.array([0, 0, 0, 0, 0, 0, 0])
        eomg = 0.01
        ev = 0.001
        j_pos_target = core.IKinSpace(self.robot.screw, self.robot.initial_pose, target, thetalist, eomg, ev)
        j_pos_target = np.array(j_pos_target[0])
        # j_pos_now = self.robot.joint_pos
        # print(j_pos_target)
        # print(j_pos_now)
        # torque = (j_pos_target-j_pos_now)*50
        # print(torque)
        # self.robot.set_joint_torque(torque)
        return j_pos_target

