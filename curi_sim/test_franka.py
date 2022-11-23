import robots
import environments
from mujoco_py import load_model_from_path, MjSim
import numpy as np
from controller import Dynamics, CuriJointPositionImpedanceGenerator, JointPositionImpedanceGenerator

model = load_model_from_path("./description/Franka/dynamic_franka_without_gripper.xml")
sim = MjSim(model=model) # generate sim by modelxml later
env = environments.Base_env(sim)
robot = robots.Franka_robot(sim)
dynamics = Dynamics(robot)
joint_controller = JointPositionImpedanceGenerator(1,0.1,robot)
target = np.array([ -0.93,-0.95, 0.803,-2.41,  -0.13,2.124, 0.935])

joint_controller.set_target(target)

while(1):
    #print(env.sim.data.qacc)
    # target = np.array([-0.93, -0.95, 0.803, -2.41, -0.13, 2.124, 0.935])
    # jtor, su_flag = joint_controller.impedance_controller_joint()
    # robot.set_rarm_joint_torque(jtor)
    # # print(jtor)
    theta = robot.joint_pos
    #print(dynamics.Quadraticforces(theta,robot.r_arm_joint_vel))
   # jtor2 = dynamics.gravityforces(theta)+dynamics.Quadraticforces(theta,robot.joint_vel)
    print('true',robot.torque_compensation)
   # print('com',jtor2)
    robot.set_joint_torque(1.2*robot.torque_compensation)
    env.sim.step()
    env.render()



# while(1):
#     #print(env.sim.data.qacc)
#     #jtor = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
#     theta = robot.joint_pos
#
#     target = np.array([-0.93, -0.95, 0.803, -2.41, -0.13, 2.124, 0.935])
#     joint_controller.set_target(target)
#     while True:
#         jtor,su_flag = joint_controller.impedance_controller_joint()
#         robot.set_joint_torque(jtor)
#         env.sim.step()
#         env.render()
#         if su_flag:
#             break
#
#     target = np.array([-0., -0., 0., -0, -0, 0, 0])
#     joint_controller.set_target(target)
#     while True:
#         jtor,su_flag = joint_controller.impedance_controller_joint()
#         robot.set_joint_torque(jtor)
#         #print(jtor)
#         env.sim.step()
#         env.render()
#         if su_flag:
#             break
#
#     target = np.array([-0.93, -0.95, 0.803, -2.41, -0.13, 2.124, 0.935])
#     joint_controller.set_target(target)
#     while True:
#         jtor,su_flag = joint_controller.impedance_controller_joint()
#         robot.set_joint_torque(jtor)
#         env.sim.step()
#         env.render()
#         if su_flag:
#             break
#     env.render()


