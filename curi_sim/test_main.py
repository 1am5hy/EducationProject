# import robots
# import environments
# from mujoco_py import load_model_from_path, MjSim
# import numpy as np
#
# model = load_model_from_path("./description/Curi/model_pla_onegeo.xml")
# sim = MjSim(model=model) # generate sim by modelxml later
# env = environments.Base_env(sim)
# robot = robots.Curi_robot(sim)
#
# jtor = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
#
# while(1):
#     # print(env.sim.data.qacc)
#     jtor = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
#     robot.set_joint_torque(jtor)
#     #jtor = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1])
#     #robot.set_joint_torque(jtor)
#     print("l_arm_joint_pos", robot.l_arm_joint_pos)
#     print('\n')
#     print("r_arm_joint_pos", robot.r_arm_joint_pos)
#     env.sim.step()
#     env.render()
#
#
import numpy as np
import math
from controller import core
# #r
# r1 = np.array([[0,0,1],[0,1,0],[-1,0,0]])
# r2 = np.array([[1,0,0],[0,math.sqrt(2)/2,-math.sqrt(2)/2],[0,math.sqrt(2)/2,math.sqrt(2)/2]])
# r3 = np.array([[math.sqrt(3)/2,-1/2,0],[1/2,math.sqrt(3)/2,0],[0,0,1]])
# r10 = np.array([[math.cos(-math.pi/18),0,math.sin(-math.pi/18)],[0,1,0],[-math.sin(-math.pi/18),0,math.cos(-math.pi/18)]])
# #print(r10.dot(r1.dot(r2).dot(r3)))
#
# r1 = np.array([[0,0,1],[0,1,0],[-1,0,0]])
# r2 = np.array([[1,0,0],[0,math.sqrt(2)/2,math.sqrt(2)/2],[0,-math.sqrt(2)/2,math.sqrt(2)/2]])
# r3 = np.array([[math.sqrt(3)/2,1/2,0],[-1/2,math.sqrt(3)/2,0],[0,0,1]])
# r10 = np.array([[math.cos(-math.pi/18),0,math.sin(-math.pi/18)],[0,1,0],[-math.sin(-math.pi/18),0,math.cos(-math.pi/18)]])

#print(r10.dot(r1.dot(r2).dot(r3)))
r_base_to_franka = np.array([[0.49856585, 0.51624503, 0.69636424, 0.17062],
                             [0.35355339, 0.61237244, -0.70710678, -0.0701],
                             [-0.79147463, 0.59874123, 0.1227878, 1.16864],
                             [0, 0, 0, 1]])
l_base_to_franka = np.array([[0.49856585, -0.51624503, 0.69636424, 0.17062],
                             [-0.35355339, 0.61237244, 0.70710678, 0.0701],
                             [-0.79147463, -0.59874123, 0.1227878, 1.16864]])

Adr = core.Adjoint(r_base_to_franka)
Adl = core.Adjoint(l_base_to_franka)

Slist_old = np.array([[0, 0, 1, 0, 0, 0],
                      [0, 1, 0, -0.333, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, -1, 0, 0.649, 0, -0.0825],
                      [0, 0, 1, 0, 0, 0],
                      [0, -1, 0, 1.033, 0, 0],
                      [0, 0, -1, 0, 0.088, 0]])  # The joint screw axes in the space frame
Slist = Slist_old.copy()
for i in range(7):
    Slist[i] = Adr.dot(Slist_old[i].T)


print(Slist)

#r = np.array([[ 0.49856585,  0.51624503,  0.69636424],
#  [ 0.35355339,  0.61237244, -0.70710678],
#  [-0.79147463,  0.59874123,  0.1227878 ]])
#l = np.array([[ 0.49856585, -0.51624503,  0.69636424],
#  [-0.35355339,  0.61237244,  0.70710678],
#  [-0.79147463, -0.59874123,  0.1227878 ]])