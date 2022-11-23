import robots
import environments
from mujoco_py import load_model_from_path, MjSim
import numpy as np

model = load_model_from_path("./description/Curi/model_pla_onegeo.xml")
sim = MjSim(model=model) # generate sim by modelxml later
env = environments.Base_env(sim)
robot = robots.Curi_robot(sim)

jtor = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])


while(1):
    # print(env.sim.data.qacc)
    jtor = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.set_joint_torque(jtor)
    #jtor = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1])
    robot.set_joint_torque(jtor)
    print("l_arm_joint_pos", robot.l_arm_joint_pos)
    print('\n')
    print("r_arm_joint_pos", robot.r_arm_joint_pos)
    env.sim.step()
    env.render()


