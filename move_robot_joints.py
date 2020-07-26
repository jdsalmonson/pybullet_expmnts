'''
Move a given robot's joints from min to max, one-at-a-time,
displaying the joint name

From a question on the web: (JDS 7/11/2020)
https://github.com/bulletphysics/bullet3/issues/1056
'''

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from enum import Enum

useRealTime = 0 #1
fixedTimeStep = 0.0001

physId = p.connect(p.SHARED_MEMORY)
if (physId<0):
	p.connect(p.GUI)
p.resetSimulation()

p.setAdditionalSearchPath(pybullet_data.getDataPath())
#p.loadURDF("plane.urdf")
#p.setGravity(0,0,-9.81)
p.setTimeStep(fixedTimeStep)

p.setRealTimeSimulation(0)
#robot = p.loadMJCF("mjcf/ant.xml")[0]
# attempt to change color didn't work:
#for link in range(-1, 20):
#    p.changeVisualShape(1, link, rgbaColor=[0.8, 0.6, 0.4, 1])

# ## robot = p.loadMJCF("mjcf/point.xml")[0]
# robot = p.loadMJCF("mjcf/half_cheetah.xml")[0]
# robot = p.loadMJCF("mjcf/humanoid.xml")[1]
# ## robot = p.loadMJCF("mjcf/humanoidstandup.xml")[1]
# robot = p.loadMJCF("mjcf/inverted_double_pendulum.xml")[0]
# robot = p.loadMJCF("mjcf/inverted_pendulum.xml")[0]
# robot = p.loadMJCF("mjcf/humanoid_symmetric.xml")[0]
# robot = p.loadMJCF("mjcf/hopper.xml")[0]
# robot = p.loadMJCF("mjcf/reacher.xml")[0]
# robot = p.loadMJCF("mjcf/swimmer.xml")[0]
# robot = p.loadMJCF("mjcf/walker2d.xml")[0]
#robot = p.loadURDF("pr2_gripper.urdf", 0.500000, 0.300006, 0.700000, -0.000000, -0.000000, -0.000031, 1.000000)
#robot = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]
#robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
robot = p.loadURDF("/home/jay/catkin_ws/src/stretch_ros/stretch_description/urdf/stretch_main.urdf")

class MoveMode(Enum):
    POSITION = 0  # set joint position directly
    VELOCITY = 1  # set joint velocity
    TORQUE   = 2  # set joint torque

my_move_mode = MoveMode.POSITION

# get object names:
obj_names = {}
for iobj in range(p.getNumBodies()):
    obj_info = p.getBodyInfo(iobj)
    # MJCF puts name in 1st field, URDF in 2nd field:
    obj_name = obj_info[0] if obj_info[1] == b'physics' else obj_info[1]
    obj_names[iobj] = obj_name.decode('utf8')

# Dynamical evolution:
p_gain = 0.8

# Debug text:
textColor = [0, 0, 0.5]
shift = 0.05

# Set all joints of robot to their mean position:
for jointIndex in range(p.getNumJoints(robot)):
    joint = p.getJointInfo(robot, jointIndex)
    mean_jointpos = np.mean(joint[8:10])
    p.resetJointState(robot, jointIndex, mean_jointpos)
    p.setJointMotorControl2(robot, jointIndex, p.TORQUE_CONTROL, 0)
    print(f"Joint: {jointIndex} {joint[1].decode('utf8')}, min/max: {joint[8:10]} pos: {joint[-3]} friction: {joint[6]} maxForce: {joint[10]}")

'''
#show this for 10 seconds
now = time.time()
while (time.time() < now+10):
	p.stepSimulation()
'''

for jointIndex in range(p.getNumJoints(robot)):
    joint = p.getJointInfo(robot, jointIndex)
    if joint[2] == p.JOINT_FIXED:
        continue
    t = 0
    p.removeAllUserDebugItems()
    p.addUserDebugText("Joint: {} {}".format(jointIndex, joint[1].decode('utf8')), [0., 0., 1.5], #[shift, 0, -.1],
                       textColor,
                       parentObjectUniqueId=robot,
                       parentLinkIndex=1)

    max, min = joint[8:10]
    mean = np.mean(joint[8:10])
    amp = 0.5*np.diff(joint[8:10])[0]
    nsteps = 1000
    dt = 1./nsteps
    for istep in range(nsteps):
        joint_pos = mean + amp*np.sin(2.*np.pi*istep/nsteps)

        if my_move_mode == MoveMode.POSITION:
            # Non-dynamical movement:
            p.resetJointState(robot, jointIndex, joint_pos)
        elif my_move_mode == MoveMode.VELOCITY:
            # Dynamical movement:
            p.setJointMotorControl2(robot, jointIndex, p.POSITION_CONTROL, joint_pos, p_gain)
        elif my_move_mode == MoveMode.TORQUE:
            p.setJointMotorControl2(robot, jointIndex, p.VELOCITY_CONTROL, targetVelocity = 0, force = 10.0)
            p.setJointMotorControl2(robot, jointIndex, p.TORQUE_CONTROL, force=300.0*np.sin(2.*np.pi*istep/nsteps)**3 )
            p.stepSimulation()
        t = t + dt
        time.sleep(0.01)

        #print(f"joint_pos = {joint_pos}, {p.getJointState(robot, jointIndex)[0]}")
