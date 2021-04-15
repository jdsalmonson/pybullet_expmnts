import os
import sys
import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
from time import sleep

import pybullet_data

sys.path.insert(1, os.path.join(os.environ["HOME"], "stash/pybullet_robots"))
import panda_sim

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP,1)
panda = panda_sim.PandaSim(p,[0,0,0])
robot = panda.panda
p.setGravity(0, -9.81, 0)
numJoints = p.getNumJoints(robot)

'''
# Set all joints of robot to their mean position:
for jointIndex in range(p.getNumJoints(robot)):
    joint = p.getJointInfo(robot, jointIndex)
    mean_jointpos = np.mean(joint[8:10])
    p.resetJointState(robot, jointIndex, mean_jointpos)
    p.setJointMotorControl2(robot, jointIndex, p.TORQUE_CONTROL, 0)
    print(f"Joint: {jointIndex} {joint[1].decode('utf8')}, min/max: {joint[8:10]} pos: {joint[-3]} friction: {joint[6]} maxForce: {joint[10]}")
'''

pandaEndEffectorIndex = 7

jd=[0.5]*12

# start logging w/o real-time simulation to ensure logs are in sync:
p.setRealTimeSimulation(0)
logId1 = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
                             "LOG_IK_PANDA_0001.txt",
                             [robot],
                             logFlags = p.STATE_LOG_JOINT_TORQUES)

useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)
t = 0

#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15
hasPrevPose = 0

while 1:
    if (useRealTimeSimulation):
        t = time.time()  #(dt, micro) = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f').split('.')
        #t = (dt.second/60.)*2.*math.pi
    else:
        p.stepSimulation()
        sleep(0.05)
        t = t + 0.001

    # Constant goal position:
    #pos = [0, 0, 1.26]
    #orn = p.getQuaternionFromEuler([0, 0, math.pi])
    # Raster goal position:
    #pos = [-0.4,0.2*math.cos(t),0.+0.2*math.sin(2*t)]
    pos = [0. + 0.2 * math.cos(3*t/2),  # left/right of tray
           0.5 + 0.2 * math.cos(4*t/2), # vertical
           -0.5 + 0.2 * math.sin(t/2),   # out to tray (<0)
           ]
    # end effector angle (down: -pi, horizontal: -pi/2, up: 0
    ee_angle = math.pi
    orn = p.getQuaternionFromEuler([0, ee_angle, 0])

    for i in range(10):
        jointPoses = p.calculateInverseKinematics(robot,
            pandaEndEffectorIndex,
            pos,
            orn,
            jointDamping=jd
            )

    for i in range(len(jointPoses)): #numJoints):
        p.setJointMotorControl2(bodyIndex=robot,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                #targetVelocity=0,
                                force=500,
                                positionGain=0.1, #0.03,
                                velocityGain=1
                                )

    # Create debug trace of end effector:
    ls = p.getLinkState(robot, pandaEndEffectorIndex)
    if (hasPrevPose):
        #p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    #prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1

    #sleep(0.05)
