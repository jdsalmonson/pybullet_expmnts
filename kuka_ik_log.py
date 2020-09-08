import pybullet as p
import time
import math
import random
from datetime import datetime
from time import sleep

import pybullet_data

x0, y0, z0 = [random.uniform(0, 0.5) for _ in range(3)]
a_x, a_y, a_z = [random.uniform(0, 0.2) for _ in range(3)]
w_x, w_y, w_z = [random.uniform(-4, 4) for _ in range(3)]

log_file = "logs/kuka_ik_0011.slog"

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -0.3])
kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)

#Joint damping coefficents. Using large values for the joints that we don't want to move.
#jd = [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 0.5]
jd=[0.5,0.5,0.5,0.5,0.5,0.5,0.5]

p.setGravity(0, 0, -9.81) #0)

useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)
t = 0

#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15
hasPrevPose = 0

logId1 = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
                             log_file,
                             [kukaId],
                             logFlags = p.STATE_LOG_JOINT_TORQUES,
                             )

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
    '''
    pos = [-0.5 + 0.2 * math.cos(3*t),
           0.1 * math.cos(4*t),
           0. + 0.2 * math.sin(t) + 0.7
           ]
    '''
    pos = [-x0 + a_x * math.cos(w_x*t),
           y0 + a_y * math.cos(w_y*t),
           z0 + a_z * math.cos(w_z*t),
           ]
    # end effector angle (down: -pi, horizontal: -pi/2, up: 0
    ee_angle = -math.pi / 2
    orn = p.getQuaternionFromEuler([0, ee_angle, 0])

    for i in range(10):

        jointPoses = p.calculateInverseKinematics(kukaId,
            kukaEndEffectorIndex,
            pos,
            orn,
            jointDamping=jd
            )

    for i in range(numJoints):
        p.setJointMotorControl2(bodyIndex=kukaId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                #targetVelocity=0,
                                force=500,
                                positionGain=0.03,
                                velocityGain=1
                                )

    # Create debug trace of end effector:
    ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
    if (hasPrevPose):
        #p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    #prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1

    #sleep(0.05)
