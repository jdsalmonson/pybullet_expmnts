'''
Use keyboard to apply torque to various robot joints as it interactively moves.
Motions are logged.

based on ~/stash/bullet3/examples/pybullet/examples/vr_kuka_setup_vrSyncPython.py
'''
import pybullet as p
import pybullet_data
import time
import math
import numpy as np

physId = p.connect(p.SHARED_MEMORY)
if (physId<0):
	p.connect(p.GUI)
p.resetSimulation()

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# add objects:
p.loadURDF("plane.urdf")

kuka = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")[0]
p.loadURDF("tray/tray.urdf", [1, 0, 0])

'''
# kuka = p.loadURDF("kuka_iiwa/model_vr_limits.urdf", 1.4, -0.2, 0.6, 0.0, 0.0, 0.0, 1.0)
kuka = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 1], useFixedBase=True)
p.resetBasePositionAndOrientation(kuka, [0, 0, 0], [0, 0, 0, 1])
'''

'''
#restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
for i, restpose in enumerate(rp):
  p.resetJointState(kuka, i, restpose)

jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
for jointIndex in range(p.getNumJoints(kuka)):
	p.resetJointState(kuka, jointIndex, jointPositions[jointIndex])
	p.setJointMotorControl2(kuka, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)

kuka_gripper = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]

# position kuka_gripper and constrain it to the base kuka robot:
p.resetBasePositionAndOrientation(kuka_gripper, [0.923103, -0.200000, 1.250036],
                                  [-0.000000, 0.964531, -0.000002, -0.263970])

jointPositions = [
    0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000
]
#for jointIndex in range(p.getNumJoints(kuka_gripper)):
  #p.resetJointState(kuka_gripper, jointIndex, jointPositions[jointIndex])
  #p.setJointMotorControl2(kuka_gripper, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)
'''
'''
kuka_cid = p.createConstraint(kuka, 6, kuka_gripper, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05],
                              [0, 0, 0])
'''

p.loadURDF("lego/lego.urdf", -0.5, -0.3, 0.7, 0.0, 0., 0.0, 1.0)
p.loadURDF("lego/lego.urdf", -0.5, -0.3, 0.8, 0.0, 0., 0.0, 1.0)
p.loadURDF("lego/lego.urdf", -0.5, -0.3, 0.9, 0.0, 0., 0.0, 1.0)

for ijenga in range(6):
    p.loadURDF("jenga/jenga.urdf", 0.3 - 0.1*ijenga, 0.7, 0.75, 0.0, 0.707107, 0.0, 0.707107)

'''
table = p.loadURDF("table_square/table_square.urdf", -1.0, 0.0, 0., 0.0,  0., 0.0, 1.0)
jointPositions = [0.000000]
for jointIndex in range(p.getNumJoints(table)):
  p.resetJointState(table, jointIndex, jointPositions[jointIndex])
'''

# get object names:
obj_names = {}
for iobj in range(p.getNumBodies()):
    obj_info = p.getBodyInfo(iobj)
    # MJCF puts name in 1st field, URDF in 2nd field:
    obj_name = obj_info[0] if obj_info[1] == b'physics' else obj_info[1]
    obj_names[iobj] = obj_name.decode('utf8')

print(obj_names)

# from Kuka brochure: https://www.kuka.com/-/media/kuka-downloads/imported/6b77eecacfe542d3b736af377562ecaa/db_lbr_iiwa_en.pdf?rev=013dce5b46bd4137aa21c50f82e912dd&hash=52CF1F3011FC69677750841C4E7DFE4D
maxTorque = {}
maxTorque["J0"] = 320.0 # [Nm]
maxTorque["J1"] = 320.0 # [Nm]
maxTorque["J2"] = 176.0 # [Nm]
maxTorque["J3"] = 176.0 # [Nm]
maxTorque["J4"] = 110.0 # [Nm]
maxTorque["J5"] = 40.0 # [Nm]
maxTorque["J6"] = 40.0 # [Nm]

joint_dict = {}
for jointIndex in range(p.getNumJoints(kuka)):
    joint = p.getJointInfo(kuka, jointIndex)
    #joint[7] = 100. # ? friction force
    #joint[10] = maxTorque[joint[1].decode('utf8')]
    joint_name = joint[1].decode('utf8')

    if joint_name in maxTorque.keys():
	    joint_dict[jointIndex] = { "name" : joint_name,
		                           "torque": maxTorque[joint_name] }

print(obj_names)

# simulate:
p.setGravity(0,0,-9.81)
'''
#show this for 10 seconds
now = time.time()
while (time.time() < now+10):
	p.stepSimulation()
'''

p.setRealTimeSimulation(0)

p.setPhysicsEngineParameter(numSubSteps = 10)
p.setPhysicsEngineParameter(numSolverIterations = 100)
print(p.getPhysicsEngineParameters())

t0 = time.time()
tau = 8 # [seconds]
jointIndex = 4
torque = joint_dict[jointIndex]["torque"]

from dataclasses import dataclass
@dataclass
class KeyCommand:
	key: 'str'
	#ord: 'int'
	joint: 'int'
	torque: 'float'
	def __post_init__(self):
		self.ord = ord(self.key)

key_map = [('j', 0, 10.0),
           ('k', 0, -10.0),
		   ('l', 1, 100.0),
		   (';', 1, -100.0),
		   ('m', 2, 100.0),
		   (',', 2, -100.0),
		   ('.', 3, 100.0),
		   ('/', 3, -100.0),
		   ]
my_keys = [KeyCommand(key, joint, torque) for key, joint, torque in key_map]

p.setJointMotorControl2(kuka, 0, p.VELOCITY_CONTROL, force = 0.0)
p.setJointMotorControl2(kuka, 1, p.VELOCITY_CONTROL, force = 0.0)
p.setJointMotorControl2(kuka, 2, p.VELOCITY_CONTROL, force = 0.0)
p.setJointMotorControl2(kuka, 3, p.VELOCITY_CONTROL, force = 0.0)

logId1 = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
                             "LOG0001.txt",
                             [kuka],
                             logFlags = p.STATE_LOG_JOINT_TORQUES,
                             )

while p.isConnected():
	keys = p.getKeyboardEvents()

	for k in my_keys:
		if k.ord in keys and keys[k.ord] & p.KEY_IS_DOWN: #WAS_TRIGGERED:
			print(f"{k.key} was pressed")
			torque = k.torque
		else:
			torque = 0.
		time.sleep(0.01)
		p.setJointMotorControl2(kuka, k.joint, p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.1)
		p.setJointMotorControl2(kuka, k.joint, p.TORQUE_CONTROL, force=torque)
	'''
	p.setJointMotorControlArray(kuka,
	                            [0,1], #id_revolute_joints,
								p.TORQUE_CONTROL,
								forces=[1., 10.])
	'''
	p.stepSimulation()

	'''
    tm = time.time() - t0

    p.setJointMotorControl2(kuka, jointIndex, p.VELOCITY_CONTROL, targetVelocity = 0, force = 0.0)
    p.setJointMotorControl2(kuka, jointIndex, p.TORQUE_CONTROL, force=0.01*torque*np.sin(2.*np.pi*tm/tau)**3 )
	'''
