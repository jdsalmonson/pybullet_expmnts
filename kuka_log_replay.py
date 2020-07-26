import pybullet as p
import struct
from collections import defaultdict

from robotlog import RobotLog

log_file_name = "LOG_IK_0001.txt"
bot_log = RobotLog(log_file_name).log

#clid = p.connect(p.SHARED_MEMORY)
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
kuka = p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")[0]
#p.resetBasePositionAndOrientation(kuka, [0, 0, 0], [0, 0, 0, 1])
p.loadURDF("tray/tray.urdf", [1, 0, 0])
p.loadURDF("block.urdf", [0, 0, 2])

recordNum = len(bot_log[1]['stepCount'])
'''
recordNum = len(log)
itemNum = len(log[0])
objectNum = p.getNumBodies()
'''

def Step(stepIndex, blog, botId):
  pos = []
  for posi in ['posX', 'posY', 'posZ']:
    pos.append(blog[posi][stepIndex])
  orn = []
  for orni in ['oriX', 'oriY', 'oriZ', 'oriW']:
    orn.append(blog[orni][stepIndex])

  p.resetBasePositionAndOrientation(botId, pos, orn)

  numJoints = p.getNumJoints(botId)
  numJointsLog = blog['qNum'][0]
  # Only use the joints available both source and destination bot:
  numJoints = min(numJoints, numJointsLog)
  for ijnt in range(numJoints):
    #jointInfo = p.getJointInfo(botId, ijnt)
    #qIndex = jointInfo[3]
    qn = 'q' + str(ijnt) # joint position field: q#
    un = 'u' + str(ijnt) # joint velocity field: u#
    p.resetJointState(botId, ijnt, blog[qn][stepIndex])

stepIndexId = p.addUserDebugParameter("stepIndex", 0, recordNum - 1, 0) #recordNum / objectNum - 1, 0)

Step(stepIndexId, bot_log[1], kuka)
while True:
  stepIndex = int(p.readUserDebugParameter(stepIndexId))
  Step(stepIndex, bot_log[1], kuka)
  p.stepSimulation()
  Step(stepIndex, bot_log[1], kuka)
