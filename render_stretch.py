'''
Render a bunch of frames flying around the Stretch robot as it moves its joints.
'''
import numpy as np
import matplotlib.pyplot as plt
import pybullet
import time
import pybullet_data

p = pybullet

plt.ion()

img = np.random.rand(200, 320)
#img = [tandard_normal((50,100))
image = plt.imshow(img, interpolation='none', animated=True, label="blah")
ax = plt.gca()

#pybullet.connect(pybullet.GUI)
pybullet.connect(pybullet.DIRECT)

fixedTimeStep = 0.0001
p.resetSimulation()
p.setTimeStep(fixedTimeStep)
p.setRealTimeSimulation(0)

pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.loadURDF("plane.urdf") #, [0, 0, -1])
#pybullet.loadURDF("r2d2.urdf")
robot = pybullet.loadURDF("/home/jay/catkin_ws/src/stretch_ros/stretch_description/urdf/stretch_main.urdf", useFixedBase=True)

# get object names:
obj_names = {}
for iobj in range(p.getNumBodies()):
    obj_info = p.getBodyInfo(iobj)
    # MJCF puts name in 1st field, URDF in 2nd field:
    obj_name = obj_info[0] if obj_info[1] == b'physics' else obj_info[1]
    obj_names[iobj] = obj_name.decode('utf8')

# Set all joints of robot to their mean position:
for jointIndex in range(p.getNumJoints(robot)):
    joint = p.getJointInfo(robot, jointIndex)
    mean_jointpos = np.mean(joint[8:10])
    p.resetJointState(robot, jointIndex, mean_jointpos)
    p.setJointMotorControl2(robot, jointIndex, p.TORQUE_CONTROL, 0)
    print(f"Joint: {jointIndex} {joint[1].decode('utf8')}, min/max: {joint[8:10]} pos: {joint[-3]} friction: {joint[6]} maxForce: {joint[10]}")

camTargetPos = [0, 0, 0.75]
cameraUp = [0, 0, 1]
cameraPos = [1, 1, 1]
pybullet.setGravity(0, 0, -10)

pitch = -10.0

roll = 0
upAxisIndex = 2
camDistance = 1.5
pixelWidth = 4*320
pixelHeight = 4*200
nearPlane = 0.01
farPlane = 100

fov = 60

nsteps = 180
yaw_offset = -90
main_start = time.time()
while (1):
  for yaw in range(0, 180, 1):

    istep = yaw
    for jointIndex in list(range(2,9)) + [10, 11, 12]:
      joint = p.getJointInfo(robot, jointIndex)
      max, min = joint[8:10]
      mean = np.mean(joint[8:10])
      amp = 0.5*np.diff(joint[8:10])[0]

      joint_pos = mean + amp*np.sin(2.*np.pi*istep/nsteps)
      p.resetJointState(robot, jointIndex, joint_pos)


    pybullet.stepSimulation()
    start = time.time()

    viewMatrix = pybullet.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, 2*yaw + yaw_offset, pitch,
                                                            roll, upAxisIndex)
    aspect = pixelWidth / pixelHeight
    projectionMatrix = pybullet.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane)
    img_arr = pybullet.getCameraImage(pixelWidth,
                                      pixelHeight,
                                      viewMatrix,
                                      projectionMatrix,
                                      shadow=1,
                                      lightDirection=[0.5, 0.5, 1],
                                      renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
    stop = time.time()
    print("renderImage %f" % (stop - start))

    w = img_arr[0]  #width of the image, in pixels
    h = img_arr[1]  #height of the image, in pixels
    rgb = img_arr[2]  #color data RGB
    dep = img_arr[3]  #depth data

    print('width = %d height = %d' % (w, h))

    #note that sending the data to matplotlib is really slow

    #reshape is needed
    np_img_arr = np.reshape(rgb, (h, w, 4))
    np_img_arr = np_img_arr * (1. / 255.)

    #show
    #plt.imshow(np_img_arr,interpolation='none',extent=(0,1600,0,1200))
    #image = plt.imshow(np_img_arr,interpolation='none',animated=True,label="blah")

    image.set_data(np_img_arr)
    ax.plot([0])
    #plt.draw()
    #plt.show()
    plt.pause(0.01)
    plt.savefig(f"test{yaw}.png")
    #image.draw()
  quit()
main_stop = time.time()

print("Total time %f" % (main_stop - main_start))

pybullet.resetSimulation()
