from pybullet_arm import PyBulletIK
from time import sleep
import pybullet as p

IK = PyBulletIK()

jointPoses = [IK.compute_ik([.15,0, .2]),IK.compute_ik([.15, 0, -.05]),IK.compute_ik([.15,0, .2])] #IK.compute_ik([.1, .1, .1]), IK.compute_ik([.1, 0, .1]), IK.compute_ik([.1, .1, .1]), IK.compute_ik([0,0, .25])]#, IK.compute_ik([.15,0, -.05])]

angles = p.getJointStates(IK.pi_arm_ID, [1,2,3,4,5])
current = [angles[0][0], angles[1][0], angles[2][0], angles[3][0], angles[4][0]]

import time

start = time.time()

print(IK.check_trajectory(current, jointPoses[1]))
print("hello")
end = time.time()
print(end - start)

