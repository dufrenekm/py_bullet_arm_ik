from pybullet_arm import PyBulletIK
from time import sleep
import pybullet as p
IK = PyBulletIK()
# for i in range(100):
jointPoses = IK.compute_ik([.1, 0, .1])

for i in [0,1,2,3,4,5]:
        p.setJointMotorControl2(bodyIndex=IK.pi_arm_ID,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=0,
                                force=500,
                                positionGain=0.03,
                                velocityGain=1)




while True:
    p.stepSimulation()
    sleep(1)