import pybullet as p
import pybullet_data
from transforms3d.euler import euler2quat, quat2euler
from time import sleep
import math
class PyBulletIK:
    def __init__(self) -> None:
        physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version or GUI
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        planeId = p.loadURDF("plane.urdf")
        p.setAdditionalSearchPath('/home/roosh/Intro2ArmPi/Functions/py_bullet_arm_ik') #optionally
        p.setGravity(0,0,-10)
        startPos = [0,0,-0.08]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.pi_arm_ID = p.loadURDF("pi_arm.urdf", startPos, startOrientation)
        self.ee = 5
        self.numJoints = 5
        self.threshold = 0.01
    
    def move_arm(self, target = [0,0,0,0,0]):
        """Reloads arm in final position - no collisions calculated"""
        if not len(target) == 5:
            print("ERROR: Incorrect number of joints.")
            exit()
        
        for i in range(5):
            p.resetJointState(self.pi_arm_ID, i+1, target[i])

    def compute_ik(self, target = [.1, 0, .2, .0, .0, .0, .1],):
        if len(target) == 3:
            xyz_target = target
            target = p.calculateInverseKinematics(self.pi_arm_ID, self.ee, target, maxNumIterations=1000, residualThreshold=0.001)
        else:
            xyz_target = target[:3]
            target = p.calculateInverseKinematics(self.pi_arm_ID, self.ee, target[:3], target[-4:], maxNumIterations=1000, residualThreshold=0.001)
        self.move_arm(target)
        ls = p.getLinkState(self.pi_arm_ID, self.ee)
        print(target)
        newPos = ls[4]
        newOr = ls[5]
        diff = [xyz_target[0] - newPos[0], xyz_target[1] - newPos[1], xyz_target[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        print(f"threshold: {str(dist2)} at pose: {str(newPos)} {str(quat2euler(newOr))} from pose {xyz_target}")
        if dist2 > self.threshold:
            return False
        return target

    def compute_ik_pair(self, xyz_target = [.1, 0, .2], orientation = [.0, .0, .0, .1]):
        closeEnough = False
        iter = 0
        dist2 = 1e30
        jointPoses = p.calculateInverseKinematics(self.pi_arm_ID, self.ee, xyz_target, orientation, maxNumIterations=1000, residualThreshold=0.01)
        self.move_arm(jointPoses)
        ls = p.getLinkState(self.pi_arm_ID, self.ee)
        newPos = ls[4]
        newOr = ls[5]

        while (not closeEnough and iter < 100):
            jointPoses = p.calculateInverseKinematics(self.pi_arm_ID, self.ee, xyz_target, orientation, maxNumIterations=1000, residualThreshold=0.01)
            for i in range(5):
                p.resetJointState(self.pi_arm_ID, i, jointPoses[i])
            ls = p.getLinkState(self.pi_arm_ID, self.ee)
            newPos = ls[4]
            newOr = ls[5]
            diff = [xyz_target[0] - newPos[0], xyz_target[1] - newPos[1], xyz_target[2] - newPos[2]]
            dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
            closeEnough = (dist2 < self.threshold)
            iter = iter + 1
        print(jointPoses)
        print (f"threshold: {str(dist2)} at pose: {str(newPos)} {str(quat2euler(newOr))} from pose {xyz_target} {quat2euler(orientation)}")
        if self.threshold < dist2:
            return False
        return jointPoses
        
if __name__ == '__main__':
    IK = PyBulletIK()
    IK.compute_ik_pair([.15, 0, .1], euler2quat(0, math.radians(30), 0))