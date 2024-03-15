import pybullet as p
import pybullet_data
class PyBulletIK:
    def __init__(self) -> None:
        physicsClient = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        planeId = p.loadURDF("plane.urdf")
        p.setAdditionalSearchPath('/home/kyle/ik_arm_test/intro_2_arm_relaxed_ik_core/configs/urdfs') #optionally
        p.setGravity(0,0,-10)
        startPos = [0,0,0]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        # print(p.getQuaternionFromEuler([0,0,0]))
        self.pi_arm_ID = p.loadURDF("pi_arm.urdf",startPos, startOrientation)
        p.resetBasePositionAndOrientation(self.pi_arm_ID, [0, 0, -.1], [0, 0, 0, 1])
        self.ee = 5
    
    def move_arm(self, target = [0,0,0,0,0]):
        if not len(target) == 5:
            print("ERROR: Incorrect number of joints.")
            exit()
        
        for i in range(5):
            p.resetJointState(self.pi_arm_ID, i+1, target[i])

    def compute_ik(self, xyz_target = [.1, 0, .2]):
        target = p.calculateInverseKinematics(self.pi_arm_ID, self.ee, xyz_target)
        return target

        
