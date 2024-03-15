from pybullet_arm import PyBulletIK

IK = PyBulletIK()
for i in range(100):
    print(IK.compute_ik([.2, 0, .1]))