# Markus Buchholz
import pybullet as p
import pybullet_data
import time
import math
import os


clid = p.connect(p.SHARED_MEMORY)
if clid < 0:
    p.connect(p.GUI)


custom_data_path = "/home/markus/underwater/xpicknik/alpha5_ik/alpha5_bullet"
p.setAdditionalSearchPath(custom_data_path)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

# URDF
p.loadURDF("plane.urdf", [0, 0, -0.3])

# custom URDF file
urdf_path = os.path.join(custom_data_path, 'urdf', 'alpha.urdf')  
robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

# get the number of joints
num_joints = p.getNumJoints(robot_id)
print(f"Number of joints in the robot: {num_joints}")

# print out joint information
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: {info[1].decode('utf-8')}, Type: {info[2]}, Lower limit: {info[8]}, Upper limit: {info[9]}")

# indices of the active joints in Alpha
active_joint_indices = [2, 3, 4, 5, 6]  
print(f"Active joint indices: {active_joint_indices}")


p.setGravity(0, 0, -9.81)

slider_x = p.addUserDebugParameter("x", -2, 2, 0.5)
slider_y = p.addUserDebugParameter("y", -1, 1, 0)
slider_z = p.addUserDebugParameter("z", -1, 1, 0.5)

def compute_ik():
    target_position = [p.readUserDebugParameter(slider_x),
                       p.readUserDebugParameter(slider_y),
                       p.readUserDebugParameter(slider_z)]

    orn = p.getQuaternionFromEuler([0, -math.pi, 0])
    joint_poses = p.calculateInverseKinematics(robot_id, active_joint_indices[-1], target_position, orn)

    for i, joint_index in enumerate(active_joint_indices):
        p.setJointMotorControl2(bodyIndex=robot_id, jointIndex=joint_index, controlMode=p.POSITION_CONTROL, targetPosition=joint_poses[i])

    p.stepSimulation()

    print(f"Target Position: {target_position}")
    print(f"Joint Positions: {joint_poses[:len(active_joint_indices)]}")

try:
    while True:
        compute_ik()
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")

p.disconnect()
