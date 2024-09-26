# Markus Buchholz
import pybullet as p
import pybullet_data
import time
import math
import os

# connect to PyBullet
clid = p.connect(p.SHARED_MEMORY)
if clid < 0:
    p.connect(p.GUI)

custom_data_path = "/home/markus/underwater/xpicknik/alpha5_ik/alpha5_bullet"
p.setAdditionalSearchPath(custom_data_path)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # include pybullet_data for plane.urdf

p.loadURDF("plane.urdf", [0, 0, -0.3])

urdf_path = os.path.join(custom_data_path, 'urdf', 'alpha.urdf')  # Update the file name as needed
robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

# get the number of joints in your robot
num_joints = p.getNumJoints(robot_id)
print(f"Number of joints in the robot: {num_joints}")

for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: {info[1].decode('utf-8')}, Type: {info[2]}, Lower limit: {info[8]}, Upper limit: {info[9]}")

# define the indices of the active joints in Alpha5
active_joint_indices = [2, 3, 4, 5, 6]  
print(f"Active joint indices: {active_joint_indices}")


p.setGravity(0, 0, -9.81)

joint_sliders = []
for i in active_joint_indices:
    joint_info = p.getJointInfo(robot_id, i)
    joint_name = joint_info[1].decode("utf-8")
    joint_type = joint_info[2]
    if joint_type == p.JOINT_PRISMATIC:
        slider = p.addUserDebugParameter(joint_name, joint_info[8], joint_info[9], (joint_info[8] + joint_info[9]) / 2)
    else:
        slider = p.addUserDebugParameter(joint_name, -3.14, 3.14, 0.0)
    joint_sliders.append(slider)

def compute_forward_kinematics():
    joint_positions = []
    for i, slider in enumerate(joint_sliders):
        pos = p.readUserDebugParameter(slider)
        joint_index = active_joint_indices[i]
        p.setJointMotorControl2(bodyIndex=robot_id, jointIndex=joint_index, controlMode=p.POSITION_CONTROL, targetPosition=pos)
        joint_positions.append(pos)

    p.stepSimulation()
    
    end_effector_state = p.getLinkState(robot_id, active_joint_indices[-1])
    end_effector_pos = end_effector_state[4]  # Link world position
    end_effector_ori = p.getEulerFromQuaternion(end_effector_state[5])  # Link world orientation
    
    print(f"End Effector Position: {end_effector_pos}")
    print(f"End Effector Orientation: {end_effector_ori}")

# main loop
try:
    while True:
        compute_forward_kinematics()
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")

p.disconnect()
