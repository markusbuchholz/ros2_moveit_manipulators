# Markus Buchholz
import pybullet as p
import pybullet_data
import time
import math
import os

# Connect to PyBullet
clid = p.connect(p.SHARED_MEMORY)
if clid < 0:
    p.connect(p.GUI)

custom_data_path = "/home/markus/underwater/xpicknik/alpha5_ik/alpha5_bullet"
p.setAdditionalSearchPath(custom_data_path)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # include pybullet_data for plane.urdf

p.loadURDF("plane.urdf", [0, 0, -0.3])

urdf_path = os.path.join(custom_data_path, 'urdf', 'alpha.urdf')  
robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

# Get the number of joints in your robot
num_joints = p.getNumJoints(robot_id)
print(f"Number of joints in the robot: {num_joints}")

for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: {info[1].decode('utf-8')}, Type: {info[2]}, Lower limit: {info[8]}, Upper limit: {info[9]}")

# active joints in Alpha5
active_joint_indices = [2, 3, 4, 5, 6]
print(f"Active joint indices: {active_joint_indices}")

p.setGravity(0, 0, -9.81)

def set_joint_positions(joint_positions):
    for i, joint_index in enumerate(active_joint_indices):
        p.setJointMotorControl2(bodyIndex=robot_id, jointIndex=joint_index, controlMode=p.POSITION_CONTROL, targetPosition=joint_positions[i])

    p.stepSimulation()

    # get the end effector state
    end_effector_state = p.getLinkState(robot_id, active_joint_indices[-1])
    end_effector_pos = end_effector_state[4]  # Link world position
    end_effector_ori = p.getEulerFromQuaternion(end_effector_state[5])  # Link world orientation

    print(f"End Effector Position: {end_effector_pos}")
    print(f"End Effector Orientation: {end_effector_ori}")

# Main loop
try:
    while True:
        input_str = input("provide the joint positions (comma-separated) for joints 2, 3, 4, 5, 6 or 'exit' to quit (e.g. ::, 0.5,0.1,-0.2,0.3,0.4): ")
        if input_str.lower() == 'exit':
            break

        try:
            joint_positions = list(map(float, input_str.split(',')))
            if len(joint_positions) != len(active_joint_indices):
                print(f"Invalid input. Please enter {len(active_joint_indices)} values.")
                continue
        except ValueError:
            print("Invalid input. Please enter valid floating-point numbers.")
            continue

        set_joint_positions(joint_positions)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")

p.disconnect()
