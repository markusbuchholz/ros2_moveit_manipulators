# Markus Buchholz
import pybullet as p
import pybullet_data
import time
import math
from datetime import datetime
import os


clid = p.connect(p.SHARED_MEMORY)
if clid < 0:
    p.connect(p.GUI)


custom_data_path = "/home/markus/underwater/xpicknik/alpha5_ik/alpha5_bullet"
p.setAdditionalSearchPath(custom_data_path)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  


p.loadURDF("plane.urdf", [0, 0, -0.3])


urdf_path = os.path.join(custom_data_path, 'urdf', 'alpha.urdf')  
robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)


num_joints = p.getNumJoints(robot_id)
print(f"Number of joints in the robot: {num_joints}")


for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    print(f"Joint {i}: {info[1].decode('utf-8')}, Type: {info[2]}")


active_joint_indices = [2, 3, 4, 5, 6]  
print(f"Active joint indices: {active_joint_indices}")


p.setGravity(0, 0, -9.81)

# initial joint positions
initial_joint_positions = [3.0, 1.0, 1.0, 0.0, 0.0]  
if len(initial_joint_positions) != len(active_joint_indices):
    print(f"Warning: The number of initial joint positions ({len(initial_joint_positions)}) does not match the number of active joints ({len(active_joint_indices)}).")
else:
    for i, joint_index in enumerate(active_joint_indices):
        p.resetJointState(robot_id, joint_index, initial_joint_positions[i])

for _ in range(100):
    p.stepSimulation()
    time.sleep(0.01)


use_null_space = True
use_orientation = True
use_simulation = False  # set to False to directly apply IK results
ik_solver = 0

# define null space parameters
# these values are placeholders, you need to adjust them based on your robot's joint limits
ll = [-3.14] * len(active_joint_indices)  # lower limits for null space
ul = [3.14] * len(active_joint_indices)  # upper limits for null space
jr = [6.28] * len(active_joint_indices)  # joint ranges for null space
rp = [0] * len(active_joint_indices)  # rest poses for null space
jd = [0.1] * len(active_joint_indices)  # joint damping coefficients

# compute IK
def compute_ik(target_position):
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])
    if use_null_space:
        if use_orientation:
            joint_poses = p.calculateInverseKinematics(robot_id, active_joint_indices[-1], target_position, orn, ll, ul, jr, rp)
        else:
            joint_poses = p.calculateInverseKinematics(robot_id, active_joint_indices[-1], target_position, lowerLimits=ll, upperLimits=ul, jointRanges=jr, restPoses=rp)
    else:
        if use_orientation:
            joint_poses = p.calculateInverseKinematics(robot_id, active_joint_indices[-1], target_position, orn, jointDamping=jd, solver=ik_solver, maxNumIterations=100, residualThreshold=0.01)
        else:
            joint_poses = p.calculateInverseKinematics(robot_id, active_joint_indices[-1], target_position, solver=ik_solver)
    return joint_poses

def main():
    while True:
        input_str = input("Enter the target position (x, y, z) as comma-separated values or 'exit' to quit: ")
        if input_str.lower() == 'exit':
            break

        try:
            x, y, z = map(float, input_str.split(','))
            target_position = [x, y, z]
        except ValueError:
            print("Invalid input. Please enter three comma-separated numbers.")
            continue

       
        joint_poses = compute_ik(target_position)

        print(f"Number of joint poses: {len(joint_poses)}")
        print(f"Joint poses: {joint_poses}")

        # set the joint positions
        for i, joint_index in enumerate(active_joint_indices):
            if i < len(joint_poses):
                p.resetJointState(robot_id, joint_index, joint_poses[i])
            else:
                print(f"Warning: Joint index {joint_index} out of range for joint poses")

        
        p.stepSimulation()
        time.sleep(0.1)  

if __name__ == "__main__":
    main()
    p.disconnect()
