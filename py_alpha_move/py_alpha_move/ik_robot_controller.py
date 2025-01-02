# Markus Buchholz
#ros2 topic pub /ik_goal geometry_msgs/Pose "{position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import pybullet as p
import pybullet_data
import os
import math
import time


class RobotController(Node):

    def __init__(self):
        super().__init__('ik_robot_controller')
        
        self.fk_pub = self.create_publisher(Float64MultiArray, '/alpha_move_joint', 10)
        
        self.create_subscription(Pose, '/ik_goal', self.ik_callback, 10)

        self.custom_data_path = "/root/colcon_ws/src/reach_alpha5/alpha5_bullet"
        self.init_pybullet()

    def init_pybullet(self):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(self.custom_data_path)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf", [0, 0, -0.3])
        urdf_path = os.path.join(self.custom_data_path, 'urdf', 'alpha.urdf')
        self.robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        self.active_joint_indices = [2, 3, 4, 5, 7]
        self.reset_initial_joint_positions()

    def reset_initial_joint_positions(self):
        initial_joint_positions = [0.0, 1.5746, 0.0, 1.5746, 0.0]
        for i, joint_index in enumerate(self.active_joint_indices):
            p.resetJointState(self.robot_id, joint_index, initial_joint_positions[i])

    def compute_fk(self, joint_poses):
        for i, joint_index in enumerate(self.active_joint_indices):
            p.resetJointState(self.robot_id, joint_index, joint_poses[i])
        link_state = p.getLinkState(self.robot_id, self.active_joint_indices[-1])
        end_effector_position = link_state[4]
        return end_effector_position

    def compute_ik(self, target_position):
        orn = p.getQuaternionFromEuler([0, math.pi/2, 0])
        joint_poses = p.calculateInverseKinematics(self.robot_id, self.active_joint_indices[-1],
                                                   target_position, orn, maxNumIterations=200, residualThreshold=0.01)
        return joint_poses

    def ik_callback(self, msg):
        target_position = [msg.position.x, msg.position.y, msg.position.z]
        
        joint_poses = self.compute_ik(target_position)

        if len(joint_poses) >= 5:
            joint_poses = joint_poses[:5]  

            joint_msg = Float64MultiArray()
            joint_msg.data = list(joint_poses) 

           
            self.fk_pub.publish(joint_msg)

            self.get_logger().info(f"Published Joint Positions: {joint_poses}")
        else:
            self.get_logger().error("Not enough joint positions calculated for IK!")



def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass

    robot_controller.destroy_node()
    rclpy.shutdown()
    p.disconnect()


if __name__ == '__main__':
    main()
