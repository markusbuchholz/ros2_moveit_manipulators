#Markus Buchholz
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.optimize import minimize
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

def rot_x(theta):
    return np.array([
        [1, 0,           0,          0],
        [0, np.cos(theta), -np.sin(theta), 0],
        [0, np.sin(theta),  np.cos(theta), 0],
        [0, 0,           0,          1]
    ])

def rot_y(theta):
    return np.array([
        [ np.cos(theta), 0, np.sin(theta), 0],
        [0,           1, 0,           0],
        [-np.sin(theta), 0, np.cos(theta), 0],
        [0,           0, 0,           1]
    ])

def rot_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta),  np.cos(theta), 0, 0],
        [0,           0,           1, 0],
        [0,           0,           0, 1]
    ])

def trans(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def joint_transform(joint_angle, joint_axis, joint_origin, joint_rpy):
    T = trans(*joint_origin)
    R_fixed = rot_z(joint_rpy[2]) @ rot_y(joint_rpy[1]) @ rot_x(joint_rpy[0])
    T_fixed = T @ R_fixed

    if np.allclose(joint_axis, [1, 0, 0]):
        T_joint = rot_x(joint_angle)
    elif np.allclose(joint_axis, [0, 1, 0]):
        T_joint = rot_y(joint_angle)
    elif np.allclose(joint_axis, [0, 0, 1]):
        T_joint = rot_z(joint_angle)
    elif np.allclose(joint_axis, [0, 0, -1]):
        T_joint = rot_z(-joint_angle)
    else:
        raise ValueError("Unsupported joint axis")

    return T_fixed @ T_joint

def forward_kinematics(joint_angles):
    joint_axes = [
        [0, 0, 1],
        [0, 1, 0],
        [0, 1, 0],
        [0, 0, -1],
    ]

    joint_origins = [
        [0, 0, 0.014],
        [-0.02, 0, 0.033],
        [-0.04, 0, -0.1453],
        [0.02, 0, 0.033],
    ]

    joint_rpys = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 3.14159],
        [0, 0, 2.09439],
    ]

    T = np.eye(4)

    for i in range(len(joint_angles)):
        axis = joint_axes[i]
        origin = joint_origins[i]
        rpy = joint_rpys[i]
        angle = joint_angles[i]
        T_joint = joint_transform(angle, axis, origin, rpy)
        T = T @ T_joint

    ee_origin = [0, 0, 0.09975]
    T_ee = trans(*ee_origin)
    T = T @ T_ee

    position = T[:3, 3]
    return position

def ik_cost_function(joint_angles, target_position):
    current_position = forward_kinematics(joint_angles)
    position_error = current_position - target_position
    return np.linalg.norm(position_error)

# Joint limits
joint_limits = [
    (2.5, 4.0),  # axis_e
    (0.2, 2.0),  # axis_d
    (0.2, 2.2),  # axis_c
    (0.0, 2.0),  # axis_b
    # axis_a is fixed at 0.0
]

def inverse_kinematics(target_position, initial_guess):
    bounds = joint_limits
    result = minimize(
        ik_cost_function,
        initial_guess,
        args=(target_position,),
        bounds=bounds,
        method='SLSQP',
        options={'ftol': 1e-6, 'maxiter': 10000}
    )
    if result.success:
        return result.x
    else:
        print("Optimization failed:", result.message)
        return None

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver')

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/target_position',
            self.target_position_callback,
            10
        )
        self.subscription  

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/xsubsea/joint_trajectory',
            10
        )

        self.initial_guess = np.array([4.0, 0.1, 0.1, 0.1])

        self.joint_names = ['axis_a', 'axis_b', 'axis_c', 'axis_d', 'axis_e']

        self.get_logger().info('Inverse Kinematics Solver Node has been started.')

    def target_position_callback(self, msg):
        if len(msg.data) != 3:
            self.get_logger().error('Received target_position does not have exactly 3 elements.')
            return

        target_position = np.array(msg.data)
        self.get_logger().info(f'Received target_position: {target_position}')

        joint_angles = inverse_kinematics(target_position, self.initial_guess)

        if joint_angles is not None:
            # Reverse the joint angles and add a fixed angle for 'axis_a'
            # Assuming 'axis_a' is fixed at 0.0
            joint_angles_reversed = joint_angles[::-1]
            joint_angles_published = np.insert(joint_angles_reversed, 0, 0.0)

            self.get_logger().info(f'Joint angles to publish (radians): {joint_angles_published}')

            # Create JointTrajectory message
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = joint_angles_published.tolist()
            # Define the time_from_start for the trajectory point
            # For example, 2 seconds from now
            point.time_from_start = Duration(sec=2, nanosec=0)

            traj_msg.points.append(point)

            self.publisher.publish(traj_msg)

            self.get_logger().info('Published JointTrajectory message.')

            # Verification
            computed_position = forward_kinematics(joint_angles)
            position_error = np.linalg.norm(computed_position - target_position)

            self.get_logger().info('--- Verification ---')
            self.get_logger().info(f'Computed end-effector position: {computed_position}')
            self.get_logger().info(f'Target end-effector position: {target_position}')
            self.get_logger().info(f'Position error: {position_error}')

            if position_error < 1e-4:
                self.get_logger().info('The IK solution is valid.')
            else:
                self.get_logger().warn('The IK solution has errors.')
        else:
            self.get_logger().error('Failed to compute a valid IK solution.')

def main(args=None):
    rclpy.init(args=args)

    ik_solver_node = IKSolverNode()

    try:
        rclpy.spin(ik_solver_node)
    except KeyboardInterrupt:
        pass
    finally:
        ik_solver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
