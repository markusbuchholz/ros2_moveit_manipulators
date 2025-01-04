import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class EndEffectorPosition(Node):
    def __init__(self):
        super().__init__('end_effector_position')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.get_end_effector_position)

    def get_end_effector_position(self):
        try:
            # Replace 'base_link' and 'ee_base_link' with your robot's frame names from the image
            transform: TransformStamped = self.tf_buffer.lookup_transform('base_link', 'ee_base_link', rclpy.time.Time())
            
            # Extract the position (XYZ)
            translation = transform.transform.translation
            x = translation.x
            y = translation.y
            z = translation.z

            # Extract the orientation (Quaternion)
            rotation = transform.transform.rotation
            qx = rotation.x
            qy = rotation.y
            qz = rotation.z
            qw = rotation.w

            # Print both position and orientation
            self.get_logger().info(f"End-Effector Position: X: {x}, Y: {y}, Z: {z}")
            self.get_logger().info(f"End-Effector Orientation: Quaternion [X: {qx}, Y: {qy}, Z: {qz}, W: {qw}]")

        except tf2_ros.LookupException:
            self.get_logger().error('Transform not available')
        except tf2_ros.ConnectivityException:
            self.get_logger().error('Connectivity error occurred')
        except tf2_ros.ExtrapolationException:
            self.get_logger().error('Extrapolation error occurred')

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPosition()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



#9993322.770694979] [end_effector_position]: End-Effector Position: X: -0.006160307442063567, Y: -0.04074780675476718, Z: 0.3306048902883848
# End-Effector Position: X: 0.23780827992557838, Y: 0.14203949077843536, Z: 0.0773801321667092
#End-Effector Position: X: -0.11849968105278785, Y: 0.11193161314495437, Z: 0.04118090618096928

#[INFO] [1729993955.377862672] [end_effector_position]: End-Effector Position: X: 0.07737917939519819, Y: 0.04524373714470452, Z: 0.12464334664840018
#[INFO] [1729993955.378163223] [end_effector_position]: End-Effector Orientation: Quaternion [X: 0.03712323978658265, Y: -0.37205806206709763, Z: 0.38633780395245026, W: 0.8431712547020456]


#ros2 topic pub /ik_goal geometry_msgs/Pose "{position: {x: 0.07737917939519819, y: 0.04524373714470452, z: 0.12464334664840018}, orientation: {x: 0.03712323978658265, y: -0.37205806206709763, z: 0.38633780395245026, w: 0.8431712547020456}}"