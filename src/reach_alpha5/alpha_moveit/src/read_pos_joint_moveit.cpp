
#include <memory>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("joint_trajectory_publisher");

  auto logger = rclcpp::get_logger("joint_trajectory_publisher");

  
  auto publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/feedback_joint_position_trajectory_controller/joint_trajectory", 10);

  
  std::this_thread::sleep_for(std::chrono::seconds(2));

 
  auto trajectory_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  trajectory_msg->header.stamp = node->now();
  trajectory_msg->joint_names = {"axis_a", "axis_b", "axis_c", "axis_d", "axis_e"};

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.0, 2.0, 2.72, 0.6, 2.85};
  point.time_from_start = rclcpp::Duration(1, 0); // 1 sek

  trajectory_msg->points.push_back(point);


  RCLCPP_INFO(logger, "Publishing joint trajectory");


  publisher->publish(*trajectory_msg);

 
  std::this_thread::sleep_for(std::chrono::seconds(5));

 
  rclcpp::shutdown();
  return 0;
}
