//see https://moveit.picknik.ai/main/doc/tutorials/your_first_project/your_first_project.html

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <chrono>
#include <thread>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto const node = std::make_shared<rclcpp::Node>(
      "joint_hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("hello_moveit");

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  moveit::core::RobotStatePtr current_state(new moveit::core::RobotState(move_group.getRobotModel()));

  std::vector<std::string> joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
  std::vector<double> joint_values = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};

  // Print the current joint values
  RCLCPP_INFO(logger, "Current joint values");
  for (size_t i = 0; i < joint_values.size(); ++i)
  {
    RCLCPP_INFO(logger, "Joint %zu: %.2f", i, joint_values[i]);
  }

  current_state->setJointGroupPositions(move_group.getName(), joint_values);

  // Set the current state in the MoveGroupInterface
  move_group.setStartState(*current_state);

  // Get the current set of joint values for the group.
  std::vector<double> joint_group_positions = joint_values;

  // Modifying one of the joint positions
  joint_group_positions[2] = joint_values[2] + 0.5; // radians

  move_group.setJointValueTarget(joint_group_positions);

  move_group.setMaxVelocityScalingFactor(0.05);
  move_group.setMaxAccelerationScalingFactor(0.05);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    move_group.execute(my_plan);
    RCLCPP_INFO(logger, "Plan and execution succeeded");
  }
  else
  {
    RCLCPP_ERROR(logger, "Plan and execution failed");
  }

  std::this_thread::sleep_for(std::chrono::seconds(10));

  rclcpp::shutdown();
  return 0;
}

