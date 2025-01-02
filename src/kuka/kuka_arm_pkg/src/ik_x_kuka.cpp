#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

std::string load_file_to_string(const std::string& file_path)
{
  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + file_path);
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

std::string process_xacro(const std::string& xacro_file)
{
  std::string urdf_file = "/tmp/processed_urdf.urdf"; 
  std::string command = "xacro " + xacro_file + " -o " + urdf_file;

  int result = std::system(command.c_str());
  if (result != 0) {
    throw std::runtime_error("Failed to process xacro file: " + xacro_file);
  }

  return urdf_file; 
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("moveit_cpp_example");

  std::string xacro_file = ament_index_cpp::get_package_share_directory("kuka_arm_moveit") + "/config/kr210_arm.urdf.xacro";
  std::string srdf_file = ament_index_cpp::get_package_share_directory("kuka_arm_moveit") + "/config/kr210_arm.srdf";

  try {
    std::string urdf_file = process_xacro(xacro_file);

    node->declare_parameter("robot_description", rclcpp::ParameterValue(load_file_to_string(urdf_file)));
    node->declare_parameter("robot_description_semantic", rclcpp::ParameterValue(load_file_to_string(srdf_file)));
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load URDF or SRDF: %s", e.what());
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface move_group(node, "kuka_arm");

  move_group.setPlanningTime(10.0);  

  std::vector<geometry_msgs::msg::Pose> target_poses;

  // Goal 1
  geometry_msgs::msg::Pose goal_1;
  goal_1.orientation.w = 1.0;
  goal_1.position.x = 2.0;
  goal_1.position.y = 1.5;
  goal_1.position.z = 2.0;
  target_poses.push_back(goal_1);

  // Goal 2
  geometry_msgs::msg::Pose goal_2;
  goal_2.orientation.w = 1.0;
  goal_2.position.x = 1.0;
  goal_2.position.y = 1.0;
  goal_2.position.z = 1.5;
  target_poses.push_back(goal_2);

  // Goal 3
  geometry_msgs::msg::Pose goal_3;
  goal_3.orientation.w = 1.0;
  goal_3.position.x = 1.5;
  goal_3.position.y = 2.0;
  goal_3.position.z = 1.0;
  target_poses.push_back(goal_3);

  for (size_t i = 0; i < target_poses.size(); ++i) {
    RCLCPP_INFO(node->get_logger(), "Planning and executing goal %zu", i + 1);
    
    move_group.setPoseTarget(target_poses[i]);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group.execute(plan);
      RCLCPP_INFO(node->get_logger(), "Goal %zu executed successfully!", i + 1);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Planning failed for goal %zu", i + 1);
    }

    rclcpp::sleep_for(std::chrono::seconds(2));
  }

  rclcpp::shutdown();
  return 0;
}
