#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ament_index_cpp/get_package_share_directory.hpp>  // Include the correct header for ament_index_cpp
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

// Function to load a file's contents into a string
std::string load_file_to_string(const std::string &file_path)
{
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + file_path);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// Function to process a Xacro file into a URDF file
std::string process_xacro(const std::string &xacro_file)
{
    std::string urdf_file = "/tmp/processed_urdf.urdf";  // Temporary URDF file path
    std::string command = "xacro " + xacro_file + " -o " + urdf_file;

    int result = std::system(command.c_str());
    if (result != 0) {
        throw std::runtime_error("Failed to process xacro file: " + xacro_file);
    }

    return urdf_file;
}

int main(int argc, char *argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_alpha_example");

    // Load URDF and SRDF using ament_index_cpp
    std::string xacro_file = ament_index_cpp::get_package_share_directory("alpha_description_simulation") + "/config/alpha.config.xacro";
    std::string srdf_file = ament_index_cpp::get_package_share_directory("alpha_description_simulation") + "/config/alpha.srdf";

    try {
        // Process the Xacro file into URDF
        std::string urdf_file = process_xacro(xacro_file);

        // Set robot_description and robot_description_semantic parameters
        node->declare_parameter("robot_description", rclcpp::ParameterValue(load_file_to_string(urdf_file)));
        node->declare_parameter("robot_description_semantic", rclcpp::ParameterValue(load_file_to_string(srdf_file)));
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load URDF or SRDF: %s", e.what());
        return 1;
    }

    // Initialize MoveGroupInterface for the planning group
    moveit::planning_interface::MoveGroupInterface move_group(node, "alpha_arm");

    // Set planning time to give enough time for planning
    move_group.setPlanningTime(20.0);

    // Set neutral joint positions
    std::vector<double> joint_group_positions = {0.0, 0.0, 0.0, 0.0, 0.0};
    move_group.setJointValueTarget(joint_group_positions);

    // Plan and execute to neutral pose
    moveit::planning_interface::MoveGroupInterface::Plan initial_plan;
    bool initial_success = (move_group.plan(initial_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (initial_success) {
        move_group.execute(initial_plan);
        RCLCPP_INFO(node->get_logger(), "Initial neutral position reached.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to move to the initial neutral position!");
        rclcpp::shutdown();
        return 1;
    }

    // Define a target pose for the end-effector
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.2;  // X in meters
    target_pose.position.y = 0.0;  // Y in meters
    target_pose.position.z = 0.3;  // Z in meters

    // Set the target pose for the end-effector
    move_group.setPoseTarget(target_pose);

    // Plan the motion to the target pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group.execute(plan);
        RCLCPP_INFO(node->get_logger(), "Motion to target pose executed successfully.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }

    rclcpp::shutdown();
    return 0;
}
