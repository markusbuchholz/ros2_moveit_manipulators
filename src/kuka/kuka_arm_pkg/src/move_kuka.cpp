#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <cstdlib>

// Function to load a file's contents into a string
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

// Function to process a xacro file into a URDF file
std::string process_xacro(const std::string& xacro_file)
{
    std::string urdf_file = "/tmp/processed_urdf.urdf";  // Temporary URDF file
    std::string command = "xacro " + xacro_file + " -o " + urdf_file;

    int result = std::system(command.c_str());
    if (result != 0) {
        throw std::runtime_error("Failed to process xacro file: " + xacro_file);
    }

    return urdf_file;
}

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher()
    : Node("trajectory_publisher_node")
    {
        // Load URDF and SRDF manually as parameters
        load_and_set_robot_description();

        // Publisher for sending joint trajectory commands to the robot arm controller
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/kuka_arm_controller/joint_trajectory", 10);

        // Timer to periodically send trajectory messages
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TrajectoryPublisher::timer_callback, this));

        // Joint names for the KUKA arm
        joints_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        
        // Goal positions for the joints (example values)
        goal_positions_ = {2.5, 1.5, -2.5, 0.5, 0.3, 0.1};
    }

private:
    // This function is called periodically by the timer to publish joint trajectory commands
    void timer_callback()
    {
        // Create a new JointTrajectory message
        auto message = trajectory_msgs::msg::JointTrajectory();
        message.joint_names = joints_;

        // Create a trajectory point and set the desired joint positions
        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        point.positions = goal_positions_;

        // Specify the time duration to reach the goal positions
        point.time_from_start.sec = 2;  // 2 seconds to reach the goal positions
        point.time_from_start.nanosec = 0;

        // Add the point to the trajectory
        message.points.push_back(point);

        // Publish the trajectory message
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Trajectory message published.");

        // Print the final joint positions (goal positions in this case)
        RCLCPP_INFO(this->get_logger(), "Final joint positions:");
        for (size_t i = 0; i < goal_positions_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Joint %zu: %f", i, goal_positions_[i]);
        }

        // Now, we will use IK to compute the end-effector XYZ position
        compute_end_effector_xyz(goal_positions_);
    }

    // Load and set robot_description and robot_description_semantic parameters
    void load_and_set_robot_description()
    {
        std::string xacro_file = ament_index_cpp::get_package_share_directory("kuka_arm_moveit") + "/config/kr210_arm.urdf.xacro";
        std::string srdf_file = ament_index_cpp::get_package_share_directory("kuka_arm_moveit") + "/config/kr210_arm.srdf";

        try {
            std::string urdf_file = process_xacro(xacro_file);
            this->declare_parameter("robot_description", rclcpp::ParameterValue(load_file_to_string(urdf_file)));
            this->declare_parameter("robot_description_semantic", rclcpp::ParameterValue(load_file_to_string(srdf_file)));
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load URDF or SRDF: %s", e.what());
        }
    }

    // Function to compute end-effector position (XYZ) from joint values using MoveIt IK
    void compute_end_effector_xyz(const std::vector<double>& joint_positions)
    {
        // Corrected the RobotModelLoader constructor call
        robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description"); // Pass node's shared pointer
        const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

        if (!kinematic_model) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model.");
            return;
        }

        // Create a RobotState and set the joint positions
        moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
        const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("kuka_arm");

        kinematic_state->setJointGroupPositions(joint_model_group, joint_positions);
        kinematic_state->update();

        // Get the end-effector pose (transformation matrix)
        const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform(joint_model_group->getLinkModelNames().back());

        // Extract XYZ position from the transformation matrix
        Eigen::Vector3d xyz = end_effector_state.translation();

        // Print the XYZ position
        RCLCPP_INFO(this->get_logger(), "End-effector position (XYZ): x=%f, y=%f, z=%f", xyz.x(), xyz.y(), xyz.z());
    }

    // Publisher for joint trajectory messages
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    
    // Timer to trigger trajectory message publishing
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Names of the joints of the KUKA arm
    std::vector<std::string> joints_;
    
    // Desired goal positions for the joints
    std::vector<double> goal_positions_;
};

int main(int argc, char * argv[])
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    
    // Create and spin the node
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
