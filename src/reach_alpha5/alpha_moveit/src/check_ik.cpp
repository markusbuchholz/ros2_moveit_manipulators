#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <chrono>
#include <thread>

class JointStateListener : public rclcpp::Node
{
public:
    JointStateListener() : Node("joint_state_listener"), joint_positions_received_(false)
    {
        // Subscribe to the /joint_states topic
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&JointStateListener::jointStateCallback, this, std::placeholders::_1));
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!msg->position.empty())
        {
            joint_positions_ = msg->position;
            joint_positions_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Received joint state:");
            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "Joint %s: Position: %.3f", msg->name[i].c_str(), msg->position[i]);
            }
        }
    }

    bool areJointPositionsReceived() const
    {
        return joint_positions_received_;
    }

    std::vector<double> getJointPositions() const
    {
        return joint_positions_;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    std::vector<double> joint_positions_;
    bool joint_positions_received_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create a node to subscribe to /joint_states
    auto joint_state_listener = std::make_shared<JointStateListener>();

    // Spin in a separate thread to keep receiving joint states
    std::thread spinner([&]() {
        rclcpp::spin(joint_state_listener);
    });

    // Wait until joint positions are received
    while (!joint_state_listener->areJointPositionsReceived())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto const node = std::make_shared<rclcpp::Node>(
        "joint_hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto const logger = rclcpp::get_logger("hello_moveit");

    static const std::string PLANNING_GROUP = "alpha";
    
    // Initialize MoveIt with MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // Ensure that the robot model is loaded correctly
    auto robot_model = move_group.getRobotModel();
    if (!robot_model)
    {
        RCLCPP_ERROR(logger, "Failed to load the robot model. Ensure MoveIt is configured correctly.");
        rclcpp::shutdown();
        return -1;
    }
    RCLCPP_INFO(logger, "Robot model successfully loaded.");

    // Get the current joint positions from the /joint_states topic
    std::vector<double> joint_positions = joint_state_listener->getJointPositions();
    RCLCPP_INFO(logger, "Received joint positions from /joint_states:");
    for (size_t i = 0; i < joint_positions.size(); ++i)
    {
        RCLCPP_INFO(logger, "Joint %zu: %.3f", i, joint_positions[i]);
    }

    // Set the received joint positions as the current robot state
    moveit::core::RobotStatePtr current_state(new moveit::core::RobotState(robot_model));
    current_state->setJointGroupPositions(move_group.getName(), joint_positions);
    move_group.setStartState(*current_state);

    // Modify one of the joint positions
    joint_positions[2] += 0.5; // Increase axis_c by 0.5 radians
    move_group.setJointValueTarget(joint_positions);

    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    move_group.setPlanningTime(10.0);

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(logger, "Planning successful, executing the plan...");
        move_group.execute(my_plan);
    }
    else
    {
        RCLCPP_WARN(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
