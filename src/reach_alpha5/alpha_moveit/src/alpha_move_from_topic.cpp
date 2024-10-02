// ros2 topic pub /alpha_move_joint std_msgs/msg/Float64MultiArray "{data: [0.0, 1.5746, 0.0, 1.5746, 0.0]}"


#include <memory>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <chrono>
#include <thread>

class MoveitSubscriber : public rclcpp::Node
{
public:
    MoveitSubscriber(const rclcpp::Node::SharedPtr& node) : Node("moveit_subscriber")
    {
        static const std::string PLANNING_GROUP = "alpha";
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, PLANNING_GROUP);

        joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/alpha_move_joint", 10, std::bind(&MoveitSubscriber::jointCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Waiting for joint positions...");
    }

private:
    void jointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received new joint positions.");
        
        std::vector<double> joint_positions = msg->data;
        
        if (joint_positions.size() != 5)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid joint positions received!");
            return;
        }

        move_group_->setJointValueTarget(joint_positions);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Motion plan successful, executing...");
            move_group_->execute(my_plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan motion.");
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("moveit_subscriber_node");

    auto moveit_subscriber = std::make_shared<MoveitSubscriber>(node);

    rclcpp::spin(moveit_subscriber);
    rclcpp::shutdown();
    return 0;
}
