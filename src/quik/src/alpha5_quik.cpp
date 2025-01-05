

#include "rclcpp/rclcpp.hpp"
#include "quik/IKSolver.hpp"
#include "quik/Robot.hpp"
#include "quik/ros_helpers.hpp"
#include "quik/utilities.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>

// Define DOF at compile time for more speed and no dynamic memory allocation
constexpr int DOF = 4;

using namespace Eigen;
using namespace std::chrono_literals;

class SampleCPPUsageNode : public rclcpp::Node
{
public:
    SampleCPPUsageNode() : Node("sample_cpp_usage_node")
    {
        // Initialize robot from parameters
        try {
            this->R = std::make_shared<quik::Robot<DOF>>(quik::ros_helpers::robotFromNodeParameters<DOF>(*this));
            RCLCPP_INFO(this->get_logger(), "Loaded robot successfully. Robot configuration is:");
            this->R->print();
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot: %s", e.what());
            throw;
        }

        // Initialize IK Solver from parameters
        try {
            this->IKS = std::make_shared<quik::IKSolver<DOF>>(quik::ros_helpers::IKSolverFromNodeParameters<DOF>(*this, this->R));
            RCLCPP_INFO(this->get_logger(), "Built IKSolver object. Configuration is:");
            this->IKS->printOptions();
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to build IKSolver: %s", e.what());
            throw;
        }

        // Subscribe to /target_position topic
        this->subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/target_position",
            10,
            std::bind(&SampleCPPUsageNode::target_position_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to /target_position topic.");

        // Optionally, initialize current joint angles to zeros or some default
        current_joint_angles_ = Vector<double, DOF>::Zero();

        RCLCPP_INFO(this->get_logger(), "IK Solver Node has been started and is ready to receive target positions.");
    }

private:
    void target_position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Validate message size
        if (msg->data.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "Received target_position does not have exactly 3 elements.");
            return;
        }

        // Extract target position
        Vector3d target_position;
        target_position << msg->data[0], msg->data[1], msg->data[2];
        RCLCPP_INFO(this->get_logger(), "Received target_position: [%.4f, %.4f, %.4f]", 
                    target_position.x(), target_position.y(), target_position.z());

        // Define desired orientation (quaternion)
        // Here, we assume no rotation: [x, y, z, w] = [0, 0, 0, 1]
        Vector4d desired_quat;
        desired_quat << 0.0, 0.0, 0.0, 1.0;

        // Optional: If you have a desired orientation, set it here
        // For example, a rotation of 90 degrees about Z-axis:
        // double angle = M_PI / 2;
        // desired_quat << 0.0, 0.0, sin(angle/2), cos(angle/2);

        // Use current joint angles as the initial guess
        Vector<double, DOF> initial_guess = current_joint_angles_;

        // Perform Forward Kinematics to get current end-effector pose (optional)
        Vector4d current_quat;
        Vector3d current_position;
        this->R->FKn(initial_guess, current_quat, current_position);
        RCLCPP_INFO(this->get_logger(), "Current End-Effector Pose: \n\tQuaternion: [%.4f, %.4f, %.4f, %.4f]\n\tPosition: [%.4f, %.4f, %.4f]",
                    current_quat.x(), current_quat.y(), current_quat.z(), current_quat.w(),
                    current_position.x(), current_position.y(), current_position.z());

        // Run inverse kinematics
        Vector<double, DOF> computed_joint_angles;
        Vector<double, 6> error;
        int iterations;
        quik::BREAKREASON_t break_reason;

        auto start_time = std::chrono::high_resolution_clock::now();
        this->IKS->IK(desired_quat, target_position, initial_guess, computed_joint_angles, error, iterations, break_reason);
        auto end_time = std::chrono::high_resolution_clock::now();

        // Calculate elapsed time in microseconds
        std::chrono::duration<double, std::micro> elapsed = end_time - start_time;

        // Check if IK was successful
        bool success = (break_reason == quik::BREAKREASON_TOLERANCE);
        if (success) {
            RCLCPP_INFO(this->get_logger(), "IK succeeded.");
        } else {
            RCLCPP_WARN(this->get_logger(), "IK did not converge. Reason: %s", quik::breakreason2str(break_reason).c_str());
        }

        // Print computed joint angles
        std::string joint_angles_str = quik::utilities::eigen2str(computed_joint_angles.transpose());
        RCLCPP_INFO(this->get_logger(), "Computed Joint Angles (radians): %s", joint_angles_str.c_str());

        // Print error norm and iterations
        RCLCPP_INFO(this->get_logger(), "Normed error: %.6f", error.norm());
        RCLCPP_INFO(this->get_logger(), "Iterations: %d", iterations);
        RCLCPP_INFO(this->get_logger(), "Elapsed time: %.2f microseconds.", elapsed.count());

        // Update current joint angles with the computed solution for future initial guesses
        if (success) {
            current_joint_angles_ = computed_joint_angles;
        }
    }

    // Members
    std::shared_ptr<quik::IKSolver<DOF>> IKS;
    std::shared_ptr<quik::Robot<DOF>> R;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    Vector<double, DOF> current_joint_angles_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SampleCPPUsageNode>());
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in node: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
