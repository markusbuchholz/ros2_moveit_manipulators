//Markus Buchholz, 2025

// ik_solver_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <vector>
#include <cmath>

using std::placeholders::_1;

Eigen::Matrix4d rot_x(double theta) {
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
    R(1,1) = cos(theta);
    R(1,2) = -sin(theta);
    R(2,1) = sin(theta);
    R(2,2) = cos(theta);
    return R;
}

Eigen::Matrix4d rot_y(double theta) {
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
    R(0,0) = cos(theta);
    R(0,2) = sin(theta);
    R(2,0) = -sin(theta);
    R(2,2) = cos(theta);
    return R;
}

Eigen::Matrix4d rot_z(double theta) {
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
    R(0,0) = cos(theta);
    R(0,1) = -sin(theta);
    R(1,0) = sin(theta);
    R(1,1) = cos(theta);
    return R;
}

Eigen::Matrix4d trans(double x, double y, double z) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0,3) = x;
    T(1,3) = y;
    T(2,3) = z;
    return T;
}

Eigen::Vector3d forward_kinematics(const std::vector<double>& joint_angles) {
    std::vector<Eigen::Vector3d> joint_axes = {
        Eigen::Vector3d(0, 0, 1),
        Eigen::Vector3d(0, 1, 0),
        Eigen::Vector3d(0, 1, 0),
        Eigen::Vector3d(0, 0, -1)
    };

    std::vector<Eigen::Vector3d> joint_origins = {
        Eigen::Vector3d(0, 0, 0.014),
        Eigen::Vector3d(-0.02, 0, 0.033),
        Eigen::Vector3d(-0.04, 0, -0.1453),
        Eigen::Vector3d(0.02, 0, 0.033)
    };

    std::vector<Eigen::Vector3d> joint_rpys = {
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(0, 0, 3.14159),
        Eigen::Vector3d(0, 0, 2.09439)
    };

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    for (size_t i = 0; i < joint_angles.size(); ++i) {
        Eigen::Matrix4d T_joint = Eigen::Matrix4d::Identity();
        T_joint(0,3) = joint_origins[i].x();
        T_joint(1,3) = joint_origins[i].y();
        T_joint(2,3) = joint_origins[i].z();

        Eigen::Matrix4d R_fixed = rot_z(joint_rpys[i].z())
                                * rot_y(joint_rpys[i].y())
                                * rot_x(joint_rpys[i].x());
        T_joint = T_joint * R_fixed;

        Eigen::Matrix4d R_joint;
        if (joint_axes[i].isApprox(Eigen::Vector3d(1, 0, 0))) {
            R_joint = rot_x(joint_angles[i]);
        } else if (joint_axes[i].isApprox(Eigen::Vector3d(0, 1, 0))) {
            R_joint = rot_y(joint_angles[i]);
        } else if (joint_axes[i].isApprox(Eigen::Vector3d(0, 0, 1))) {
            R_joint = rot_z(joint_angles[i]);
        } else if (joint_axes[i].isApprox(Eigen::Vector3d(0, 0, -1))) {
            R_joint = rot_z(-joint_angles[i]);
        } else {
            throw std::invalid_argument("Unsupported joint axis");
        }

        T_joint = T_joint * R_joint;
        T = T * T_joint;
    }

    // End-effector translation
    Eigen::Matrix4d T_ee = trans(0, 0, 0.09975);
    T = T * T_ee;

    Eigen::Vector3d position = T.block<3,1>(0,3);
    return position;
}

//-----------------------------------------------------------------------------
// Cost functor for Ceres
//-----------------------------------------------------------------------------

struct IKCostFunction {
    IKCostFunction(const Eigen::Vector3d& target_position)
        : target_position_(target_position) {}

    // We want a 3D residual (x,y,z) and 4 unknowns (the 4 joint angles)
    template <typename T>
    bool operator()(const T* const joint_angles, T* residuals) const
    {
        // Joint definitions as T
        std::vector<Eigen::Matrix<T,3,1>> joint_axes = {
            Eigen::Matrix<T,3,1>(T(0), T(0), T(1)),
            Eigen::Matrix<T,3,1>(T(0), T(1), T(0)),
            Eigen::Matrix<T,3,1>(T(0), T(1), T(0)),
            Eigen::Matrix<T,3,1>(T(0), T(0), T(-1))
        };

        std::vector<Eigen::Matrix<T,3,1>> joint_origins = {
            Eigen::Matrix<T,3,1>(T(0), T(0), T(0.014)),
            Eigen::Matrix<T,3,1>(T(-0.02), T(0), T(0.033)),
            Eigen::Matrix<T,3,1>(T(-0.04), T(0), T(-0.1453)),
            Eigen::Matrix<T,3,1>(T(0.02), T(0), T(0.033))
        };

        std::vector<Eigen::Matrix<T,3,1>> joint_rpys = {
            Eigen::Matrix<T,3,1>(T(0), T(0), T(0)),
            Eigen::Matrix<T,3,1>(T(0), T(0), T(0)),
            Eigen::Matrix<T,3,1>(T(0), T(0), T(3.14159)),
            Eigen::Matrix<T,3,1>(T(0), T(0), T(2.09439))
        };

        // Identity
        Eigen::Matrix<T,4,4> T_current = Eigen::Matrix<T,4,4>::Identity();

        // Lambda expressions for rotation
        auto rotX = [](const T theta) -> Eigen::Matrix<T,4,4> {
            Eigen::Matrix<T,4,4> R = Eigen::Matrix<T,4,4>::Identity();
            R(1,1) = cos(theta);
            R(1,2) = -sin(theta);
            R(2,1) = sin(theta);
            R(2,2) = cos(theta);
            return R;
        };

        auto rotY = [](const T theta) -> Eigen::Matrix<T,4,4> {
            Eigen::Matrix<T,4,4> R = Eigen::Matrix<T,4,4>::Identity();
            R(0,0) = cos(theta);
            R(0,2) = sin(theta);
            R(2,0) = -sin(theta);
            R(2,2) = cos(theta);
            return R;
        };

        auto rotZ = [](const T theta) -> Eigen::Matrix<T,4,4> {
            Eigen::Matrix<T,4,4> R = Eigen::Matrix<T,4,4>::Identity();
            R(0,0) = cos(theta);
            R(0,1) = -sin(theta);
            R(1,0) = sin(theta);
            R(1,1) = cos(theta);
            return R;
        };

        for (size_t i = 0; i < 4; ++i) {
            // Translation
            Eigen::Matrix<T,4,4> T_joint = Eigen::Matrix<T,4,4>::Identity();
            T_joint(0,3) = joint_origins[i].x();
            T_joint(1,3) = joint_origins[i].y();
            T_joint(2,3) = joint_origins[i].z();

            // Fixed rotation
            Eigen::Matrix<T,4,4> R_fixed = rotZ(joint_rpys[i].z()) *
                                           rotY(joint_rpys[i].y()) *
                                           rotX(joint_rpys[i].x());
            T_joint = T_joint * R_fixed;

            // Joint rotation
            Eigen::Matrix<T,4,4> R_joint = Eigen::Matrix<T,4,4>::Identity();
            if (joint_axes[i].isApprox(Eigen::Matrix<T,3,1>(T(1), T(0), T(0)))) {
                R_joint = rotX(joint_angles[i]);
            } else if (joint_axes[i].isApprox(Eigen::Matrix<T,3,1>(T(0), T(1), T(0)))){
                R_joint = rotY(joint_angles[i]);
            } else if (joint_axes[i].isApprox(Eigen::Matrix<T,3,1>(T(0), T(0), T(1)))){
                R_joint = rotZ(joint_angles[i]);
            } else if (joint_axes[i].isApprox(Eigen::Matrix<T,3,1>(T(0), T(0), T(-1)))){
                R_joint = rotZ(-joint_angles[i]);
            } else {
                // Unsupported axis
                return false;
            }

            T_joint = T_joint * R_joint;
            T_current = T_current * T_joint;
        }

        Eigen::Matrix<T,4,4> T_ee = Eigen::Matrix<T,4,4>::Identity();
        T_ee(2,3) = T(0.09975);  // FIXED
        T_current = T_current * T_ee;

        Eigen::Matrix<T,3,1> position = T_current.template block<3,1>(0,3);

        residuals[0] = position.x() - T(target_position_.x());
        residuals[1] = position.y() - T(target_position_.y());
        residuals[2] = position.z() - T(target_position_.z());

        return true;
    }

private:
    Eigen::Vector3d target_position_;
};

std::vector<Eigen::Vector3d> interpolate_points(const Eigen::Vector3d& start,
                                                const Eigen::Vector3d& end,
                                                int num_points)
{
    std::vector<Eigen::Vector3d> points;
    points.reserve(num_points);

    for (int i = 0; i < num_points; ++i) {
        double alpha = static_cast<double>(i) / (num_points - 1);
        points.emplace_back(start + alpha * (end - start));
    }
    return points;
}

class IKSolverNode : public rclcpp::Node {
public:
    IKSolverNode()
    : Node("ik_solver"), is_executing_(false)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/target_position",
            10,
            std::bind(&IKSolverNode::target_position_callback, this, _1)
        );

        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/xsubsea/joint_trajectory",
            10
        );

        initial_guess_ = {4.0, 0.1, 0.1, 0.1};

        joint_names_ = {"axis_a", "axis_b", "axis_c", "axis_d", "axis_e"};

        RCLCPP_INFO(this->get_logger(), "Inverse Kinematics Solver Node with linear motion has been started.");
    }

private:
    void target_position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (is_executing_) {
            RCLCPP_WARN(this->get_logger(), "Already executing a trajectory. Ignoring new target.");
            return;
        }

        if (msg->data.size() != 3) {
            RCLCPP_ERROR(this->get_logger(), "Received target_position does not have exactly 3 elements.");
            return;
        }

        Eigen::Vector3d target_position;
        target_position << msg->data[0], msg->data[1], msg->data[2];
        RCLCPP_INFO(this->get_logger(), "Received target_position: [%.4f, %.4f, %.4f]", 
                    target_position.x(), target_position.y(), target_position.z());

        Eigen::Vector3d current_position = forward_kinematics(initial_guess_);
        RCLCPP_INFO(this->get_logger(), "Current end-effector position: [%.4f, %.4f, %.4f]", 
                    current_position.x(), current_position.y(), current_position.z());

        // Interpolate points
        //------------------------------//
        //------------------------------//
        int num_points = 15; //number of interpoloated points
        double time_step = 0.5; // seconds between points
        //------------------------------//
        //------------------------------//
        std::vector<Eigen::Vector3d> points = interpolate_points(current_position, target_position, num_points);
        RCLCPP_INFO(this->get_logger(), "Interpolated %d points for linear motion.", num_points);

        // Create JointTrajectory message
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = joint_names_;

        for (int i = 0; i < static_cast<int>(points.size()); ++i) {
            std::vector<double> joint_angles = inverse_kinematics(points[i], initial_guess_);

            if (!joint_angles.empty()) {
                // Reverse joint angles and insert axis_a fixed at 0.0
                std::vector<double> joint_angles_reversed(joint_angles.rbegin(), joint_angles.rend());
                joint_angles_reversed.insert(joint_angles_reversed.begin(), 0.0); // axis_a

                RCLCPP_INFO(this->get_logger(),
                            "Point %d/%d: [%.4f, %.4f, %.4f]",
                            i+1, num_points,
                            points[i].x(), points[i].y(), points[i].z());

                std::string joint_angles_str;
                for (const auto& angle : joint_angles_reversed) {
                    joint_angles_str += std::to_string(angle) + " ";
                }
                RCLCPP_INFO(this->get_logger(), "Joint angles (radians): %s", joint_angles_str.c_str());

                trajectory_msgs::msg::JointTrajectoryPoint traj_point;
                traj_point.positions = joint_angles_reversed;

                builtin_interfaces::msg::Duration duration;
                duration.sec = static_cast<int>(i * time_step);
                duration.nanosec = static_cast<int>((i * time_step - duration.sec) * 1e9);
                traj_point.time_from_start = duration;

                traj_msg.points.push_back(traj_point);

                initial_guess_ = joint_angles;
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to compute IK for point %d: [%.4f, %.4f, %.4f]", 
                             i+1, points[i].x(), points[i].y(), points[i].z());
                return;
            }
        }

        publisher_->publish(traj_msg);
        RCLCPP_INFO(this->get_logger(), "Published JointTrajectory with %d points for linear motion.", num_points);

        Eigen::Vector3d computed_position = forward_kinematics(initial_guess_);
        double position_error = (computed_position - target_position).norm();

        RCLCPP_INFO(this->get_logger(), "--- Verification ---");
        RCLCPP_INFO(this->get_logger(),
                    "Computed end-effector position after trajectory: [%.6f, %.6f, %.6f]", 
                    computed_position.x(), computed_position.y(), computed_position.z());
        RCLCPP_INFO(this->get_logger(),
                    "Target end-effector position: [%.6f, %.6f, %.6f]", 
                    target_position.x(), target_position.y(), target_position.z());
        RCLCPP_INFO(this->get_logger(), "Position error: %.6f", position_error);

        if (position_error < 1e-4) {
            RCLCPP_INFO(this->get_logger(), "The IK solution is valid.");
        } else {
            RCLCPP_WARN(this->get_logger(), "The IK solution has errors.");
        }
    }

    // Inverse Kinematics function using Ceres Solver
    std::vector<double> inverse_kinematics(const Eigen::Vector3d& target_position,
                                           const std::vector<double>& initial_guess)
    {
        // Define joint limits
        struct JointLimit {
            double lower;
            double upper;
        };

        std::vector<JointLimit> joint_limits = {
            {2.5, 4.0},  // axis_e
            {0.2, 2.0},  // axis_d
            {0.2, 2.2},  // axis_c
            {0.0, 2.0}   // axis_b
            // axis_a is fixed at 0.0
        };

        // Initial guess (only for axis_b to axis_e)
        double angles[4];
        for (size_t i = 0; i < 4; ++i) {
            angles[i] = initial_guess[i];
        }

        // Build the problem
        ceres::Problem problem;

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<IKCostFunction, 3, 4>(
                new IKCostFunction(target_position)  // FIXED
            );

        problem.AddResidualBlock(cost_function, nullptr, angles);

        // Set bounds for joint angles
        for (size_t i = 0; i < 4; ++i) {
            problem.SetParameterLowerBound(angles, i, joint_limits[i].lower);
            problem.SetParameterUpperBound(angles, i, joint_limits[i].upper);
        }

        // Configure solver
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        options.max_num_iterations = 10000;
        options.function_tolerance = 1e-6;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        if (summary.IsSolutionUsable()) {
            return {angles[0], angles[1], angles[2], angles[3]};
        } else {
            RCLCPP_ERROR(this->get_logger(), "Ceres Solver failed: %s", summary.BriefReport().c_str());
            return {};
        }
    }

    // Forward Kinematics using Eigen
    Eigen::Vector3d forward_kinematics(const std::vector<double>& joint_angles) {
        std::vector<Eigen::Vector3d> joint_axes = {
            Eigen::Vector3d(0, 0, 1),
            Eigen::Vector3d(0, 1, 0),
            Eigen::Vector3d(0, 1, 0),
            Eigen::Vector3d(0, 0, -1)
        };

        std::vector<Eigen::Vector3d> joint_origins = {
            Eigen::Vector3d(0, 0, 0.014),
            Eigen::Vector3d(-0.02, 0, 0.033),
            Eigen::Vector3d(-0.04, 0, -0.1453),
            Eigen::Vector3d(0.02, 0, 0.033)
        };

        std::vector<Eigen::Vector3d> joint_rpys = {
            Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(0, 0, 0),
            Eigen::Vector3d(0, 0, 3.14159),
            Eigen::Vector3d(0, 0, 2.09439)
        };

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        for (size_t i = 0; i < joint_angles.size(); ++i) {
            Eigen::Matrix4d T_joint = Eigen::Matrix4d::Identity();
            T_joint(0,3) = joint_origins[i].x();
            T_joint(1,3) = joint_origins[i].y();
            T_joint(2,3) = joint_origins[i].z();

            Eigen::Matrix4d R_fixed = rot_z(joint_rpys[i].z())
                                    * rot_y(joint_rpys[i].y())
                                    * rot_x(joint_rpys[i].x());
            T_joint = T_joint * R_fixed;

            Eigen::Matrix4d R_joint;
            if (joint_axes[i].isApprox(Eigen::Vector3d(1, 0, 0))) {
                R_joint = rot_x(joint_angles[i]);
            } else if (joint_axes[i].isApprox(Eigen::Vector3d(0, 1, 0))) {
                R_joint = rot_y(joint_angles[i]);
            } else if (joint_axes[i].isApprox(Eigen::Vector3d(0, 0, 1))) {
                R_joint = rot_z(joint_angles[i]);
            } else if (joint_axes[i].isApprox(Eigen::Vector3d(0, 0, -1))) {
                R_joint = rot_z(-joint_angles[i]);
            } else {
                throw std::invalid_argument("Unsupported joint axis");
            }

            T_joint = T_joint * R_joint;
            T = T * T_joint;
        }

        Eigen::Matrix4d T_ee = trans(0, 0, 0.09975);
        T = T * T_ee;

        Eigen::Vector3d position = T.block<3,1>(0,3);
        return position;
    }

    std::vector<Eigen::Vector3d> interpolate_points(const Eigen::Vector3d& start,
                                                    const Eigen::Vector3d& end,
                                                    int num_points) {
        std::vector<Eigen::Vector3d> points;
        points.reserve(num_points);
        for (int i = 0; i < num_points; ++i) {
            double alpha = static_cast<double>(i) / (num_points - 1);
            points.emplace_back(start + alpha * (end - start));
        }
        return points;
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    std::vector<double> initial_guess_;
    std::vector<std::string> joint_names_;
    bool is_executing_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IKSolverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
