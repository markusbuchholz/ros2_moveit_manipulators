import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description to run MoveIt with the Reach Alpha 5 manipulator,
    including both Joint Trajectory and Joint Velocity Controllers.
    
    Returns:
        LaunchDescription: ROS 2 launch description
    """
    # =====================
    # 1. Declare Launch Arguments
    # =====================
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="alpha_description_simulation",
            description=(
                "The description package with the Alpha URDF files. This is typically"
                " not set, but is available in case another description package has"
                " been defined."
            ),
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="alpha.config.xacro",
            description="The URDF/XACRO description file with the Alpha.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description=(
                "The prefix of the joint names. This is useful for multi-robot setups."
                " If the prefix is changed, then the joint names in the controller"
                " configuration must be updated. Expected format '<prefix>/'."
            ),
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description=(
                "The namespace of the launched nodes. This is useful for multi-robot"
                " setups. If the namespace is changed, then the namespace in the"
                " controller configuration must be updated. Expected format '<ns>/'."
            ),
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Flag to enable use_sim_time if running in simulation.",
        ),
    ]

    # =====================
    # 2. Initialize Launch Configurations
    # =====================
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    namespace = LaunchConfiguration("namespace")
    use_sim = LaunchConfiguration("use_sim")

    # =====================
    # 3. Robot Description and Semantic Description
    # =====================
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "config", description_file]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "config",
                    "alpha.config.srdf.xacro",
                ]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "description_package:=",
            description_package,
            " ",
            "namespace:=",
            namespace,
        ]
    )

    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # =====================
    # 4. MoveIt2 Configuration Files
    # =====================
    robot_description_planning_joint_limits = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "joint_limits.yaml",
        ]
    )

    robot_description_planning_cartesian_limits = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "pilz_cartesian_limits.yaml",
        ]
    )

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(description_package), "moveit2", "kinematics.yaml"]
    )

    planning_pipelines_config = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "planning_pipelines.yaml",
        ]
    )

    ompl_planning_config = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "ompl_planning.yaml",
        ]
    )

    moveit_controllers = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "moveit2",
            "moveit_controllers.yaml",
        ]
    )

    # =====================
    # 5. MoveIt2 Parameters
    # =====================
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.0,
        "trajectory_execution.allowed_start_tolerance": 2.1,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    # =====================
    # 6. Define ROS 2 Nodes
    # =====================

    # 6.1 Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace=namespace,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipelines_config,
            trajectory_execution,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            moveit_controllers,
            ompl_planning_config,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim},
        ],
    )

    # 6.2 RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "rviz", "moveit.rviz"]
            ),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning_cartesian_limits,
            robot_description_planning_joint_limits,
            robot_description_kinematics,
            planning_pipelines_config,
            ompl_planning_config,
            {"use_sim_time": use_sim},
        ],
        # condition=IfCondition(use_rviz),  # Uncomment if you want to make RViz optional
    )

    # 6.3 Static Transform Publisher (TF)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # 6.4 Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim}],
    )

    # 6.5 ros2_control Node
    ros2_controllers_path = os.path.join(
        get_package_share_directory("alpha_description_simulation"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
    )

    # 6.6 Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 6.7 Joint Trajectory Controller (xsubsea) Spawner
    xsubsea_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "xsubsea",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 6.8 Joint Velocity Controller Spawner
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # =====================
    # 7. Aggregate All Nodes
    # =====================
    nodes_to_start = [
        rviz_node,
        robot_state_publisher_node,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        xsubsea_controller_spawner,
        velocity_controller_spawner,  # Added Velocity Controller Spawner
        static_tf,  # Optional: Include if static transforms are needed
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
