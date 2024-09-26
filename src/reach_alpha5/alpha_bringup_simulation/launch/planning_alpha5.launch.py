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
    Generate a launch description to run MoveIt with the Reach Alpha 5 manipulator.

    Returns:
        LaunchDescription: ROS 2 launch description
    """
    # Declare arguments
    args = [
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

    ]

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    #use_rviz = LaunchConfiguration("use_rviz")
    use_sim = LaunchConfiguration("use_sim")
    prefix = LaunchConfiguration("prefix")
    namespace = LaunchConfiguration("namespace")

    # Get the robot description using xacro
    robot_description = {
        "robot_description": Command(
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
    }

    # Get the robot semantic description using xacro
    robot_description_semantic = {
        "robot_description_semantic": Command(
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
    }

    # Load the moveit configuration files
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

    # Configure MoveIt2 settings
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

    # Declare ROS 2 nodes
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
            {"use_sim_time": True},
        ],
    )

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
            {"use_sim_time": True},
        ],
       # condition=IfCondition(use_rviz),
    )

        # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

        # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

        # ros2_control using FakeSystem as hardware
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

    alpha_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "xsubsea",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    nodes = [
        rviz_node,
        robot_state_publisher,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        alpha_controller_spawner,
    ]

    return LaunchDescription(args + nodes)
