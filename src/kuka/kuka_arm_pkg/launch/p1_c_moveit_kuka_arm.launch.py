from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
import xacro
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Path to Robot's Xacro File
    pkg_path= get_package_share_directory("kuka_arm_pkg")
    xacro_file= os.path.join(pkg_path,'urdf','kr210.urdf.xacro')

    #Processing Xacro File
    xacro_parser=xacro.parse(open(xacro_file))
    xacro.process_doc(xacro_parser)

    #Feeding URDF to ROS
    parameters = {'robot_description': xacro_parser.toxml()}


    #Ros Packages
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[parameters]
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    gazebo_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={
                "use_sim_time" : use_sim_time,
                "verbose"      : "true",
                }.items()
             )


    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic','robot_description',
            '-entity','kuka_arm',

        ],
    )
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    kuka_arm_controller_executor = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'kuka_arm_controller'],
        output='screen'
    )

    kuka_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'kuka_gripper_controller'],
        output='screen'
    )

    # Moveit URDF loading with controllers
    moveit_config_node = (
        MoveItConfigsBuilder("kuka_arm_moveit_ws",package_name="kuka_arm_moveit")
        .robot_description(file_path="config/kr210_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/kr210_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # Moveit action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_node.to_dict(), {"use_sim_time" : use_sim_time}],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("kuka_arm_moveit"), "config"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config_node.robot_description,
            moveit_config_node.robot_description_semantic,
            moveit_config_node.planning_pipelines,
            moveit_config_node.robot_description_kinematics,
            {"use_sim_time" : use_sim_time},
        ],
    )



    #Running all definations
    nodes_to_run = [
        robot_state_publisher_node,
        gazebo_node,
        spawn_robot_node,
        load_joint_state_controller,
        kuka_arm_controller_executor,
        kuka_gripper_controller,
        move_group_node,
        rviz_node


    ]


    return LaunchDescription(nodes_to_run)