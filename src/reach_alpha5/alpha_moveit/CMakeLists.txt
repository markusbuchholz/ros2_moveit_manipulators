cmake_minimum_required(VERSION 3.8)
project(alpha_moveit)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(control_msgs REQUIRED)


#move joints
add_executable(joint_moveit src/joint_moveit.cpp)
add_executable(read_pos_joint_moveit src/read_pos_joint_moveit.cpp)

ament_target_dependencies(joint_moveit "moveit_ros_planning_interface" "rclcpp" std_srvs rclcpp rclcpp_action rclcpp_components control_msgs std_msgs sensor_msgs)
ament_target_dependencies(read_pos_joint_moveit "moveit_ros_planning_interface" "rclcpp" std_srvs rclcpp rclcpp_action rclcpp_components control_msgs std_msgs sensor_msgs)


install(TARGETS
  joint_moveit
  read_pos_joint_moveit
  DESTINATION lib/${PROJECT_NAME})

ament_package()




