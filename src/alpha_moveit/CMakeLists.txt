cmake_minimum_required(VERSION 3.8)
project(alpha_moveit)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(joint_moveit src/joint_moveit.cpp)
ament_target_dependencies(joint_moveit
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_visual_tools
  geometry_msgs
  control_msgs
)

add_executable(ik_moveit src/ik_moveit.cpp)
ament_target_dependencies(ik_moveit
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_visual_tools
  geometry_msgs
  control_msgs
)



install(TARGETS
  joint_moveit
  ik_moveit
  DESTINATION lib/${PROJECT_NAME})

ament_package()
