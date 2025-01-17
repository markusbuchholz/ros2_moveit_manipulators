cmake_minimum_required(VERSION 3.8)
project(kuka_arm_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core  REQUIRED)
find_package(graph_msgs  REQUIRED)

set(Moveit_includes
rclcpp
moveit_ros_planning
moveit_ros_planning_interface
moveit_core
graph_msgs
)



add_executable(p1_a_jtc_node src/p1_a_joint_trajectory_controller.cpp)
ament_target_dependencies(p1_a_jtc_node rclcpp trajectory_msgs)

add_executable(p1_b_moveit_drive src/p1_b_motion_drive_moveit.cpp)
ament_target_dependencies(p1_b_moveit_drive ${Moveit_includes})

add_executable(ik_kuka src/ik_kuka.cpp)
ament_target_dependencies(ik_kuka ${Moveit_includes})

add_executable(ik_x_kuka src/ik_x_kuka.cpp)
ament_target_dependencies(ik_x_kuka ${Moveit_includes})

# add_executable(ik_read src/ik_read.cpp)
# ament_target_dependencies(ik_read ${Moveit_includes})

add_executable(move_kuka src/move_kuka.cpp)
ament_target_dependencies(move_kuka ${Moveit_includes})


install(TARGETS
  p1_a_jtc_node
  p1_b_moveit_drive
  ik_kuka
  ik_x_kuka
  #ik_read
  move_kuka  
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY src urdf meshes config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
