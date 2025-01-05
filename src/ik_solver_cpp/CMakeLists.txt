cmake_minimum_required(VERSION 3.5)
project(ik_solver_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(Glog QUIET)  # If your system needs it

# Create the node executable
add_executable(ik_solver_node src/ik_solver_node.cpp)

# ament_target_dependencies uses the plain signature
ament_target_dependencies(
  ik_solver_node
  rclcpp
  std_msgs
  trajectory_msgs
  builtin_interfaces
  Eigen3
  Ceres
)

# Link Ceres (and possibly Glog) with the plain signature
# (no PRIVATE/PUBLIC/INTERFACE keywords allowed)
if(Glog_FOUND)
  target_link_libraries(ik_solver_node
    ${CERES_LIBRARIES}
    glog::glog
  )
else()
  target_link_libraries(ik_solver_node
    ${CERES_LIBRARIES}
  )
endif()

install(TARGETS
  ik_solver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
