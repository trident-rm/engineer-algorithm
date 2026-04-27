cmake_minimum_required(VERSION 3.8)
project(tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(serial REQUIRED)

# Include directories
include_directories(include)

# Create shared library
add_library(tools SHARED src/usb_data.cpp)

ament_target_dependencies(tools
  rclcpp
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2_eigen
  serial
)

target_link_libraries(tools Eigen3::Eigen)

# Install library
install(TARGETS tools
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(FILES include/usb_data.hpp
  DESTINATION include/tools
)

# Export library for ament
ament_export_libraries(tools)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  trajectory_msgs
  moveit_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2_eigen
  Eigen3
  serial
)

ament_package()
