cmake_minimum_required(VERSION 3.8)
project(z_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)

# Include directories
include_directories(include)

# 添加工具库
add_library(trajectory_utils src/trajectory_utils.cpp)
ament_target_dependencies(trajectory_utils
  rclcpp
  moveit_msgs
  trajectory_msgs
)

# Add executable for MTC version
add_executable(z_plan_node src/z_plan_node.cpp)
ament_target_dependencies(z_plan_node
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_visual_tools
)
target_include_directories(z_plan_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(z_plan_node Eigen3::Eigen)

# Add executable for MoveGroupInterface version
add_executable(z_plan_movegroup src/z_plan_movegroup.cpp)
target_link_libraries(z_plan_movegroup trajectory_utils)
ament_target_dependencies(z_plan_movegroup
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  trajectory_msgs
)

# Install executables
install(TARGETS
  z_plan_node
  z_plan_movegroup
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
