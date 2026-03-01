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
find_package(serial QUIET)
find_package(graph_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pinocchio REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(control_msgs REQUIRED)

# Include directories
include_directories(include)

# 添加工具库
add_library(utils src/utils.cpp)
ament_target_dependencies(utils
  rclcpp
  moveit_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  graph_msgs
  moveit_core
)
target_link_libraries(utils pinocchio::pinocchio)

if(serial_FOUND)
  message(STATUS "serial library found - building hardware targets")

  add_library(usb_data src/usb_data.cpp)
  ament_target_dependencies(usb_data
    rclcpp
    moveit_msgs
    trajectory_msgs
    serial
  )
else()
  message(WARNING "serial library NOT found - hardware targets (real_world, read_data) will not be built")
endif()

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
target_link_libraries(z_plan_movegroup utils)
ament_target_dependencies(z_plan_movegroup
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  trajectory_msgs
)

# Add executable for MoveGroupInterface version with robo_arm3
add_executable(z_plan_movegroup_robo_arm src/z_plan_movegroup_robo_arm.cpp)
target_link_libraries(z_plan_movegroup_robo_arm utils)
ament_target_dependencies(z_plan_movegroup_robo_arm
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  trajectory_msgs
)

if(serial_FOUND)
  # Add executable for real world situation
  add_executable(real_world src/real_world.cpp)
  target_link_libraries(real_world utils usb_data pinocchio::pinocchio)
  ament_target_dependencies(real_world
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    moveit_msgs
    trajectory_msgs
    serial
    tf2_eigen
  )

  # Add executable for real world new (controller-based control)
  add_executable(real_world_new src/real_world_new.cpp)
  target_link_libraries(real_world_new utils usb_data pinocchio::pinocchio)
  ament_target_dependencies(real_world_new
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    moveit_msgs
    trajectory_msgs
    serial
    tf2_eigen
    moveit_core
    moveit_servo
    control_msgs
  )

  # Add executable for servo-based real-time control
  add_executable(real_world_servo src/real_world_servo.cpp)
  target_link_libraries(real_world_servo utils usb_data pinocchio::pinocchio)
  ament_target_dependencies(real_world_servo
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    moveit_msgs
    trajectory_msgs
    serial
    tf2_eigen
    moveit_core
    moveit_servo
    control_msgs
  )

  add_executable(read_data src/read_data_test.cpp src/usb_data.cpp)
  ament_target_dependencies(read_data
    rclcpp
    sensor_msgs
    trajectory_msgs
    serial
  )
endif()

# Test node for constraint control verification (no hardware dependency)
add_executable(test_constraint src/test_constraint.cpp)
target_link_libraries(test_constraint utils pinocchio::pinocchio)
ament_target_dependencies(test_constraint
  rclcpp
  geometry_msgs
  sensor_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  moveit_core
)

# Install executables
set(_install_targets
  z_plan_node
  z_plan_movegroup
  z_plan_movegroup_robo_arm
  test_constraint
)
if(serial_FOUND)
  list(APPEND _install_targets real_world real_world_new real_world_servo read_data)
endif()

install(TARGETS ${_install_targets}
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
