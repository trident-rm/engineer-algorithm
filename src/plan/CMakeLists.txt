cmake_minimum_required(VERSION 3.8)
project(plan)

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
find_package(visualization_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robo_arm_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)

# Include directories
include_directories(include)
# Directly include tools headers from installed location
include_directories(/home/hku/robo_arm/install/tools/include)

# Link to tools library built by colcon  
link_directories(/home/hku/robo_arm/install/tools/lib)

# 添加工具库
add_library(utils SHARED src/utils.cpp)
ament_target_dependencies(utils
  rclcpp
  moveit_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  graph_msgs
  moveit_core
)
target_link_libraries(utils pinocchio::pinocchio)
install(TARGETS utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_libraries(utils)

# Add executable for TURN half-circle planning mode
add_executable(turn_movegroup src/turn_movegroup.cpp)
target_link_libraries(turn_movegroup utils tools)
ament_target_dependencies(turn_movegroup
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  trajectory_msgs
  serial
)

# Add executable for retreat Cartesian path test
add_executable(turn_retreat src/turn_retreat.cpp)
target_link_libraries(turn_retreat utils)
ament_target_dependencies(turn_retreat
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  trajectory_msgs
)

# Add executable for half-auto up interactive test
add_executable(key_step_control src/key_step_control.cpp)
target_link_libraries(key_step_control utils tools)
ament_target_dependencies(key_step_control
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  trajectory_msgs
)

# Pyroki + MoveIt integration: subscribe /joint_states from pyroki_ik_node and run a chosen action
add_executable(control_with_pyroki src/control_with_pyroki.cpp)
target_link_libraries(control_with_pyroki utils)
ament_target_dependencies(control_with_pyroki
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  trajectory_msgs
)

if(serial_FOUND)
  # Add executable for real world situation
  add_executable(real_world src/real_world.cpp)
  target_link_libraries(real_world utils tools pinocchio::pinocchio)
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

add_executable(flow_with_pyroki src/flow_with_pyroki.cpp)
target_link_libraries(flow_with_pyroki geometric_shapes::geometric_shapes)
ament_target_dependencies(flow_with_pyroki
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2_ros
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  robo_arm_msgs
)

# Keyboard input node: reads stdin in raw mode and publishes /key_input (std_msgs/String).
# Run with: ros2 run plan keyboard_input  (must be ros2 run, not launch)
add_executable(keyboard_input src/keyboard_input.cpp)
ament_target_dependencies(keyboard_input rclcpp std_msgs)

  # Add executable for real world new (controller-based control)
  add_executable(real_world_new src/real_world_new.cpp)
  target_link_libraries(real_world_new utils tools pinocchio::pinocchio)
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
    control_msgs
  )

  add_executable(read_data src/read_data_test.cpp)
  target_link_libraries(read_data tools)
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

# Complete control node with real hardware (requires serial)
if(serial_FOUND)
  add_executable(complete_control src/complete_control.cpp)
  target_link_libraries(complete_control utils tools pinocchio::pinocchio)
  ament_target_dependencies(complete_control
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
    moveit_msgs
    trajectory_msgs
    moveit_core
    serial
    tf2_eigen
  )
endif()




# Install executables
set(_install_targets
  turn_movegroup
  turn_retreat
  key_step_control
  control_with_pyroki
  test_constraint
  flow_with_pyroki
  keyboard_input
)

if(serial_FOUND)
  # Gravity compensation debug node (requires serial)
  if(EXISTS ${GRAVITY_COMP_C_FILE})
    add_executable(gravity_comp_debug
      src/gravity_comp_debug.cpp
      ${GRAVITY_COMP_C_FILE}
    )
    target_link_libraries(gravity_comp_debug utils tools pinocchio::pinocchio)
    ament_target_dependencies(gravity_comp_debug
      rclcpp
      sensor_msgs
      moveit_ros_planning_interface
      moveit_msgs
      moveit_core
      serial
    )
  else()
    message(STATUS "gravity_comp.c not found, skipping target: gravity_comp_debug")
  endif()

  list(APPEND _install_targets
    real_world real_world_new read_data complete_control)
  if(TARGET gravity_comp_debug)
    list(APPEND _install_targets gravity_comp_debug)
  endif()
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
