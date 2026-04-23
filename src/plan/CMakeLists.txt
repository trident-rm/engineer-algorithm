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

# Include directories
include_directories(include)

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

if(serial_FOUND)
  message(STATUS "serial library found - building hardware targets")

  add_library(usb_data SHARED src/usb_data.cpp)
  ament_target_dependencies(usb_data
    rclcpp
    moveit_msgs
    trajectory_msgs
    serial
  )
  install(TARGETS usb_data
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_libraries(usb_data)
else()
  message(WARNING "serial library NOT found - hardware targets (real_world, read_data) will not be built")
endif()

# Add executable for TURN half-circle planning mode
add_executable(turn_movegroup src/turn_movegroup.cpp)
target_link_libraries(turn_movegroup utils)
ament_target_dependencies(turn_movegroup
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  trajectory_msgs
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
target_link_libraries(key_step_control utils)
ament_target_dependencies(key_step_control
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

# RViz-based simulation control node (no hardware dependency)
add_executable(rviz_simulation src/rviz_simulation.cpp)
target_link_libraries(rviz_simulation utils pinocchio::pinocchio)
ament_target_dependencies(rviz_simulation
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  moveit_core
  interactive_markers
  std_srvs
)

# Test node for gravity compensation comparison (no hardware dependency)
set(GRAVITY_COMP_C_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/generated/gravity_comp.c)
if(EXISTS ${GRAVITY_COMP_C_FILE})
  add_executable(test_gravity_comp
    src/test_gravity_comp.cpp
    ${GRAVITY_COMP_C_FILE}
  )
  target_link_libraries(test_gravity_comp utils pinocchio::pinocchio)
  ament_target_dependencies(test_gravity_comp
    rclcpp
    moveit_ros_planning_interface
    moveit_msgs
    moveit_core
  )
else()
  message(STATUS "gravity_comp.c not found, skipping target: test_gravity_comp")
endif()

# Install executables
set(_install_targets
  turn_movegroup
  turn_retreat
  key_step_control
  test_constraint
  rviz_simulation
)
if(TARGET test_gravity_comp)
  list(APPEND _install_targets test_gravity_comp)
endif()
if(serial_FOUND)
  # Gravity compensation debug node (requires serial)
  if(EXISTS ${GRAVITY_COMP_C_FILE})
    add_executable(gravity_comp_debug
      src/gravity_comp_debug.cpp
      ${GRAVITY_COMP_C_FILE}
    )
    target_link_libraries(gravity_comp_debug utils usb_data pinocchio::pinocchio)
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
    real_world real_world_new read_data)
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
