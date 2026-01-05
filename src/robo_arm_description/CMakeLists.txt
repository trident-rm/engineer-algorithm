cmake_minimum_required(VERSION 3.8)
project(robo_arm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

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

# Add executable
add_executable(end_effector_position src/end_effector_position.cpp)
ament_target_dependencies(end_effector_position 
  rclcpp 
  sensor_msgs)
target_include_directories(end_effector_position PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(end_effector_position Eigen3::Eigen)

# Install executable
install(TARGETS
  end_effector_position
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
