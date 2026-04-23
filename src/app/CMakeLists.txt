cmake_minimum_required(VERSION 3.8)
project(app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plan REQUIRED)
find_package(tools REQUIRED)

# Include directories
include_directories(include)

# Add your executables here:
# add_executable(my_app_node src/my_app_node.cpp)
# ament_target_dependencies(my_app_node rclcpp std_msgs plan tools)
# install(TARGETS my_app_node DESTINATION lib/${PROJECT_NAME})

ament_package()
