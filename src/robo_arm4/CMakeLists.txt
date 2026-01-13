cmake_minimum_required(VERSION 3.8)

project(robo_arm4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

foreach(dir config launch meshes urdf)
install(DIRECTORY ${dir}/
DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

ament_package()
