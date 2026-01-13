# Install script for directory: /home/hku/robo_arm/src/z_plan

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/hku/robo_arm/install/z_plan")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/z_plan" TYPE EXECUTABLE FILES "/home/hku/robo_arm/build/z_plan/z_plan_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_node"
         OLD_RPATH "/home/hku/ws_moveit/install/moveit_task_constructor_core/lib:/home/hku/ws_moveit/install/moveit_visual_tools/lib:/home/hku/ws_moveit/install/moveit_ros_planning_interface/lib:/home/hku/ws_moveit/install/moveit_ros_move_group/lib:/home/hku/ws_moveit/install/moveit_ros_warehouse/lib:/home/hku/ws_moveit/install/moveit_ros_planning/lib:/home/hku/ws_moveit/install/moveit_core/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/home/hku/ws_moveit/install/moveit_msgs/lib:/home/hku/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/jazzy/lib:/home/hku/ws_moveit/install/rviz_marker_tools/lib:/home/hku/ws_moveit/install/moveit_task_constructor_msgs/lib:/home/hku/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_node")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/hku/robo_arm/build/z_plan/CMakeFiles/z_plan_node.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/z_plan" TYPE EXECUTABLE FILES "/home/hku/robo_arm/build/z_plan/z_plan_movegroup")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup"
         OLD_RPATH "/home/hku/ws_moveit/install/moveit_ros_planning_interface/lib:/home/hku/ws_moveit/install/moveit_visual_tools/lib:/home/hku/ws_moveit/install/moveit_ros_move_group/lib:/home/hku/ws_moveit/install/moveit_ros_warehouse/lib:/home/hku/ws_moveit/install/moveit_ros_planning/lib:/home/hku/ws_moveit/install/moveit_core/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/home/hku/ws_moveit/install/moveit_msgs/lib:/home/hku/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/jazzy/lib:/home/hku/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/hku/robo_arm/build/z_plan/CMakeFiles/z_plan_movegroup.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup_robo_arm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup_robo_arm")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup_robo_arm"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/z_plan" TYPE EXECUTABLE FILES "/home/hku/robo_arm/build/z_plan/z_plan_movegroup_robo_arm")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup_robo_arm" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup_robo_arm")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup_robo_arm"
         OLD_RPATH "/home/hku/ws_moveit/install/moveit_ros_planning_interface/lib:/home/hku/ws_moveit/install/moveit_visual_tools/lib:/home/hku/ws_moveit/install/moveit_ros_move_group/lib:/home/hku/ws_moveit/install/moveit_ros_warehouse/lib:/home/hku/ws_moveit/install/moveit_ros_planning/lib:/home/hku/ws_moveit/install/moveit_core/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/home/hku/ws_moveit/install/moveit_msgs/lib:/home/hku/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/jazzy/lib:/home/hku/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/z_plan_movegroup_robo_arm")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/hku/robo_arm/build/z_plan/CMakeFiles/z_plan_movegroup_robo_arm.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/real_world" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/real_world")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/real_world"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/z_plan" TYPE EXECUTABLE FILES "/home/hku/robo_arm/build/z_plan/real_world")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/real_world" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/real_world")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/real_world"
         OLD_RPATH "/home/hku/ws_moveit/install/moveit_ros_planning_interface/lib:/home/hku/ws_moveit/install/moveit_visual_tools/lib:/home/hku/ws_moveit/install/moveit_ros_move_group/lib:/home/hku/ws_moveit/install/moveit_ros_warehouse/lib:/home/hku/ws_moveit/install/moveit_ros_planning/lib:/home/hku/ws_moveit/install/moveit_core/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/home/hku/ws_moveit/install/moveit_msgs/lib:/home/hku/ws_moveit/install/rviz_visual_tools/lib:/opt/ros/jazzy/lib:/home/hku/ws_moveit/install/moveit_ros_occupancy_map_monitor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/real_world")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/hku/robo_arm/build/z_plan/CMakeFiles/real_world.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/read_data" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/read_data")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/read_data"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/z_plan" TYPE EXECUTABLE FILES "/home/hku/robo_arm/build/z_plan/read_data")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/read_data" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/read_data")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/read_data"
         OLD_RPATH "/opt/ros/jazzy/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/z_plan/read_data")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/hku/robo_arm/build/z_plan/CMakeFiles/read_data.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan" TYPE DIRECTORY FILES
    "/home/hku/robo_arm/src/z_plan/launch"
    "/home/hku/robo_arm/src/z_plan/config"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/hku/robo_arm/build/z_plan/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/z_plan")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/hku/robo_arm/build/z_plan/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/z_plan")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan/environment" TYPE FILE FILES "/home/hku/robo_arm/build/z_plan/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan/environment" TYPE FILE FILES "/home/hku/robo_arm/build/z_plan/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan" TYPE FILE FILES "/home/hku/robo_arm/build/z_plan/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan" TYPE FILE FILES "/home/hku/robo_arm/build/z_plan/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan" TYPE FILE FILES "/home/hku/robo_arm/build/z_plan/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan" TYPE FILE FILES "/home/hku/robo_arm/build/z_plan/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan" TYPE FILE FILES "/home/hku/robo_arm/build/z_plan/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/hku/robo_arm/build/z_plan/ament_cmake_index/share/ament_index/resource_index/packages/z_plan")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan/cmake" TYPE FILE FILES
    "/home/hku/robo_arm/build/z_plan/ament_cmake_core/z_planConfig.cmake"
    "/home/hku/robo_arm/build/z_plan/ament_cmake_core/z_planConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/z_plan" TYPE FILE FILES "/home/hku/robo_arm/src/z_plan/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/hku/robo_arm/build/z_plan/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
