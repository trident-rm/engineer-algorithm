* **Build workspace**

  $ colcon build
* **Source environment**

  $ source ./install/setup.bash
* **Launch demo in RVIZ**

  $ ros2 launch z_plan z_plan_movegroup_robo_arm.launch.py
  Wait for RVIZ interface to fully load，and follow terminal prompts to plan and execute motions
* **Data communication test**

  $ ros2 run z_plan read_data
* **Real world execution**

  $ ros2 launch z_plan real_world.launch.py
* **Robot files**

  The URDF model corresponding to the real-world robot is stored in `robo_arm6/`, along with corresponding SRDF files and configuration files for use with the MoveIt motion planning pipeline.
