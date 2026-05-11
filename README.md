* **Build workspace**

  $ colcon build
* **Source environment**

  $ source ./install/setup.bash
* **Launch demo in RVIZ**

  $ ros2 launch plan plan_movegroup_robo_arm.launch.py
  Wait for RVIZ interface to fully load，and follow terminal prompts to plan and execute motions
* **Data communication test**

  $ ros2 run plan read_data
* **Real world execution**

  $ ros2 launch plan real_world.launch.py
* **Robot files**

  The URDF model corresponding to the real-world robot is stored in `robo_arm6/`, along with corresponding SRDF files and configuration files for use with the MoveIt motion planning pipeline.


pyroki版启动方式
python3 ./src/plan/scripts/pyroki_ik_node.py
ros2 launch plan pyroki_ik.launch.py
ros2 launch plan control_with_pyroki.launch.py action:=1
