`pose_gen` is a ROS 2 package inside the top-level `robo_arm` workspace, not a standalone workspace.

Build from the workspace root:

```bash
cd ~/robo_arm
colcon build --packages-select pose_gen
source ~/robo_arm/install/setup.bash
```

Run the package nodes with:

```bash
ros2 run pose_gen cam_node
ros2 run pose_gen aptag_node
ros2 run pose_gen corner_node
ros2 run pose_gen tf_node

后续单独跑：ros2 run pose_gen marker_reco_node
```

The Python module namespace now matches the ROS package name:

```bash
python3 -m pose_gen.aptag
```


P-axis就对应规则里面的方向，V-axis对应marker灯竖轴的方向，normal direction就是marker灯的法向量