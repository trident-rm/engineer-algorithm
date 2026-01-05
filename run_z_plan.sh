#!/bin/bash

# 移除snap库路径以避免冲突
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v snap | tr '\n' ':')

# Source ROS2环境
source /opt/ros/jazzy/setup.bash
source /home/hku/robo_arm/install/setup.bash

# 启动
ros2 launch z_plan z_plan.launch.py
