* **Build workspace**

  $ colcon build / colcon build --packages-select <package name>
* **Source environment**

  $ source ./install/setup.bash
* **Launch demo in RVIZ**

  $ ros2 launch plan plan_movegroup_robo_arm.launch.py
  Wait for RVIZ interface to fully load，and follow terminal prompts to plan and execute motions
* **Data communication test**

  $ ros2 run plan read_data

* **启动文件们**

  $ ros2 launch plan real_world.launch.py 
  （最开始写的规划末端往上平移10cm的代码，可以在机械臂上执行  
  $ ros2 launch plan real_world_new.launch.py （平面约束版本)  
  $ ros2 launch plan key_step_control.launch.py (通过按键控制规划往指定方向移动1cm,可以与机械臂通讯版本)  
  $ ros2 launch plan turn_movegroup.launch.py (通过按键控制规划三级对矿的绕Q轴转90度,可以与机械臂通讯版本)

* **Robot files**

  The URDF model corresponding to the real-world robot is stored in `robo_arm6/`, along with corresponding SRDF files and configuration files for use with the MoveIt motion planning pipeline.

  最新版机械臂文件是robo_arm10, UC还需要更新机械臂文件


* **目录结构**
  
  src/  
  ├── arms/                          # 机械臂模型 + MoveIt 配置  
  ├── plan/                          # 规划与控制节点  
  ├── tools/                         # 工具库，现在只含有通讯需要的文件  
  └── app/                           # 预留的应用层包  
  
  debug_serial.py #用于调试串口通信的脚本，打印串口里的十六进制数据

* **排查连接** 

  ls /dev/ttyUSB* 或者 ls /dev/ttyACM*
  sudo chmod 666 /dev/ttyUSB0
  或将用户加入dialout组（永久改变）
  ls -l /dev/ttyUSB0 查看所属的group
  sudo usermod -aG dialout $USER   # 把当前用户加入 dialout 组, 之后必须 注销重登 或 重启，组权限才会生效
  groups                            # 验证：输出里应该出现 dialout
  ros2 run plan read_data

* **可视化们** 
  
  1. 透明机械臂是moveit启动后自带的scene robot，橙色的是target。 Scene Robot的消息来自/joint_state, moveit自动订阅这个话题。话题由SerialNode里的成员变量joint_pub_这个publisher创建。
  2.  key_step_control里，绿色的是通过串口收到的机械臂数据画的。topic是/display_robot_state。MoveItVisualTools内部会创建一个moveit_msgs/DisplayRobotState类型的publisher，topic就是这个.


* **TODOs**
  
  1. 自动识别串口设备节点
  2. 用pyroki进行teleop，测试规划
  3. 测平面约束算法
  4. 检查/joint_state和/display_robot_state数据流
  ros2 topic hz /joint_states
  ros2 topic info /joint_states --verbose
  ros2 topic echo /joint_states --once
  5. 确定按键对应的功能
  6. 测所有基本功能
  7. 合并基本功能并部署

* **Notes**
  
  1. Apr20左右把所有的数据的接收改成了timer的回调, 减少了一个线程，把数据接受和movegroup放在同一个线程里（有可能rviz not available的问题是因为只启动了一个executor）
  2. 如果工作去不在/home/hku/robo_arm，需要改plan/CMakeList.txt里的绝对路径
  3. 规划组叫arm，end effector叫gripper
  4. robot arm 10里添加了第七个link，一个虚拟连杆，在抓夹的中心，把这个作为规划的目标(ef)。后续直接在solidwork里创建这个虚拟的link

* **生成SRDF文件之方法**
  
  $ ros2 launch moveit_setup_assistant setup_assistant.launch.py
  


* **control_with_pyroki的启动方式**
python3 ./src/plan/scripts/pyroki_ik_node.py
ros2 launch plan pyroki_ik.launch.py
ros2 launch plan control_with_pyroki.launch.py action:=1
control_with_pyroki是测试单独的轨迹规划，例如只测试turn，或者只测试向上平移1cm这样。

* **May11 Log**
修改了serialNode里的话题发布，serial没有打开时也会发送默认数据到话题
添加了key_board_input节点用于接收键盘输入，模拟真实键盘输入，发布话题到/key_input

* **flow_with_pyroki运行方式.** 
模拟比赛中的上位机控制的全流程
python3 ./src/plan/scripts/pyroki_ik_node.py（发布话题/pyroki_joint_states和/pyroki_controller）
ros2 launch plan pyroki_ik.launch.py(启动Movegroup和rviz节点)
ros2 launch plan flow_with_pyroki.launch.py（订阅/pyroki_joint_states, /key_input, 发布/joint_states）
ros2 run plan keyboard_input（发布/key_input）
/pyroki_controller是存粹为了可视化，话题类型是robotstate
/pyroki_joint_states是为了做计算，但是现在不知道为什么rviz里一个类型为robotstate的sub在订阅他
scene robot模拟的是真实机械臂，订阅/joint_state话题，所以计算出的目标theta会被发布到/joint_state话题上



rviz里的机械臂在不受controller控制时，用的是自己的关节角度
source ./install/setup.bash
