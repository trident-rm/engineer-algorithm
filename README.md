### 编译和环境配置
```bash
colcon build
# 或只编译特定包:
colcon build --packages-select <package_name>

# 配置环境
source ./install/setup.bash
```

## 运行场景

### 1. 仿真场景 - MoveIt演示
```bash
ros2 launch plan plan_movegroup_robo_arm.launch.py
# 等待RVIZ加载完毕，按照终端提示规划和执行动作
```

### 2. 键盘控制场景（仿真 + 机械臂通讯）

#### 基础步长控制 (1cm步长)
```bash
ros2 launch plan key_step_control.launch.py
```
- **W/S**: 上/下 (Z轴)
- **A/D**: 左/右 (Y轴)  
- **Q/E**: 后/前 (X轴)
- **I/K**: 末端Z轴正/负方向

#### 旋转控制 (绕Q轴转90°)
```bash
ros2 launch plan turn_movegroup.launch.py
```

### 3. 实物机械臂控制

#### 简单版本 - 末端Z向上平移10cm
```bash
ros2 launch plan real_world.launch.py
```

#### 平面约束版本 - 末端在平面内运动
```bash
ros2 launch plan real_world_new.launch.py
```

### 4. flow_with_pyroki（模拟比赛上位机控制）

运行顺序（三个终端）：
```bash
# 终端1: 启动pyroki IK节点
python3 ./src/plan/scripts/pyroki_ik_node.py

# 终端2: 启动RViz和MoveIt(no need)
ros2 launch plan pyroki_ik.launch.py(move these into flow_with_pyroki.launch.py)

# 终端3: 启动flow节点
ros2 launch plan flow_with_pyroki.launch.py

# 终端4: 启动键盘输入
ros2 run plan keyboard_input

#终端5: 启动视觉节点
ros2 run pose_gen marker_reco_node

```

**控制按键：**
- **G**: Following 模式（跟随pyroki控制器）
- **W/S/A/D/E/Q**: 基座坐标系方向增量运动
- **I/K**: 末端Z轴正/负方向增量运动
- **R**: UpRestrain（约束末端在某方向不动）
- **T**: Turn（绕P轴旋转）
- **1-6**: 去取能量单元
- **X**: 退出

**话题说明：**
- `/pyroki_joint_states`: pyroki的关节角（用于计算）
- `/pyroki_controller`: pyroki的RobotState（用于可视化）
- `/joint_states`: 实际控制的机械臂状态（scene robot订阅此话题）
- `/display_robot_state`: 真实机械臂数据的可视化

### 5. 通讯测试
```bash
ros2 run plan read_data
```
打印串口接收的十六进制数据

## 项目结构

### 代码目录
```
src/  
├── arms/                          # 机械臂模型 + MoveIt配置  
│   ├── robo_arm6/                # 旧版机械臂（6关节）
│   ├── robo_arm10/               # 最新版机械臂（含虚拟link）
│   ├── robo_arm*_moveit_config/  # MoveIt配置包
│   └── ...
├── plan/                         # 规划与控制节点
├── tools/                        # 通讯工具库
└── app/                          # 应用层包（预留）
```

### 机械臂配置
- **最新版本**: `robo_arm10` 
  - 包含7个link（第7个是虚拟连杆，位于抓夹中心）
  - 规划组名: `arm`
  - End Effector名: `gripper`
- **旧版本**: `robo_arm6` 
  - 实物机械臂对应的URDF和SRDF

### 调试脚本
- `debug_serial.py` - 串口通讯调试，打印十六进制数据

## 串口连接排查

```bash
# 列出串口设备
ls /dev/ttyUSB* 或 ls /dev/ttyACM*

# 临时修改权限
sudo chmod 666 /dev/ttyUSB0

# 永久加入dialout组
sudo usermod -aG dialout $USER
# 修改后需要注销重新登录或重启

# 验证
groups  # 输出应该包含dialout

# 测试通讯
ros2 run plan read_data
```

## 可视化说明

### Scene Robot（透明色）
- 订阅 `/joint_states` 话题
- MoveIt自动同步最新关节状态
- 代表实际或规划的机械臂状态

### Target（橙色）
- MoveIt的目标位置显示
- Trajectory播放规划路径

### Display Robot（绿色）
- 通过串口接收的真实机械臂数据
- 话题：`/display_robot_state`
- 由MoveItVisualTools发布

## SRDF文件生成

使用MoveIt Setup Assistant：
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

## 常用ros2命令

```bash
# 检查/joint_states话题发布频率
ros2 topic hz /joint_states

# 查看话题详细信息
ros2 topic info /joint_states --verbose

# 打印话题单条消息
ros2 topic echo /joint_states --once
```

## 开发笔记

### 重要更改历史

**Apr 20**: 所有数据接收改为timer回调，减少了一个线程，将数据接收和move_group放在同一个executor里

**May 11**: 
- SerialNode话题发布改进：即使串口未打开也会发送默认数据
- 添加keyboard_input节点用于接收和模拟键盘输入

**May 16**:
- 删除robo_arm10的base_link inertia（KDL不支持root link带inertia）
- 重构flow_with_pyroki.launch.py

**May 20**:
- created three threads in flow_with_pyroki:main to provent the process from blocking due to the way moveit writes it source code.

**June 19**
- sometimes setup.bash may not be generated under a clean environment, which induces a failure of dynamic linking when starting rviz2. In case of this, rebuild in a new shell:
env -i HOME=$HOME USER=$USER TERM=xterm-256color PATH=/usr/bin:/bin:/usr/sbin:/sbin bash --noprofile --norc
source /opt/ros/jazzy/setup.bash
cd ~/robo_arm
rm -rf build install log
colcon build

or 

source ~/.bashrc
source /opt/ros/jazzy/setup.bash
source ~/robo_arm/install/setup.bash
rviz2


usually, use this order: source /opt/ros/jazzy/setup.bash; source /home/hku/ws_moveit/install/setup.bash; source ~/robo_arm/install/setup.bash (to prevent using ws_moveit2)(have to be used in a newly opened terminal)

**June 20**
- testing mode R and discover a problem that the arm is not moving at all
- todo: draw the marker array of arrows of three directions
- modified  case ControlMode::Turn
- 如果要用L515，需要用老版本驱动，不能用ros jazzy里面带的，所以执行命令的时候要写全路径，例如/usr/local/bin/rs-enumerate-devices

**June 21**
- 添加了Turn模式下的坐标轴和轨迹可视化
- 在pose gen package 下添加了marker_reco节点

**June 24**
- 把mark灯平面和法向量发布并在rviz里可视化（目前是把mark灯的平面和法向量放在base_link之下去显示，camera frame到base_link的frame的转换目前是单位矩阵）
- 在flow_with_pyroki里添加了一个全局变量 hover pose，计算规划最开始的时候，机械臂该去到什么地方

**June 27**
- 解决failed to cal plane问题（因为marker_reco没有把计算完的centerpoint发布）
- 修改plan_turn_p和plan_turn_q,并把半圆轨迹点可视化
- 改相机坐标系的外参（后续需要把camera的link也在urdf里添加，根据机械臂位姿动态计算）
- 把flow_with_pyroki中所有规划相关的函数都改成了返回bool

**June 28**
- 修改了相机坐标系的外参（刚知道相机坐标系是倒着的）
- 删除python虚拟环境（因为之前用的也是系统环境，所以把workspace里的.venv删了）
- 把get_energy_unit里的setJointValueTarget改成了setPoseTarget.
**Jul 4**
- 修改平面参数的滤波在marker_reco_node节点里做，calculate_plane目前只做全局变量的赋值，rviz里打开/plane_visualization看可视化
- 添加了robo_arm_msgs用于存放自定义的消息类型，这样就不用收到数据之后根据时间戳再拼接成同一帧了。


### 路径配置
如果工作目录不在 `/home/hku/robo_arm`，需要更新 `plan/CMakeLists.txt` 中的绝对路径

## TODOs

- [ ] 自动识别串口设备节点
- [ ] 测平面约束算法
- [ ] 合并基本功能并部署到实物
- [ ] 更新所有机械臂文件到robo_arm10版本

##complete version
串口的真实机械臂角度被publish到/joint_states
串口的真实控制器角度被publish到/

高度需要减去底盘的高度
x方向需要加上臂base到轮子最前面的距离
y方向不用调

## How to run package pose_gen

## 查看realsense相关
- 查看版本：which realsense-viewer(which会找可执行文件的精确名字); 或dpkg -l | grep -i 'librealsense\|realsense'
- 打开rsviewer: realsense-viewer
- 查看devices: rs-enumerate-devices









[move_group-2] [INFO] [1783182893.114386255] [move_group]: Calling PlanningRequestAdapter 'CheckStartStateBounds'
[move_group-2] [INFO] [1783182893.114434772] [move_group]: Calling PlanningRequestAdapter 'CheckStartStateCollision'
[move_group-2] [INFO] [1783182893.117147128] [move_group.move_group.moveit.planners.ompl.model_based_planning_context]: Planner configuration 'arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[move_group-2] [INFO] [1783182893.117348087] [move_group]: Calling Planner 'OMPL'
[flow_with_pyroki-3] [INFO] [1783182895.233796239] [flow_with_pyroki]: Current State Mode: 14
[move_group-2] [ERROR] [1783182898.127977745] [move_group.move_group.moveit.planners.ompl.planner_manager]: ./src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:265 - arm/arm: Unable to sample any valid states for goal tree
[move_group-2] [WARN] [1783182898.128897744] [move_group.move_group.moveit.planners.ompl.model_based_planning_context]: Invalid goal state
[move_group-2] [ERROR] [1783182898.396633156] [move_group.move_group.moveit.planners.ompl.model_based_planning_context]: Unable to solve the planning problem
[move_group-2] [ERROR] [1783182898.397236530] [move_group]: Planner 'OMPL' failed with error code GOAL_STATE_INVALID
[move_group-2] [ERROR] [1783182898.397309904] [move_group.move_group.moveit.ros.move_group.move_action]: Generating a plan with planning pipeline failed.
[move_group-2] [INFO] [1783182898.399297517] [move_group.move_group.moveit.ros.move_group.move_action]: FAILURE
[flow_with_pyroki-3] [INFO] [1783182898.401388647] [flow_with_pyroki.moveit.ros.move_group_interface]: Planning request aborted
[flow_with_pyroki-3] [ERROR] [1783182898.401927817] [flow_with_pyroki.moveit.ros.move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[flow_with_pyroki-3] [WARN] [1783182898.402035405] [flow_with_pyroki]: Planning to energy_unit_0 failed (code=99999, error='FAILURE', message='', source='')
[flow_with_pyroki-3] [WARN] [1783182898.402065674] [flow_with_pyroki]: Hover execution failed. Save flow stopped and switched to Idle.
[flow_with_pyroki-3] [INFO] [1783182898.402076204] [flow_with_pyroki]: Save operation completed or failed. Switched to global Idle.
[flow_with_pyroki-3] [INFO] [1783182899.233915690] [flow_with_pyroki]: Current State Mode: 0
[flow_with_pyroki-3] [INFO] [1783182903.233791134] [flow_with_pyroki]: Current State Mode: 0
[flow_with_pyroki-3] [INFO] [1783182905.553760950] [flow_with_pyroki]: Switched to G mode
[flow_with_pyroki-3] [INFO] [1783182905.571674959] [flow_with_pyroki]: Joint 1: current=0.5000, target=0.5000, diff=0.0000
[flow_with_pyroki-3] [INFO] [1783182905.571803967] [flow_with_pyroki]: Joint 2: current=-0.5000, target=-0.5000, diff=0.0000
[flow_with_pyroki-3] [INFO] [1783182905.571818441] [flow_with_pyroki]: Joint 3: current=0.3000, target=0.3000, diff=0.0000
[flow_with_pyroki-3] [INFO] [1783182905.571829825] [flow_with_pyroki]: Joint 4: current=-0.0000, target=0.0000, diff=0.0000
[flow_with_pyroki-3] [INFO] [1783182905.571840327] [flow_with_pyroki]: Joint 5: current=1.0000, target=1.0000, diff=0.0000