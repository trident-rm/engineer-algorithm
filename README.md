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
│   ├── robo_arm10/               # 机械臂描述（含虚拟link）
│   ├── robo_arm10_moveit_config/ # MoveIt配置包
│   └── ...
├── plan/                         # 规划与控制节点
├── tools/                        # 通讯工具库
└── app/                          # 应用层包（预留）
```

### 机械臂配置
- **当前版本**: `robo_arm10` 
  - 包含7个link（第7个是虚拟连杆，位于抓夹中心）
  - 规划组名: `arm`
  - End Effector名: `gripper`

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
- 把调用get_energy_unit的地方都换成了get_energy_unit_target_ik, 添加了一个pyroki的service来做IK
- 解决moveit启动时候的问题，解决视觉识别的平面问题，解决求解器更换问题，导有底盘碰撞的版本
- 添加了一个target_pose可视化函数，发现取矿的旋转矩阵写的不对
- 添加了一个pyroki_ik的单元测试文件，测试指定一个pose能否逆解。
**Jul 8**
- 添加了位置语义信息
**Jul 12**
- 突然出现了连之前能够规划成功的路径现在也无法规划成功的问题。对比了修改的东西，发现是删掉了一行srdf里link2和link3的碰撞检测。
- 添加了一个脚本，绘制可达工作空间


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