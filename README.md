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

# 终端2: 启动RViz和MoveIt
ros2 launch plan pyroki_ik.launch.py(move these into flow_with_pyroki.launch.py)

# 终端3: 启动flow节点
ros2 launch plan flow_with_pyroki.launch.py

# 终端4: 启动键盘输入
ros2 run plan keyboard_input
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

low_with_pyroki-3] [INFO] [1779451698.911733369] [flow_with_pyroki.moveit.ros.move_group_interface]: Planning request aborted
[flow_with_pyroki-3] [ERROR] [1779451698.912200396] [flow_with_pyroki.moveit.ros.move_group_interface]: MoveGroupInterface::plan() failed or timeout reached
[flow_with_pyroki-3] [WARN] [1779451698.912329098] [flow_with_pyroki]: Plan to energy_unit_1 failed (code=99999, error='FAILURE', message='', source='')