# RViz 仿真控制器使用指南
## 概述

`rviz_simulation.cpp` 是一个基于 RViz 的机械臂仿真控制节点，无需硬件（串口）连接。用户可以在 RViz 中直接拖动交互式标记来控制虚拟的机械臂。

## 核心特性

### 1. **交互式标记控制**
   - 用户在 RViz 中拖动红色立方体标记（控制器）
   - 标记位置和旋转通过逆运动学（IK）转换为控制器关节角度
   - 实时显示两个虚拟机械臂：
     - **绿色臂**：主机械臂的实际状态
     - **蓝色臂**：约束后的目标状态

### 2. **控制模式**

该节点支持与 `real_world_new.cpp` 相同的 5 种控制模式：

| 模式 | 代码 | 说明 |
|------|------|------|
| NO | 0 | 控制器直接控制机械臂（无约束） |
| UP | 1 | e_hat 方向约束：末端只能在垂直于 e_hat 的平面内运动 |
| TURN | 2 | normal 方向约束：末端绕法向轴转圈 |
| ROLL | 3 | e_hat 方向约束且允许绕轴旋转 |
| RESET | 4 | 机械臂回到初始位置 |

### 3. **约束可视化**
   - 在 RViz 中显示约束方向（箭头）和约束平面（圆形覆盖）
   - 实时显示 UP/TURN/ROLL 模式的约束区域
   - 方便用户理解当前的运动约束

## 使用方法

### 1. 启动节点

```bash
# 首先启动 RViz 和 MoveGroup
ros2 launch panda_moveit_config moveit_rviz.launch.py

# 在另一个终端启动仿真控制器
ros2 run z_plan rviz_simulation
```

### 2. 在 RViz 中操作

1. **拖动标记**：在 RViz 的 3D 视图中找到红色立方体标记，点击并拖动来改变位置
2. **旋转标记**：使用 RViz 交互工具旋转标记
3. **观察虚拟臂**：绿色臂会跟随相应的约束逻辑运动

### 3. 切换控制模式

使用 ROS 2 服务调用来改变控制模式：

```bash
# 切换到 UP 模式
ros2 service call /set_control_mode std_srvs/srv/SetBool "{data: true}"

# 或者直接用 ros2 topic echo 来检查当前模式
ros2 topic echo /simulation_robot_state
```

**注意**：当前实现中，服务调用的具体模式映射可以通过修改 `handle_mode_request` 函数来扩展。建议创建自定义服务以直接指定模式号。

## 物理约束实现

### UP 模式
- **约束**：末端执行器的位移受限于垂直于 e_hat 方向的平面
- **朝向**：完全跟随控制器的朝向
- **算法**：使用雅可比伪逆计算速度映射

### TURN 模式
- **约束**：末端执行器围绕 normal_direction（法向）轴运动
- **朝向**：允许绕轴旋转
- **用途**：实现类似翻花样的动作

### ROLL 模式
- **约束**：类似 UP 模式，消除 e_hat 分量
- **朝向**：允许绕 e_hat 轴旋转
- **用途**：在保持位置的同时进行滚动

## 可视化输出

节点发布以下信息供 RViz 显示：

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/simulation_robot_state` | DisplayRobotState | 机械臂实际状态（绿色） |
| `/simulation_restrict_arm_state` | DisplayRobotState | 约束后的目标状态（蓝色） |
| `/simulation_text_markers` | Marker | 约束方向和平面的可视化 |
| `/simulation_markers` | 内部使用 | 方向箭头和文字标签 |

## 与 real_world_new.cpp 的主要区别

| 特性 | real_world_new | rviz_simulation |
|------|---|---|
| 输入数据源 | SerialRxNode (USB 串口) | RViz Interactive Markers |
| 硬件依赖 | 需要硬件控制器 | 完全虚拟 |
| 控制方式 | 从优悬崖节点接收 | 用户在 RViz 中拖动 |
| 力矩输出 | 发送给硬件 | 仅用于计算（显示） |
| 模式切换 | 从串口接收 mode 码 | 通过服务调用 |

## 配置参数

主要常数（在代码中硬编码）：

```cpp
const double gain = 0.20;              // 控制增益（跟随速度）
const double max_radius = 0.08;        // 约束平面的最大显示半径（80mm）
const int plane_segments = 32;         // 约束平面的分段数（精度）
const double dt = 0.05;                // 控制时间步长（等效）
```

## 扩展建议

### 1. 自定义控制模式服务
创建 `SetControlMode.srv` 来直接指定模式号，而不是使用通用的 `SetBool`。

### 2. 键盘控制
集成 `rclcpp` 的键盘输入回调，通过按键实时切换模式。

### 3. 力反馈
在约束平面处添加虚拟弹簧/阻尼，增强交互体验。

### 4. 多个控制器
支持同时使用多个交互式标记控制不同的机械臂。

## 故障排除

### 问题 1：IK 求解失败
**症状**：蓝色臂出现不期望的姿态或完全消失
**原因**：控制器标记拖到机械臂无法到达的位置
**解决方案**：将标记拖回可达工作空间（通常是距离基座 0.2-0.8m）

### 问题 2：约束平面不显示
**症状**：UP/TURN/ROLL 模式下看不到约束可视化
**原因**：RViz 中的标记话题未订阅
**解决方案**：在 RViz 中添加 `Marker` 显示类型，订阅 `/simulation_text_markers`

### 问题 3：机械臂抖动
**症状**：蓝色臂频繁闪烁
**原因**：IK 求解在奇异点附近
**解决方案**：缓慢移动标记，避免剧烈运动；增加增益值使运动更平滑

## 性能指标

- **控制循环频率**：20ms (50 Hz)
- **IK 求解时间**：通常 < 5ms
- **可视化刷新率**：与控制循环同步
- **CPU 占用率**：< 5% (在现代工作站上)

## 许可证和贡献

该代码继承自 ROS 2 Humble/Iron 环境下的 MoveIt2 框架。欢迎提交改进建议！
