# OMPL 错误：GOAL_STATE_INVALID 分析

## 🔴 错误含义

```
Planner OMPL failed with error code GOAL_STATE_INVALID
```

**翻译：** 目标状态无效

---

## 🎯 可能的原因

### 1️⃣ **目标位置超出关节限制** ⭐ 最可能
```
问题：计算出的目标关节角度超出了 URDF 或 joint_limits.yaml 定义的范围

示例：
- joint2 的 lower=0, upper=π (3.14)
- 规划器计算出需要 joint2=-0.5 才能到达目标
- 结果：GOAL_STATE_INVALID
```

**检查方法：**
```bash
# 查看关节限制
cat src/robo_arm_description/urdf/robo_arm.urdf | grep -A 3 "joint.*type=\"revolute\""
```

---

### 2️⃣ **目标位置发生碰撞**
```
问题：末端执行器移动到目标位置时，机器人的某个连杆会与自身或环境碰撞

示例：
- 向下移动 30mm
- 导致 link3 与 link5 相撞
- 结果：GOAL_STATE_INVALID（目标状态在碰撞中）
```

---

### 3️⃣ **IK 求解失败** ⭐ 很可能
```
问题：从笛卡尔坐标（X,Y,Z）转换到关节角度时，逆运动学求解器找不到解

原因：
- 目标位置超出工作空间（太远、太近、太低）
- 目标位置在奇异点附近
- IK 求解器配置不正确
```

**你的情况：**
```cpp
// MoveRelative 使用相对移动
stage->setMinMaxDistance(0.03, 0.03);  // 30mm

// 但是 IK 可能找不到到达 "当前位置 + Z方向30mm" 的关节配置
```

---

### 4️⃣ **工作空间限制**
```
问题：目标位置在工作空间定义之外

检查 moveit_config 中的 workspace_parameters：
- position_only_ik 模式下的 workspace bounds
- 默认通常是 [-1, -1, -1] 到 [1, 1, 1]
```

---

## 🔍 调试方法

### **方法 1: 检查当前末端执行器位置**

在 z_plan_node.cpp 的 `doTask()` 中添加：

```cpp
void MTCTaskNode::doTask()
{
  RCLCPP_INFO(LOGGER, "Waiting for planning scene to be ready...");
  rclcpp::sleep_for(std::chrono::seconds(3));
  
  // ===== 添加这段代码 =====
  // 获取当前机器人状态
  auto robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node_);
  auto robot_model = robot_model_loader->getModel();
  auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
  robot_state->setToDefaultValues();
  
  // 获取末端执行器当前位置
  const Eigen::Isometry3d& end_effector_pose = 
      robot_state->getGlobalLinkTransform("end_effector");
  
  RCLCPP_INFO(LOGGER, "Current end_effector position:");
  RCLCPP_INFO(LOGGER, "  X: %.3f", end_effector_pose.translation().x());
  RCLCPP_INFO(LOGGER, "  Y: %.3f", end_effector_pose.translation().y());
  RCLCPP_INFO(LOGGER, "  Z: %.3f", end_effector_pose.translation().z());
  
  RCLCPP_INFO(LOGGER, "Target position (Z - 30mm):");
  RCLCPP_INFO(LOGGER, "  X: %.3f", end_effector_pose.translation().x());
  RCLCPP_INFO(LOGGER, "  Y: %.3f", end_effector_pose.translation().y());
  RCLCPP_INFO(LOGGER, "  Z: %.3f", end_effector_pose.translation().z() - 0.03);
  // ===== 添加结束 =====
  
  RCLCPP_INFO(LOGGER, "Creating task...");
  task_ = createTask();
  // ... 其余代码
}
```

---

### **方法 2: 使用更宽松的目标约束**

修改 MoveRelative 阶段：

```cpp
// 不要求精确 30mm，允许 10mm - 30mm 的范围
stage->setMinMaxDistance(0.01, 0.03);  // 至少 10mm，最多 30mm
```

---

### **方法 3: 改用 MoveTo（绝对位置）代替 MoveRelative**

```cpp
// 创建一个明确的、已知可达的目标位置
auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_safe_position", pipeline);
stage->properties().set("group", arm_group_name);

// 设置目标关节值（使用已知的安全配置）
std::map<std::string, double> target_state;
target_state["joint1"] = 0.0;
target_state["joint2"] = 1.57;  // 90度
target_state["joint3"] = 0.0;
target_state["joint4"] = 1.57;
target_state["joint5"] = 0.0;
target_state["joint6"] = 0.0;

stage->setGoal(target_state);
task.add(std::move(stage));
```

---

### **方法 4: 测试 IK 求解器是否工作**

运行 MoveIt 的 IK 测试：

```bash
# 在另一个终端
ros2 run moveit_ros_planning moveit_print_planning_scene_info --ros-args -r __ns:=/move_group

# 或者用 RViz 手动拖动末端执行器，看 IK 是否能求解
```

---

## 🎯 针对你的情况的推荐方案

### **方案 1: 先测试固定的安全位置（最简单）**

```cpp
mtc::Task MTCTaskNode::createTask()
{
  mtc::Task task;
  task.stages()->setName("z_plan_task");
  task.loadRobotModel(node_);

  const auto& arm_group_name = "arm";
  
  // CurrentState
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  task.add(std::move(stage_state_current));

  // 使用 PipelinePlanner
  auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  pipeline->setPlannerId("RRTConnectkConfigDefault");

  // MoveTo: 移动到一个已知安全的关节配置
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_home", pipeline);
    stage->properties().set("group", arm_group_name);
    
    // 定义一个安全的"home"位置（所有关节在中间范围）
    std::map<std::string, double> home_position;
    home_position["joint1"] = 0.0;       // 范围 [-π, π]，中间=0
    home_position["joint2"] = 1.57;      // 范围 [0, π]，中间=π/2
    home_position["joint3"] = 0.0;       // 范围 [-π, π]，中间=0
    home_position["joint4"] = 3.14;      // 范围 [0, 2π]，中间=π
    home_position["joint5"] = 0.0;       // 范围 [-π, π]，中间=0
    home_position["joint6"] = 0.0;       // 范围 [-π, π]，中间=0
    
    stage->setGoal(home_position);
    task.add(std::move(stage));
  }

  return task;
}
```

**优点：**
- 不需要 IK 求解
- 目标状态明确且已知可达
- 容易验证 OMPL 规划器是否能工作

---

### **方案 2: 使用 MoveRelative 但缩小距离**

```cpp
// 只移动 5mm 测试
stage->setMinMaxDistance(0.005, 0.005);  // 5mm
```

**如果 5mm 成功，说明：**
- IK 求解器工作正常
- 只是 30mm 的距离太大，超出了某些限制

---

### **方案 3: 检查并修改 IK 超时时间**

在 `kinematics.yaml` 中增加超时时间：

```yaml
arm:
  kinematics_solver: cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05  # 改为 0.5（更长的求解时间）
  kinematics_solver_attempts: 3    # 改为 10（更多尝试次数）
```

---

## 📊 你的错误日志分析

从你之前的输出：

```
[z_plan_node-6]     1  - ←   1 →   -  0 / current
[z_plan_node-6]     -  0 →   0 →   -  0 / move z-30mm
```

**解读：**
- `current` 阶段成功：生成了 1 个解，向后传播成功
- `move z-30mm` 阶段完全失败：
  - 没有从 current 接收到解（左边是 `-`）
  - 没有生成解（中间是 `0 → 0`）
  - 没有向后传播解（右边是 `-`）

**等等！左边是 `-` 很奇怪！**

这意味着 **MoveRelative 没有从 CurrentState 接收到输入状态**！

可能的原因：
1. **属性不匹配**：MoveRelative 的 `group` 或 `link` 属性设置错误
2. **接口不兼容**：CurrentState 和 MoveRelative 之间的接口有问题

---

## 🔧 立即修复：检查属性设置

```cpp
// 确保这些属性与 task 的全局属性一致
task.setProperty("group", arm_group_name);
task.setProperty("eef", end_effector_name);
task.setProperty("ik_frame", end_effector_name);

// MoveRelative 阶段
auto stage = std::make_unique<mtc::stages::MoveRelative>("move z-30mm", pipeline);
stage->properties().set("group", arm_group_name);  // ✓ 必须与 task 一致
stage->properties().set("link", end_effector_name);  // 参考连杆

// ❌ 移除这个！可能导致冲突
// stage->properties().set("ik_frame", ik_frame);
```

---

## 💡 我的建议

**按照这个顺序尝试：**

1. **立即尝试：** 移除 `ik_frame` 设置，只保留 `group` 和 `link`
2. **如果还失败：** 改用 MoveTo 到已知的关节配置
3. **如果 MoveTo 成功：** 说明 OMPL 工作正常，问题在 MoveRelative 的 IK
4. **如果 MoveTo 也失败：** 检查关节限制和碰撞配置

你想先试哪个？
