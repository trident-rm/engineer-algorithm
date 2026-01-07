# MTC 任务失败诊断分析

## 📊 你的任务失败图表

```
[z_plan_node-6]   0  - ←   0 →   -  0 / z_plan_task
[z_plan_node-6]     1  - ←   1 →   -  0 / current
[z_plan_node-6]     -  0 →   0 →   -  0 / move z+100mm
[z_plan_node-6] Failing stage(s):
[z_plan_node-6] move z+100mm (0/1): CartesianPath: min_fraction not met. Achieved: 0.000000
```

---

## 🔍 图表解读（按照 MTC 官方文档）

### **Stage 1: `current` (CurrentState 阶段)**
```
1  - ←   1 →   -  0
```

| 位置 | 数值 | 含义 |
|------|------|------|
| **左侧** `1` | ✅ 成功 | 1 个解决方案成功向后传播到前一阶段 |
| **中间** `1` | ✅ 成功 | 生成了 1 个解决方案（当前机器人状态） |
| **右侧** `0` | ❌ **失败** | **没有成功向前传播到下一阶段** |

**含义：**
- CurrentState 正确捕获了机器人当前状态
- **但是这个状态无法传递给下一阶段 `move z+100mm`**
- 问题的根源在下一阶段！

---

### **Stage 2: `move z+100mm` (MoveRelative 阶段)**
```
-  0 →   0 →   -  0
```

| 位置 | 数值 | 含义 |
|------|------|------|
| **左侧** `0` | ❌ **关键失败** | 没有从前一阶段接收到解决方案 |
| **中间** `0` | ❌ 失败 | 没有生成任何解决方案 |
| **右侧** `0` | ❌ 失败 | 没有向后传播解决方案 |

**箭头方向：** `0 →   0 →` （向前传播）

**错误信息：**
```
CartesianPath: min_fraction not met. Achieved: 0.000000
failed to move full distance
```

---

## 🎯 问题诊断

### **根本原因：CartesianPath 规划器完全失败**

从 `Achieved: 0.000000` 可以看出：
- ❌ 规划器连 **1mm** 都无法移动
- ❌ 不是"移动了一半失败"，而是"完全无法开始移动"

### **可能的原因（按优先级排序）：**

#### **1️⃣ 初始位姿在工作空间边界 ⚠️ 最可能**
```cpp
// 你要求向上移动 100mm
vec.vector.z = 1.0;
stage->setMinMaxDistance(0.10, 0.10);
```

**问题：**
- 如果机械臂已经在最高位置，向上 1mm 都不可能
- CartesianPath 是严格的直线路径，碰到任何限制就失败

**验证方法：**
```bash
# 查看当前关节状态
ros2 topic echo /joint_states --once
```

---

#### **2️⃣ 初始姿态在奇异位形附近 🔧**

**奇异位形特征：**
- 某些关节角度接近 0° 或 180°
- 多个关节轴线对齐
- 雅可比矩阵行列式接近 0

**你的 URDF 中的危险配置：**
```xml
<joint name="joint2" type="revolute">
  <limit lower="0" upper="3.141592653589793"/>  ← 注意：下限是 0！
</joint>

<joint name="joint4" type="revolute">
  <limit lower="0" upper="6.283185307179586"/>  ← 下限也是 0
</joint>
```

**如果 joint2 或 joint4 在 0° 附近：**
- 逆运动学可能无解
- CartesianPath 无法计算中间路径点

---

#### **3️⃣ 路径与碰撞检测冲突 💥**

**可能的碰撞：**
- 末端执行器碰到基座
- 连杆之间自碰撞
- 超出工作空间限制

**检查方法：**
```bash
# 查看 planning scene 的碰撞检测结果
ros2 topic echo /monitored_planning_scene --once
```

---

#### **4️⃣ 距离设置过大 📏**

```cpp
stage->setMinMaxDistance(0.10, 0.10);  // 100mm
```

**问题：**
- 从当前位置向上 100mm 可能超出关节限制
- CartesianPath 要求 `min_fraction = 1.0`（完整路径）
- 即使能移动 99mm，也会因为达不到 100mm 而失败

---

## ✅ 解决方案（按优先级）

### **方案 1：改为向下移动小距离（推荐）✨**

```cpp
// Move relative: Z-30mm (向下移动 3厘米)
auto stage = std::make_unique<mtc::stages::MoveRelative>("move z-30mm", cartesian_planner);
stage->setMinMaxDistance(0.03, 0.03);  // 30mm

// Set downward direction (Z-)
geometry_msgs::msg::Vector3Stamped vec;
vec.header.frame_id = "base_link";
vec.vector.z = -1.0;  // 向下
stage->setDirection(vec);
```

**为什么向下更可能成功：**
- ✅ 远离工作空间上限
- ✅ 顺应重力方向
- ✅ 远离奇异位形（假设初始在中间位置）
- ✅ 距离更小（30mm vs 100mm）

---

### **方案 2：降低 min_fraction 要求**

```cpp
auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
cartesian_planner->setMinFraction(0.8);  // 允许 80% 完成即可
```

**适用场景：**
- 测试阶段，先确认能移动一部分
- 接受部分路径完成

---

### **方案 3：切换到 OMPL 规划器（PipelinePlanner）**

```cpp
// 使用 OMPL 替代 CartesianPath
auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
pipeline->setPlannerId("RRTConnectkConfigDefault");

auto stage = std::make_unique<mtc::stages::MoveTo>("move to target", pipeline);
// 设置目标位姿...
```

**优势：**
- 不要求直线路径
- 自动避开奇异位形
- 更容易找到解

---

### **方案 4：检查并调整初始关节状态**

在 `joint_state_publisher_gui` 中：

```
调整关节到"中间位置"：
- joint1: 0°
- joint2: 90° (π/2)  ← 远离下限 0°
- joint3: 0°
- joint4: 180° (π)   ← 远离下限 0°
- joint5: 0°
- joint6: 0°
```

**然后重新规划。**

---

### **方案 5：增加调试信息**

修改 `z_plan_node.cpp`：

```cpp
// 在 createTask() 开始时添加
auto current_state = mtc::stages::CurrentState::make("current");
task.add(std::move(current_state));

// 获取当前末端执行器位姿
RCLCPP_INFO(LOGGER, "Checking current robot state...");
auto robot_model = task.getRobotModel();
auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
// ... 打印关节角度和末端位姿
```

---

## 🧪 推荐的测试顺序

### **测试 1：向下移动 30mm**
```cpp
vec.vector.z = -1.0;
stage->setMinMaxDistance(0.03, 0.03);
```

**预期结果：**
- 如果成功 → 说明问题是"向上超限"或"距离过大"
- 如果失败 → 说明问题是"初始状态有问题"或"奇异位形"

---

### **测试 2：向下移动 10mm**
```cpp
stage->setMinMaxDistance(0.01, 0.01);  // 更小距离
```

**预期结果：**
- 如果成功 → 说明只是距离设置问题
- 如果失败 → 深层问题（奇异位形、碰撞）

---

### **测试 3：使用 OMPL 向上移动**
```cpp
// 切换规划器，保持方向向上
auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
// ...
```

**预期结果：**
- 如果成功 → CartesianPath 的严格直线要求导致失败
- 如果失败 → 真的到达工作空间极限

---

## 📝 当前状态总结

### **确认的信息：**
✅ CurrentState 阶段正常工作（捕获了机器人状态）
✅ MTC 框架正常运行
✅ 参数加载成功（move_group 有加速度限制）

### **失败的阶段：**
❌ MoveRelative with CartesianPath
❌ 完全无法生成任何路径（0.000000）

### **最可能的原因：**
1. **向上移动 100mm 超出工作空间** （70% 可能）
2. **初始位姿在奇异位形附近** （20% 可能）
3. **路径碰撞或自碰撞** （10% 可能）

---

## 🎯 立即行动建议

### **Step 1: 改为向下移动 30mm**
```bash
# 修改代码
vim src/z_plan/src/z_plan_node.cpp
# 改 vec.vector.z = -1.0
# 改 setMinMaxDistance(0.03, 0.03)

# 重新编译
colcon build --packages-select z_plan

# 运行
source install/setup.bash
ros2 launch z_plan z_plan.launch.py
```

### **Step 2: 观察新的 MTC 图表**
```
期待看到：
  1  - ←   1 →   -  1 / current          ← 右侧应该是 1
  -  1 →   1 →   -  1 / move z-30mm      ← 应该全是 1
```

### **Step 3: 如果还是失败**
```bash
# 检查初始关节状态
ros2 topic echo /joint_states --once

# 查看是否有关节在极限位置
# joint2 接近 0° ?
# joint4 接近 0° 或 360° ?
```

---

## 💡 关键启示

**MTC 的图表告诉我们：**
```
1  - ←   1 →   -  0 / current
     ↑         ↑    ↑
     │         │    └─ 问题在这里！无法传递给下一阶段
     │         └────── 成功生成了当前状态
     └──────────────── 向后传播成功（不重要）
```

**这意味着：**
- 不是"找不到机器人"
- 不是"配置错误"
- 而是：**当前状态 + 移动要求 = 物理上不可能**

**类比：**
就像你站在房间顶部，要求"向上跳 1 米" → 不可能，因为会撞到天花板！

**解决方法：**
- 改为"向下跳" → 很容易
- 或者"向上跳 1 厘米" → 可能可以
- 或者"用非直线方式向上移动" → 绕过障碍

---

现在就去试试**向下移动 30mm**吧！🚀
