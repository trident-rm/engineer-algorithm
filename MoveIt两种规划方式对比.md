# MoveIt 的两种规划方式对比

## 🎯 你的代码目前使用的方式

### **MoveIt Task Constructor (MTC)**
```cpp
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>

// 创建任务
mtc::Task task;
task.loadRobotModel(node_);

// 添加阶段
auto stage = std::make_unique<mtc::stages::MoveRelative>("move z+20mm", planner);
stage->setMinMaxDistance(0.02, 0.02);
stage->setDirection(vec);
task.add(std::move(stage));

// 规划
task.plan();

// 执行
task.execute(*task.solutions().front());
```

**特点：**
- ✅ **适合复杂任务**：多阶段、串联规划
- ✅ **更高级**：可以组合多个动作（抓取、移动、放置等）
- ✅ **更灵活**：支持 Generators、Propagators、Connect 等高级阶段
- ❌ **更复杂**：学习曲线陡峭
- ❌ **调试困难**：错误信息不如 MoveGroupInterface 直观

**你当前的任务：** 简单的 Z 轴移动 20mm

---

## 🆚 MoveGroupInterface 方式

### **你提供的示例代码**
```cpp
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "panda_arm");

// 设置目标位置
geometry_msgs::msg::Pose target_pose;
target_pose.orientation.w = 1.0;
target_pose.position.x = 0.28;
target_pose.position.y = -0.2;
target_pose.position.z = 0.5;
move_group_interface.setPoseTarget(target_pose);

// 规划
moveit::planning_interface::MoveGroupInterface::Plan plan;
bool success = move_group_interface.plan(plan);

// 执行
if(success) {
  move_group_interface.execute(plan);
}
```

**特点：**
- ✅ **简单直观**：适合单次移动任务
- ✅ **容易调试**：错误信息清晰
- ✅ **文档丰富**：MoveIt 官方主推的方式
- ❌ **不适合复杂任务**：无法轻松组合多个阶段
- ❌ **功能有限**：没有 MTC 的高级功能

**适合的任务：** 简单的移动到目标位置

---

## 🤔 你应该用哪种？

### **对于你当前的任务（Z 轴移动 20mm）**

#### **推荐：MoveGroupInterface** ⭐

**原因：**
1. **任务简单**：只需要移动到一个目标位置
2. **更容易调试**：CartesianPath 一直失败，用 MoveGroupInterface 更容易找到原因
3. **代码更短**：大约 30 行就能完成，而 MTC 需要 100+ 行

#### **示例代码（改写你的任务）：**

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("z_plan_node");
  
  RCLCPP_INFO(node->get_logger(), "Starting Z Plan Node...");
  
  // 等待 MoveGroup 启动
  rclcpp::sleep_for(std::chrono::seconds(3));
  
  // 创建 MoveGroupInterface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group = MoveGroupInterface(node, "panda_arm");
  
  // 获取当前末端执行器位置
  auto current_pose = move_group.getCurrentPose("panda_hand");
  RCLCPP_INFO(node->get_logger(), "Current position: x=%.3f, y=%.3f, z=%.3f",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);
  
  // 设置目标：Z 轴向上移动 20mm
  geometry_msgs::msg::Pose target_pose = current_pose.pose;
  target_pose.position.z += 0.02;  // +20mm
  
  RCLCPP_INFO(node->get_logger(), "Target position: x=%.3f, y=%.3f, z=%.3f",
              target_pose.position.x,
              target_pose.position.y,
              target_pose.position.z);
  
  move_group.setPoseTarget(target_pose);
  
  // 规划
  RCLCPP_INFO(node->get_logger(), "Planning...");
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group.plan(plan));
  
  if(success) {
    RCLCPP_INFO(node->get_logger(), "✓ Planning succeeded!");
    
    // 执行
    RCLCPP_INFO(node->get_logger(), "Executing...");
    move_group.execute(plan);
    RCLCPP_INFO(node->get_logger(), "✓ Execution completed!");
  } else {
    RCLCPP_ERROR(node->get_logger(), "❌ Planning failed!");
  }
  
  rclcpp::shutdown();
  return 0;
}
```

---

## 📊 两种方式的对比表

| 特性 | MoveGroupInterface | MoveIt Task Constructor |
|------|-------------------|------------------------|
| **学习难度** | ⭐ 简单 | ⭐⭐⭐⭐ 困难 |
| **代码行数** | 30-50 行 | 100-200 行 |
| **适合任务** | 单次移动 | 复杂串联任务 |
| **调试难度** | ⭐ 容易 | ⭐⭐⭐⭐ 困难 |
| **错误信息** | 清晰 | 模糊 |
| **官方文档** | 丰富 | 较少 |
| **性能** | 相同 | 相同 |
| **灵活性** | 有限 | 极高 |
| **Pick & Place** | 需要手动组合 | 内置支持 |

---

## 🛠️ 我的建议

### **方案 1：切换到 MoveGroupInterface（推荐）**

**优点：**
- 更容易找到为什么规划失败
- 代码更简洁
- 调试更方便

**缺点：**
- 需要重写代码（但只需要 30 分钟）

---

### **方案 2：继续使用 MTC**

**如果你坚持用 MTC，需要：**
1. 改用 `MoveTo` 阶段（指定绝对位置）而不是 `MoveRelative`
2. 或者改用 `PipelinePlanner` 代替 `CartesianPath`
3. 添加更多调试输出

---

## 💡 我可以帮你做什么？

### **选项 A：帮你改写成 MoveGroupInterface**
- 完全重写 `z_plan_node.cpp`
- 使用简单的 MoveGroupInterface API
- 大约 30 行代码完成相同任务

### **选项 B：继续调试 MTC**
- 改用 `MoveTo` 到已知的安全位置
- 或者改用 `PipelinePlanner`
- 添加详细的调试输出

### **选项 C：两种都写，你可以对比测试**
- 创建 `z_plan_movegroup.cpp`（MoveGroupInterface 版本）
- 保留 `z_plan_node.cpp`（MTC 版本）
- 分别测试哪个更容易成功

---

## 🎯 你想选哪个？

**告诉我你的选择：**
- **A** - 改用 MoveGroupInterface（简单快速）
- **B** - 继续用 MTC 并调试（学习 MTC）
- **C** - 两个都写，对比测试

我会立即帮你实现！
