# 为什么换成 PipelinePlanner？

## 🎯 CartesianPath vs PipelinePlanner 对比

| 特性 | CartesianPath | PipelinePlanner (OMPL) |
|------|---------------|------------------------|
| **路径类型** | 严格直线 | 自由路径（可弯曲） |
| **障碍物处理** | 无法绕过 | 可以绕过 |
| **奇异点处理** | 失败 | 可以避开 |
| **规划速度** | 快（简单插值） | 较慢（采样搜索） |
| **成功率** | 低（严格要求） | 高（灵活） |
| **适用场景** | 精确轨迹（如焊接） | 一般运动规划 |

---

## 🔍 CartesianPath 为什么失败？

### **你的输出：**
```
move z+100mm (0/1): CartesianPath: min_fraction not met. Achieved: 0.000000
```

### **这意味着：**
- CartesianPath 要求从起点到终点的**每一个中间点**都必须可达
- `min_fraction = 0.0` 说明**连第一步都无法移动**
- 可能的原因：
  1. ✅ **初始位置在奇异点附近** - 逆运动学无解
  2. ✅ **工作空间边界** - 100mm 太远，超出范围
  3. ✅ **关节限制** - 某个关节已经接近极限
  4. ✅ **碰撞检测** - 路径上有碰撞（虽然你的场景很简单）

---

## ✅ PipelinePlanner 的优势

### **1. 使用 OMPL 算法（RRTConnect）**
```cpp
auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
```

**工作原理：**
- 随机采样关节空间
- 构建树状搜索路径
- 找到从起点到终点的**任意可行路径**（不一定是直线）

### **2. 更高的容错性**
- 可以绕过奇异点
- 可以避开障碍物
- 对初始姿态不敏感

### **3. 默认使用 move_group 配置的规划器**
你的 `ompl_planning.yaml` 中配置了：
```yaml
arm:
  default_planner_config: RRTConnect
  planner_configs:
    - RRTConnect
    - RRT
    - LBKPIECE
```

---

## 🚀 修改后的代码

### **关键变化：**

#### **Before (CartesianPath):**
```cpp
auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
cartesian_planner->setMaxVelocityScalingFactor(1.0);
cartesian_planner->setMaxAccelerationScalingFactor(1.0);
cartesian_planner->setStepSize(.01);  // 1cm 步长，严格插值

auto stage = std::make_unique<mtc::stages::MoveRelative>("move z+100mm", cartesian_planner);
```

**问题：**
- 要求直线路径
- 步长 1cm，100mm 需要 10 步，每步都要可达
- 一旦某步失败，整个规划失败

---

#### **After (PipelinePlanner):**
```cpp
auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
pipeline_planner->setProperty("goal_joint_tolerance", 1e-5);

auto stage = std::make_unique<mtc::stages::MoveRelative>("move z-30mm", pipeline_planner);
```

**优势：**
- 使用 OMPL RRTConnect 算法
- 路径可以弯曲，绕过问题区域
- 更鲁棒，成功率更高

---

## 🎯 期望的 MTC 输出

### **如果成功，应该看到：**
```
[z_plan_node-6]   0  - ←   1 →   -  1 / z_plan_task
[z_plan_node-6]     1  - ←   1 →   -  1 / current
[z_plan_node-6]     1  - ←   1 →   1 →  -  1 / move z-30mm
                    ↑    ↑    ↑    ↑     ↑
                    |    |    |    |     |
                    |    |    |    |     +-- 成功传递给下一阶段
                    |    |    |    +-------- 生成了 1 个解决方案
                    |    |    +------------- 从前一阶段接收了 1 个解
                    |    +------------------ 向后传播了 1 个解
                    +----------------------- 向后接收了 1 个解
```

### **含义：**
- `current` 阶段：成功获取当前状态 ✓
- `move z-30mm` 阶段：成功规划出路径 ✓
- 整个任务成功 ✓

---

## 📋 如果还是失败

### **检查清单：**

#### 1️⃣ **查看完整日志**
```bash
tail -100 /tmp/new_launch_output.log | grep -E "(move z|Failing|error)"
```

#### 2️⃣ **检查初始关节状态**
```bash
ros2 topic echo /joint_states --once
```

**关注：**
- 是否有关节角度接近 `lower` 或 `upper` 限制？
- 是否所有关节都在合理范围内？

#### 3️⃣ **尝试更小的距离**
```cpp
stage->setMinMaxDistance(0.01, 0.01);  // 从 10mm 开始测试
```

#### 4️⃣ **增加规划时间**
```cpp
if (!task_.plan(10))  // 从 5 秒增加到 10 秒
```

#### 5️⃣ **检查 IK 求解器状态**
日志中应该看到：
```
[z_plan_node-6] [INFO] cache file /home/hku/robo_arm/robotic_armarm_worldend_effector_5000_1.000000_1.000000.ikcache initialized!
```

如果 IK 缓存损坏，删除重建：
```bash
rm ~/robo_arm/robotic_armarm_worldend_effector_*.ikcache
```

---

## 🎓 学到的东西

### **什么时候用 CartesianPath？**
- ✅ 绘画、喷涂、焊接（必须走直线）
- ✅ 末端轨迹精确要求
- ✅ 场景简单，无障碍物

### **什么时候用 PipelinePlanner？**
- ✅ 一般运动规划（如你的 Z 轴移动）
- ✅ 复杂场景，有障碍物
- ✅ 不关心路径形状，只关心到达目标
- ✅ CartesianPath 失败时

---

## 🚦 下一步

1. **编译代码：**
   ```bash
   colcon build --packages-select z_plan
   source install/setup.bash
   ```

2. **运行测试：**
   ```bash
   ./run_z_plan.sh
   ```

3. **观察 MTC 图表：**
   - 看是否从 `0 → 0 → 0` 变成 `1 → 1 → 1`

4. **如果成功：**
   - 逐步增加距离：30mm → 50mm → 100mm
   - 测试向上移动：`vec.vector.z = 1.0`

5. **如果失败：**
   - 检查日志中的具体错误
   - 使用 `ros2 topic echo /joint_states` 查看初始姿态
   - 尝试使用 `joint_state_publisher_gui` 手动调整到一个好的初始姿态
