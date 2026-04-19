# 验证用例模板 — MoveIt2 末端移动规划

## 用例表格说明
- `Case ID`: 唯一标识
- `Description`: 场景简述
- `Initial Pose`: 初始末端位姿（position + orientation）
- `Target Pose`: 目标位姿
- `Orientation Constraint`: 是/否
- `Obstacles`: 是否包含障碍（若有，简要描述）
- `Expected`: 通过条件（如 RMS pos < 0.005m）
- `Metrics to Record`: RMS position error, max orientation error (deg), steps to converge, limit_scale events (count), damping events (记录 λ 值变化), notes

## 用例表（示例）

| Case ID | Description | Initial Pose | Target Pose | Orientation Constraint | Obstacles | Expected | Metrics to Record |
|---------|-------------|--------------|-------------|------------------------|-----------|----------|-------------------|
| TC01 | 自由空间短距离平移 | (x0,y0,z0, qx,qy,qz,qw) | (x1,y1,z1, qx,qy,qz,qw) | 否 | 无 | RMS pos < 0.003m | 记录 RMS, max orient err, steps, limit_scale=0, λ history |
| TC02 | 近奇异点姿态跟踪 | ... | ... | 是 | 无 | 无发散，平滑收敛 | 记录是否触发较大 λ 与收敛步数 |
| TC03 | 接近关节边界 | ... | ... | 否 | 无 | 无越界（limit_scale 生效） | 记录 limit_scale 最小值与是否将步长降为 0 |
| TC04 | 带障碍全局规划 | ... | ... | 可选 | 多障碍 | 可达且避障 | 记录规划时间、成功率、轨迹平滑性 |
| TC05 | 平面约束跟随（局部雅可比） | 起始位姿 | 目标平面内位姿 | 可选（通常否） | 无 | 保持平面约束，pos RMS < 5mm | 记录每步 Δq, limit_scale, λ, 平面误差 |
| TC06 | 自动归位（MoveIt 全局规划） | 任意位姿 | 预设归位点 | 否 | 有/无 | 归位成功且无碰撞 | 记录规划时间、成功率、轨迹执行误差 |

## 测试记录模板（每次运行）
- 时间戳
- 控制周期 dt
- 初始关节角 q0
- 目标位姿
- 每步记录：Δq, limit_scale, λ (damping), pos_err, orient_err
- 结束记录：RMS pos, max orient err, total steps, 是否越限、是否失败

---
将本文件保存为 `/home/hku/robo_arm/docs/plans/validation_template.md`，每次验证将测试结果附在此目录下的 `validation_results` 子目录中（可按 Case ID 命名文件）。
