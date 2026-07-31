# Pyroki Workspace Visualizer

这个脚本用于计算机械臂在三维体素空间中的可达性指数（Reachability Index, RI）。

当前实现要点：

- 体素大小默认 `2cm`（`--step 0.02`）。
 - 体素大小默认 `3cm`（`--step 0.03`）。
- 每个体素在中心点采样，不是边界采样。
- 每个体素内部按固定姿态总数采样：
  - 用 Fibonacci Sphere 在单位球面生成 `N` 个方向（默认 `N=64`）作为工具 Z 轴指向。
  - 每个方向再做 `M` 个离散 Roll（默认 `M=4`，即 `0/90/180/270` 度）。
  - 因此每个体素总姿态数是 `N * M`（默认 `256`）。
- 每个姿态会先合成为 `4x4` 齐次变换矩阵 `T`，再送入 IK 验证流程。
- RI 定义：`RI = N_reach / (N * M)`。
- 可视化：RI 归一化后映射为 `蓝(低) -> 黄(高)` 点云。

脚本文件：`src/plan/scripts/pyroki_workspace_visualizer.py`

## 怎么启动

当前脚本是独立 Python 脚本，不是 `ros2 run` 入口，所以最直接的方式是用 `python3` 启动：

```bash
cd /home/hku/robo_arm
python3 src/plan/scripts/pyroki_workspace_visualizer.py
```

不加任何输出参数时，脚本也会默认保存到：

```bash
/home/hku/robo_arm/output/workspace_scan.npz
```

并同时生成同目录下的：

```bash
/home/hku/robo_arm/output/workspace_scan_voxels.csv
```

常用参数：

```bash
python3 src/plan/scripts/pyroki_workspace_visualizer.py \
  --bounds -0.6 0.6 -0.6 0.6 0.1 0.9 \
  --step 0.03 \
  --direction-samples 64 \
  --roll-samples 4 \
  --output-npz /tmp/workspace_scan.npz
```

参数含义：

- `--bounds XMIN XMAX YMIN YMAX ZMIN ZMAX`：体素区域范围，单位米。
- `--step`：体素边长（米），默认 `0.03`。
- `--direction-samples`：Fibonacci 球面方向数 `N`。
- `--roll-samples`：每个方向的离散 Roll 数 `M`。
- `--ik-pos-tol`：位置误差阈值参数（当前严格零误差模式下不参与可达判定）。
- `--ik-ori-tol-deg`：姿态误差阈值参数（当前严格零误差模式下不参与可达判定）。
- `--fixed-wxyz`：固定末端姿态，格式是 `W X Y Z`。不传时默认用初始姿态的四元数。
- `--seed-count`：每个采样点额外尝试多少组随机 IK 初值。
- `--max-seed-jitter`：随机扰动的标准差。
- `--output-npz`：把结果保存成一个导出 bundle，包含 `.npz` 和体素级 `.csv`；默认路径是 `/home/hku/robo_arm/output/workspace_scan.npz`。
- `--input-npz`：从之前保存的 `.npz` 直接重建 viser slicing 界面，不重新跑 IK 扫描。
- `--port`：viser 服务端口，默认 `8080`。
- `--point-size-scale`：点云显示大小缩放系数，默认 `0.6`，越小点越小。

运行后会自动启动 viser 服务，通常打开 `http://localhost:8080` 就能看到机器人和点云。

## 重开历史结果

如果长时间扫描已经跑完，并且你保留了导出的 `.npz`，以后可以随时直接重开同一个 slicing 可视化，不需要再跑十几个小时：

```bash
cd /home/hku/robo_arm
python3 src/plan/scripts/pyroki_workspace_visualizer.py \
  --input-npz /home/hku/robo_arm/output/workspace_scan.npz
```

这个模式会：

- 直接读取 `voxel_points` 和 `ri_values`。
- 自动恢复保存时的 `bounds` 和 `step`，所以 Z slice 滑条还是按原体素间距走。
- 重新打开和扫描完成后相同的 reachable / unreachable 点云 slicing 界面。
- 同时加载机械臂 URDF，并用全 0 关节角显示机器人本体，便于和点云对照。
- 不依赖 `pyroki`、URDF、IK 再次求解；只要 `viser` 和 `numpy` 还在，就能重看。

也就是说，后续你完全可以把“重计算”和“看结果”分开：

1. 首次长时间扫描时正常加 `--output-npz`。
2. 之后任何一次只想看结果，就改用 `--input-npz`。
3. 即使原来扫描的终端已经关掉，或者那次 viser 会话早就没了，也不影响重新打开。

## 数据流

脚本里的数据流可以理解为下面这条链路：

1. 读取 URDF：从 `ROBO_ARM_URDF`，默认是 `[src/arms/robo_arm10/urdf/robo_arm10.urdf](/home/hku/robo_arm/src/arms/robo_arm10/urdf/robo_arm10.urdf)`。
2. 由 `yourdfpy` 加载机器人模型，再交给 `pyroki` 构建内部机器人对象。
3. 先取一个全 0 的默认关节配置作为初始姿态，用 FK 算出末端初始位姿。
4. 在边界盒中按 `step` 生成体素中心采样点。
5. 为每个体素中心生成 `N * M` 个姿态（Fibonacci 方向 + 离散 Roll）。
6. 每个姿态组合为一个 `4x4` 的 `T` 矩阵。
7. 对每个 `T` 调用 IK，统计该体素成功姿态数 `N_reach`。
8. 计算 `RI = N_reach / (N * M)`。
9. 收集所有体素中心和对应 RI。
10. 将 RI 归一化后做颜色映射（蓝到黄），并用 `viser.scene.add_point_cloud(...)` 绘制。
11. 如果开启 `--output-npz`，最终结果会写入本地文件，便于后处理和在 Google Colab 里重建可视化。

数据对象大致如下：

- 输入：采样边界、体素大小、Fibonacci 方向数、Roll 离散数、IK 初值策略。
- 中间结果：每个体素的 `N_reach`、`RI`、姿态求解统计。
- 输出：RI 点云、每体素姿态命中统计、可选的 npz 文件。

## 工作流

### 1. 初始化

脚本启动后会先：

- 载入 URDF。
- 构建 `pyroki` 机器人模型。
- 设置默认关节角。
- 启动 viser server。
- 把机器人模型和一个参考坐标系显示出来。

### 2. 生成体素中心网格

脚本会根据 `--bounds` 和 `--step` 生成一个三维规则网格，但采样的是体素中心：

- X 方向从 `xmin` 到 `xmax`
- Y 方向从 `ymin` 到 `ymax`
- Z 方向从 `zmin` 到 `zmax`

这一步相当于把机械臂周围包一个盒子，并对每个体素中心逐点测试。

每个轴的中心坐标由下界锚定，即 `min + step / 2 + i * step`。因此修改下界后，只有当下界变化量是 `step` 的整数倍时，原网格中心通常才会与新网格中心精确重合；比较同一点结果时应以 CSV 中完整精度的 `x/y/z` 为准。

### 3. 每体素姿态集求 IK

对每个体素中心，脚本会：

- 以固定的默认关节配置作为初始 seed，使同一坐标的结果不依赖体素遍历顺序或 `bounds` 范围。
- 再加几组随机扰动 seed，避免 IK 因初值不好而失败。
- 生成 `N * M` 个姿态，并组合成 `T` 矩阵。
- 对每个姿态调用一次 `solve_ik`。
- 用 FK 再验证一次末端位置和姿态误差。

可达判定说明（当前版本）：

- 只有当 `pos_err == 0.0` 且 `ori_err == 0.0` 时，该姿态才记入 `N_reach`。
- 这是严格零误差标准，通常会显著降低可达率。

### 4. RI 的含义

脚本对每个体素计算：

- `N_reach`：该体素下成功姿态数。
- `RI = N_reach / (N * M)`：该体素可达性指数。

这能区分：

- 位置几乎不可达（RI 低）。
- 位置在姿态上自由度较高（RI 高）。

### 5. 汇总并可视化

扫描完成后会得到：

- `voxel_points`：所有体素中心坐标。
- `ri_values`：每个体素的 RI。

可视化时先做 RI 归一化，再映射颜色：

- 低 RI：蓝色。
- 高 RI：黄色。

### 6. 可选保存

如果传了 `--output-npz`，脚本会在同目录下写出一组文件：

- `workspace_scan.npz`：数组格式，适合 `numpy.load(...)`。
- `workspace_scan_voxels.csv`：逐体素明细，适合直接在 Colab / pandas 里查看。

其中 `.npz` 里会包含：

- `voxel_points`：每个体素中心坐标，形状 `(K, 3)`。
- `voxel_indices`：每个体素在规则网格中的 `(ix, iy, iz)`。
- `ri_values`：每个体素的 `RI`。
- `pose_success_counts`：每个体素的 `N_reach`。
- `reachable_mask`：是否 `N_reach > 0`。
- `total_pose_per_voxel`：每个体素测试的总姿态数，也就是 `N * M`。
- `x_centers / y_centers / z_centers`：三个轴上的体素中心坐标。
- `grid_shape`：规则网格尺寸。
- `fixed_wxyz / bounds / step / direction_samples / roll_samples`：重建所需元数据。

其中 `.csv` 每一行就是一个体素，字段如下：

- `voxel_id`
- `ix, iy, iz`
- `x, y, z`
- `N_reach`
- `N_total`
- `RI`
- `reachable`

这样你后面不但能用 Python、matplotlib、numpy 继续分析，还能非常直接地检查某个体素的 `N_reach` 到底是多少。

### 7. Google Colab 重建示例

把导出的两个文件上传到 Colab 后，可以直接运行下面这段：

```python
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

npz = np.load('/content/workspace_scan.npz')
voxels = pd.read_csv('/content/workspace_scan_voxels.csv')

print('grid_shape =', npz['grid_shape'].tolist())
print('total_pose_per_voxel =', int(npz['total_pose_per_voxel']))
print(voxels[['ix', 'iy', 'iz', 'x', 'y', 'z', 'N_reach', 'N_total', 'RI']].head())

fig = plt.figure(figsize=(8, 7))
ax = fig.add_subplot(111, projection='3d')

reachable = voxels[voxels['reachable'] == 1]
unreachable = voxels[voxels['reachable'] == 0]

ax.scatter(
  unreachable['x'], unreachable['y'], unreachable['z'],
  c='lightgray', s=8, alpha=0.15, label='unreachable'
)

sc = ax.scatter(
  reachable['x'], reachable['y'], reachable['z'],
  c=reachable['RI'], cmap='viridis', s=12, alpha=0.9, label='reachable'
)

ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.set_title('Workspace Reachability Reconstruction')
fig.colorbar(sc, ax=ax, label='RI')
ax.legend()
plt.show()

# 查看某一个体素的详细数据，例如 N_reach
target = voxels.sort_values('RI', ascending=False).head(10)
display(target[['voxel_id', 'ix', 'iy', 'iz', 'x', 'y', 'z', 'N_reach', 'N_total', 'RI']])
```

如果你想把 `RI` 重新还原成三维规则张量，也可以这样做：

```python
grid_shape = tuple(npz['grid_shape'].tolist())
ri_grid = np.full(grid_shape, np.nan, dtype=np.float32)
n_reach_grid = np.zeros(grid_shape, dtype=np.int32)

indices = npz['voxel_indices']
ri_values = npz['ri_values']
n_reach = npz['pose_success_counts']

ri_grid[indices[:, 0], indices[:, 1], indices[:, 2]] = ri_values
n_reach_grid[indices[:, 0], indices[:, 1], indices[:, 2]] = n_reach

print('RI at [0,0,0] =', ri_grid[0, 0, 0])
print('N_reach at [0,0,0] =', n_reach_grid[0, 0, 0])
```

## 建议的测试方式

建议先用更稀疏参数跑通，例如：

```bash
python3 src/plan/scripts/pyroki_workspace_visualizer.py \
  --step 0.05 \
  --direction-samples 16 \
  --roll-samples 2
```

先确认：

- URDF 能正常加载。
- viser 页面能打开。
- 点云能正常显示。
- 采样范围没有完全偏离机械臂工作区。

然后再逐步增加采样密度到目标配置（例如 `--step 0.02 --direction-samples 64 --roll-samples 4`）。

如果你更关注速度而不是边界细节，建议先保持 `--step 0.03` 或 `--step 0.04`。

## 常见问题

- 如果页面打开了但没有点云，先看终端日志里是否已经扫描完成。
- 如果几乎全部不可达，先确认你是否在使用严格零误差判定（这是预期现象之一）。
- 如果扫描很慢，优先把 `--step` 调大。
- 如果 IK 成功率异常低，可以尝试换一个 `--fixed-wxyz`，或者增大 `--seed-count`。

## 适合看什么结果

这个脚本最适合回答下面几个问题：

- 机械臂在某个工作区域里大概能伸到哪里。
- 哪些位置在多方向/多滚转条件下仍保持较高可达性。
- 机械臂工作空间有没有明显的“空洞”或“薄弱区”。
- 不同采样参数（`N`、`M`、体素大小）对 RI 分布的影响。
