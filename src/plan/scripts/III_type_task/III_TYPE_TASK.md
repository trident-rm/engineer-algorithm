# Type-III Task Planning for RoboMaster Engineer Using Pyroki

## Overview

This document describes the Type-III constrained-manifold planner implemented in this directory. Type-I fixed motions remain controller actions and do not use this optimizer.

## Type-III Planning

Type-III tasks include insertion and constrained sliding, plus the P-axis and Q-axis assembly motions in the end-to-end demo. These tasks define a constrained motion manifold that the TCP must follow. The planner optimizes motion on that manifold rather than forcing the robot through a sequence of fixed 6-DOF poses.

Its design pipeline is:

```text
Task geometry -> target position/axis samples -> keyframe warm start
-> joint-space interpolation -> joint trajectory optimization -> executable trajectory
```

The implementation accepts target positions and target TCP-X axes and enforces them at every supplied sample. Rotation about TCP-X is left unconstrained by the axis-alignment cost.

## Planner API

`TypeIIIPlanner(robot, link_index, weights=None).plan(start_joint_positions, target_positions, target_axes)` optimizes a warm start at every fifth sample and at the final sample, interpolates those joint configurations to initialize the full trajectory, then performs joint-trajectory optimization. Position and TCP-X alignment costs apply at every supplied sample in both the global objective and reported errors. Approach, Insert, Slide, Turn P, and Turn Q are represented as one interpolated Type-III task path. The current demo applies a 10x per-waypoint multiplier at the Approach/Hover endpoint for both position and TCP-X; all other waypoints use a multiplier of 1. Global cost weights are shared across the whole path.

The end-to-end demo runs the initial-pose IK and compiles the planner objectives at startup for the configured default weights and sample count. It reuses that planner on the first plan; changing weights or the samples-per-stage setting can require a separate JAX compilation the first time that configuration is used. The robustness test uses the same default path resolution and planner weights as the demo: 8 intervals per stage (41 path samples total), 0.1 m radius, position weight `1.0`, TCP-X weight `1.55`, and zero weights for smoothness, joint limits, and manipulability. The TCP-X weight was tuned against both average- and peak-error pass rates at 3 intervals per stage; it improves the stricter peak-error rate on the tested cases, while intentionally prioritizing axis accuracy over the equal-normalized 1 cm/5 degree balance. The test accepts matching `--*-weight` options and records the selected weights in its summary. It reports separate average- and peak-error pass flags; the legacy `quality_pass` field remains an alias for peak-error pass. It resolves the demo's default start TCP pose with IK and pre-warms the planner for the full five-stage path before timing cases; randomized core poses and post-plan quality/robustness measurements are specific to the test.


## Implemented Objective and Weighting

The planner's task-space terms are normalized before applying their global cost weights:

```text
position_error = ||tcp_position - target_position||_2
position_cost = position_error^2 / (0.01 m)^2

axis_error = acos(clip(dot(tcp_x_axis, target_tcp_x_axis), -1, 1))
axis_cost = (1 - dot(tcp_x_axis, target_tcp_x_axis))
            / (1 - cos(5 degrees))

trajectory_cost = weighted mean(position_cost per waypoint)
                + weighted mean(axis_cost per waypoint)
                + optional smoothness, joint-limit, manipulability terms
```

`CostWeights.position` and `CostWeights.axis_alignment` are **global multipliers** shared by every trajectory segment and waypoint. `position_weights` and `axis_weights` passed to `plan()` are **per-waypoint multipliers**; they are arrays sized to the full combined path, not one scalar per named segment. The demo/test use `1.0` at every point except the Approach/Hover endpoint, where both arrays are set to `10.0`. The resulting contribution is normalized by the sum of the waypoint multipliers, so the hover point gets greater relative emphasis without multiplying the entire segment by ten.

Defaults in the planner API, demo, and robustness test are position `1.0`, TCP-X `1.55`, and smoothness, joint-limit, and manipulability `0.0`. The TCP-X multiplier `1.55` was selected from finite weight scans to improve peak-error pass rate on the tested configuration; it favors axis accuracy relative to equal normalized weighting. It is an empirical setting, not a universal optimum. The demo GUI can change global weights for a plan, but there are not independent GUI sliders per segment.

### Optimizer stages and result interpretation

1. L-BFGS-B optimizes sparse warm-start keyframes, at five-sample intervals and the final waypoint.
2. Per-joint linear interpolation expands those keyframes to a full initial trajectory.
3. A second L-BFGS-B minimizes the objective over every free joint waypoint; the first joint configuration remains fixed.
4. FK is evaluated for every optimized waypoint to report position and TCP-X residuals.

The returned path is a sequence of joint configurations. This code does not time-parameterize the path, guarantee actuator tracking, perform collision checking, or enforce strict Cartesian error inequalities. `optimizer_success` indicates optimizer termination status, not that task error is small.

## Why MoveIt Cartesian Planning Fails

Current approach:

TCP trajectory

↓

Sample cartesian waypoints

↓

computeCartesianPath()

↓

IK for every waypoint

↓

Concatenate joint trajectory

This often fails because Cartesian planning treats every waypoint as a fully constrained 6-DOF pose.

For assembly tasks this is usually incorrect.

Example:

Position must be controlled.

TCP X-axis must be aligned.

Rotation around TCP X-axis is free.

The task therefore contains:

Position = 3 DOF

Tool-axis alignment = 2 DOF

Free spin around tool axis = 1 DOF

Total:

5 constrained DOFs
1 free DOF

However waypoint planning artificially fixes the free rotational DOF and converts the task into a rigid 6-DOF pose-following problem.

This unnecessarily reduces the solution space and frequently causes IK failure, singularities, or joint-space discontinuities.

---

## Planning Philosophy

Do NOT plan:

Pose Trajectory

Plan:

Constraint Manifold

The objective is:

Find robot configuration q that satisfies task constraints while simultaneously optimizing:

- smoothness
- collision avoidance
- joint limit avoidance
- manipulability

The planner should exploit unconstrained task dimensions whenever beneficial.

---

## State Definition

Robot joint state:

q ∈ R^n

Forward kinematics:

T(q)

TCP position:

p(q)

TCP orientation:

R(q)

Jacobian:

J(q)

---

## Constraint 1: Position Tracking

TCP should remain close to the desired assembly trajectory.

Target position:

p_d

Cost:

J_pos = || p(q) - p_d ||²

Purpose:

Maintain geometric assembly motion.

---

## Constraint 2: Tool-Axis Alignment

Engineer assembly tasks are usually defined by the TCP X-axis.

Current TCP X-axis:

x_tcp = R(q) * e_x

Desired task axis:

x_target

Cost:

J_axis = 1 - dot(x_tcp, x_target)

Properties:

0° error -> 0

small angle error -> small penalty

large angle error -> large penalty

Purpose:

Keep the tool aligned with the assembly direction.

---

## Constraint 3: Free Rotation Around Tool Axis

No cost should be applied to rotation around the TCP X-axis.

This degree of freedom must remain available to the optimizer.

Expected benefits:

- improved IK feasibility
- avoidance of singularities
- improved smoothness
- reduced joint motion

---

## Constraint 4: Joint Smoothness

Between consecutive samples:

J_smooth = || q_k - q_(k-1) ||²

Purpose:

- avoid oscillation
- reduce controller effort
- improve execution quality

---

## Constraint 5: Joint Limit Avoidance

Soft penalty near joint limits.

Example:

J_limit = Σ penalty(q_i)

where penalty increases rapidly near:

q_min

q_max

Purpose:

Keep the robot away from problematic configurations.

---

## Constraint 6: Manipulability

Manipulability measure:

m(q) = sqrt(det(J * J^T))

Cost:

J_manip = -m(q)

Purpose:

Avoid singular configurations.

Higher manipulability is preferred.

---

## Constraint 7: Collision Avoidance

Self-collision cost:

J_self

Environment collision cost:

J_env

Combined:

J_collision

This cost should increase as distance to obstacles decreases.

Purpose:

Keep trajectories collision-free.

---

## Global Objective Function

The implemented objective is:

J_total =
    w_pos   * J_pos
  + w_axis  * J_axis
  + w_smooth * J_smooth
  + w_limit * J_limit
  + w_manip * J_manip

There is no collision term in the current implementation. The default weights are `w_pos=1.0`, `w_axis=1.55`, and `w_smooth=w_limit=w_manip=0.0`; demo waypoint multipliers additionally emphasize the Approach/Hover endpoint.

---

## Example: P-Axis Rotation

Task:

Rotate the TCP around a fixed axis by 90 degrees.

Given:

Center:

C

Rotation axis:

n

Radius:

r

Rotation parameter:

θ ∈ [0, π/2]

Desired position:

p_d(θ) = C + R_n(θ)(p0 - C)

Desired axis direction:

x_d(θ) = R_n(θ)x0

Optimization targets:

- remain on circular trajectory
- maintain axis alignment
- exploit free spin DOF
- remain smooth
- avoid singularities
- avoid collisions

The planner should NOT constrain the complete 6-DOF orientation.

Only assembly-relevant constraints should be enforced.

---

## Current Optimizer Scope

The current implementation is concentrated in `III_type_task_planner.py` and exposes `TypeIIIPlanner`, which performs keyframe warm-start solves followed by full joint-trajectory optimization.

The objective terms are position tracking, TCP-X alignment, joint smoothness, soft joint limits, and optional translational manipulability. Manipulability is disabled by default (weight `0`) to avoid Jacobian evaluations; set its weight above `0` to enable it. Collision checking is not implemented.

## Type-III Optimization Strategy

For every task parameter:

θ0

θ1

θ2

...

θN

Optimize every fifth sample as a warm-start keyframe (always including the final sample):

q0

q5 using q0 as warm start

q10 using q5 as warm start

...

qN

Linearly interpolate the keyframe joint configurations to initialize q1 through qN-1. The global trajectory optimizer still evaluates and optimizes every supplied sample.

Advantages:

- simple
- robust
- easy to debug

This sequential solve is used only to initialize the trajectory optimization.

---

## Trajectory Optimization Strategy

Optimization variable:

Q = [q0, q1, q2, ..., qN]

Optimize the entire trajectory simultaneously.

Objective:

Σ J_total(q_k)

Advantages:

- globally consistent
- smoother trajectories
- more robust near singularities

The current Type-III planner optimizes the joint trajectory after constructing this warm start.

### Intended Type-III acceptance criteria

A Type-III planner is considered successful when:

- TCP follows assembly geometry
- Tool axis remains aligned
- Free rotational DOF is exploited
- No self-collision occurs
- No environment collision occurs
- Joint limits are respected
- Singularities are avoided
- Motion is smooth and continuous

The planner should not depend on exact 6-DOF waypoint satisfaction.

The planner should optimize motion on a task manifold.

These are design acceptance criteria, not guarantees of the current code. The current optimizer reports residuals and optimization status; it has no collision checker and does not enforce all these criteria as hard constraints.

---

## Key Principle

Type-III Engineer tasks are NOT pose-tracking problems.

They are constrained-manifold optimization problems.

The planner must exploit remaining task redundancy rather than artificially removing it through exact Cartesian waypoint constraints.

---

## Executable Full Pipeline Demo

`end2end_plan_demo.py` generates a randomized core and plans the complete sequence: Approach to Hover, Insert, Slide, Turn P, then Turn Q. Core orientation is sampled from the RoboMaster Type-III parameters: `theta` uniformly in `[-90°, 90°]`, `phi` in `[0°, 90°]`, and `alpha` in `[-45°, 45°]`. The assembly/local-X direction is `q=[-sin(phi)cos(theta), -sin(phi)sin(theta), cos(phi)]`. The unrolled frame uses `p=[sin(theta), -cos(theta), 0]` and `up=q×p`; consequently at `theta=0`, `phi=90°`, `alpha=0`, local X points along world `-X`, local Y along world `-Y`, and local Z along world `+Z`. The core is then rolled about its current local X by `+alpha` (counter-clockwise when viewed from the positive local-X direction toward the origin), giving `p_axis=cos(alpha)p+sin(alpha)up` and `up_axis=-sin(alpha)p+cos(alpha)up`. The final rotation columns are the core's `q_axis`, `p_axis`, and `up_axis`. The generated task path uses these same axes, so it follows the displayed core orientation.

In the Viser GUI, enter `theta`, `phi`, and `alpha` in degrees and click **Apply core orientation** to set the current core's orientation without changing its center. **Generate random core** continues to randomize both center and orientation, and updates the angle inputs to the generated values.

The demo samples the core center around `[0.45, 0.0, 0.40] m` with half-ranges `[0.05, 0.10, 0.10] m`, yielding `X=[0.40, 0.50] m`, `Y=[-0.10, 0.10] m`, and `Z=[0.30, 0.50] m`. This is the current demo sampling box and is independent of the TCP. The assembly axis has non-negative world-Z component by construction. Regenerating the core changes its pose but does not move this world-coordinate frame.

MuJoCo end-to-end simulation is maintained separately under [`src/plan/scripts/mujoco_sim/`](../mujoco_sim/README.md).

### Type-III API

The task module lives in `src/plan/scripts/III_type_task/`. Use `TypeIIIPlanner` from `III_type_task_planner.py`:

```python
planner = TypeIIIPlanner(robot, ee_link_index)
result = planner.plan(start_q, target_positions, target_tcp_x_axes)
```

All supplied task samples participate in Cartesian position and TCP-X alignment costs. The demo interpolates Approach to Hover and optimizes it together with Insert, Slide, Turn P, and Turn Q. The Approach endpoint receives 10x position and TCP-X weights; Turn endpoints use the regular path-point weights.

### State Machine Integration

The planner is a Python in-process API. A state-machine planning state must already own a Pyroki `robot`, the current measured joint configuration, and the task geometry needed to build the target arrays. It does not perform ROS communication, IK for the start pose, collision checking, or hardware execution.

```python
from pathlib import Path
import sys
import numpy as np

planner_dir = Path("/home/hku/robo_arm/src/plan/scripts/III_type_task")
sys.path.insert(0, str(planner_dir))

from III_type_task_planner import CostWeights, TypeIIIPlanner

ee_link_index = list(robot.links.names).index("gripper")
planner = TypeIIIPlanner(
  robot,
  ee_link_index,
  weights=CostWeights(
    position=1.0,
    axis_alignment=1.55,
    smoothness=0.0,
    joint_limit=0.0,
    manipulability=0.0,
  ),
)

result = planner.plan(
  start_joint_positions=current_joint_positions,
  target_positions=target_positions,
  target_axes=target_tcp_x_axes,
  position_weights=position_weights,
  axis_weights=axis_weights,
)

if not result.optimizer_success:
  raise RuntimeError("Type-III optimizer stopped before convergence")

joint_trajectory = result.joint_positions
```

`current_joint_positions` must have shape `(joint_count,)`, in the order of `robot.joints.actuated_names`. `target_positions` and `target_tcp_x_axes` must both have shape `(sample_count, 3)`, with `sample_count >= 2`; every TCP-X target must be a nonzero unit-direction vector. The first target sample should represent the current task state, and later samples define the full task path in execution order.

`position_weights` and `axis_weights` are optional non-negative arrays of shape `(sample_count,)`. Omit them for unit weights. To make a waypoint stricter without turning it into a hard equality constraint, increase its two entries. The demo assigns `10.0` to the Hover endpoint and `1.0` elsewhere:

```python
position_weights = np.ones(len(target_positions))
axis_weights = np.ones(len(target_positions))
position_weights[hover_index] = 10.0
axis_weights[hover_index] = 10.0
```

The returned `TrajectoryResult` provides `joint_positions`, per-sample `position_errors` in meters, `axis_errors_rad`, `optimizer_success`, `optimizer_iterations`, `objective_value`, and `weighted_cost_components`. A state machine should apply its own acceptance thresholds to the maximum residuals before sending `joint_positions` to a downstream time parameterizer/executor. The planner has no collision checking, so collision validation remains the caller's responsibility.

### Normalized optimizer weights

All cost terms are dimensionless averages with interpretable reference scales: 1 cm TCP position error, 5 degree TCP-X error, 0.1 rad RMS joint step, 0.15 rad joint-limit margin, and 0.05 m^3 translational manipulability. Their GUI/CLI weights are relative multipliers in the `0–10` range. The end-to-end demo, robustness test, and `CostWeights` defaults use position weight `1.0`, TCP-X weight `1.55`, and zero smoothness, joint-limit, and manipulability weights. This setting favors TCP-X alignment; it does not treat 1 cm and 5 degrees as equal normalized priorities. Any term with a zero weight is skipped by the planner, including its associated FK/Jacobian work when no other active term needs it. A weight of `2` doubles that term's normalized contribution; changing waypoint count does not automatically scale it. Manipulability is a negative reward. The Status text and robustness CSV report each final weighted contribution so tuning can be based on actual objective shares.

### Batch robustness test

Run `python3 src/plan/scripts/III_type_task/end2end_robustness_test.py` to evaluate the complete interpolated Type-III target path for deterministic randomized core poses. The generator samples the competition core center ranges and `(theta, phi, alpha)` orientation parameters and records version identifiers and sampled angles in each case CSV. Older case CSVs without the current position and orientation version markers are rejected instead of being silently treated as samples from the new competition distribution; specify a new `--csv` path to generate fresh cases. Defaults match the demo's path resolution and cost weights; the test applies 50 mm/30 degree thresholds separately to trajectory-average errors and per-trajectory peak errors, and computes post-plan metrics. Planner weights are configurable with the same `--position-weight`, `--axis-weight`, `--smoothness-weight`, `--joint-limit-weight`, and `--manipulability-weight` options as the demo. The CSV stores and validates the path and weight configuration; use a new `--csv` path when changing these settings rather than mixing new settings with old case results. The `quality_pass` CSV field remains an alias for peak-error pass for backward compatibility; use `average_quality_pass` and `peak_quality_pass` to distinguish the two criteria. See [TYPE_III_PLANNER_TEST_REPORT.md](./TYPE_III_PLANNER_TEST_REPORT.md) for the parameter scan, full fixed-seed comparison, holdout validation, and limitations.

Both the demo and test use the FK pose of the saved configuration (`joint2=1.9`, `joint3=0.5`, `joint5=-1.6`, other joints zero) as the default TCP start and resolve it with IK; the demo additionally lets the user drag the TCP start. The test fixes this default start for reproducible cases. Hover is `core_center + hover_distance*q` with TCP-X aligned to `-q`; Insert moves along `-q` by the same configured distance to the core center; Slide moves along `+up` by its configured distance.

Turn P follows the C++ p/q/up convention: the initial radius is `radius*q`, and the sign of rotation about `p` is chosen so the initial tangent points toward `up`. Turn Q starts at the P endpoint and uses the P-terminal TCP-Z as its axis; its circle center is one radius along the P-terminal TCP-X, and positive rotation is counter-clockwise when viewed along +Z.

The demo interpolates Approach from the user-selected start to Hover, then optimizes Approach, Insert, Slide, Turn P, and Turn Q as one Type-III trajectory. Only the Hover endpoint receives increased position and TCP-X weights. Optimizer convergence does not guarantee every task is reachable.

Run it from the workspace root. A 3 cm radius is a practical interactive example:

```bash
python3 src/plan/scripts/III_type_task/end2end_plan_demo.py --radius 0.03 --steps 8
```

In Viser, drag the TCP position control to choose the task start, then click `Plan full pipeline`. The target orientation stays fixed while dragging. Approach is linearly interpolated from the current TCP pose to Hover, and the complete Approach/Insert/Slide/Turn P/Turn Q path is optimized in one Type-III call. Only the Hover endpoint receives the increased position and TCP-X weights; Turn P/Q endpoints remain regular path samples. Turn Q starts at the P endpoint: its circle center is displaced by `+radius * TCP-X`, its rotation axis is the terminal TCP-Z, and positive rotation is counter-clockwise when viewed along +Z. Turn P defaults to 90 degrees; Turn Q defaults to 60 degrees and is configurable with `--turn-q-sweep-deg`. Both segments share the configured radius. The P and Q targets are shown in orange and blue. After a successful plan, use `Pipeline sample` or `Play optimized arc`. A failed IK or unreachable path is reported in `Status`; the radius is never silently reduced. The current default radius is 20 cm; use `--radius 0.03` for the validated interactive example. `--sweep-deg`, `--steps`, cost weights, and viewer port are also configurable. This remains visualization-only and does not command hardware or check collisions.