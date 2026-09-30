# Type-III Task Planning for RoboMaster Engineer Using Pyroki

## Overview

This document describes the Type-III constrained-manifold planner implemented in this directory. Type-I fixed motions remain controller actions and do not use this optimizer.

## Type-III Planning

Type-III tasks include insertion and constrained sliding, plus the P-axis and Q-axis assembly motions in the end-to-end demo. These tasks define a constrained motion manifold that the TCP must follow. The planner optimizes motion on that manifold rather than forcing the robot through a sequence of fixed 6-DOF poses.

Its design pipeline is:

```text
Task geometry -> target position/axis samples -> sequential warm start
-> joint trajectory optimization -> executable trajectory
```

The implementation accepts target positions and target TCP-X axes and enforces them at every supplied sample. Rotation about TCP-X is left unconstrained by the axis-alignment cost.

## Planner API

`TypeIIIPlanner(robot, link_index, weights=None).plan(start_joint_positions, target_positions, target_axes)` performs a sequential warm start followed by joint-trajectory optimization. Position and TCP-X alignment costs apply at every supplied sample. Approach, Insert, Slide, Turn P, and Turn Q are represented as one interpolated Type-III task path. The Hover endpoint uses 10x position and TCP-X weights; Turn endpoints use the same weights as their other samples.


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

Total optimization objective:

J_total =
    w_pos   * J_pos
  + w_axis  * J_axis
  + w_smooth * J_smooth
  + w_limit * J_limit
  + w_manip * J_manip
  + w_collision * J_collision

Example weights:

w_pos = 100

w_axis = 50

w_smooth = 10

w_limit = 5

w_manip = 1

w_collision = 50

Weights should be tuned experimentally.

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

The current implementation is concentrated in `III_type_task_optimizer.py` and exposes `TypeIIIPlanner`, which performs sequential warm-start solves followed by full joint-trajectory optimization.

The objective terms are position tracking, TCP-X alignment, joint smoothness, soft joint limits, and translational manipulability. Collision checking is not implemented.

## Type-III Optimization Strategy

For every task parameter:

θ0

θ1

θ2

...

θN

Solve:

q0

q1 using q0 as warm start

q2 using q1 as warm start

...

qN

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

`end2end_plan_demo.py` generates a randomized core and plans the complete sequence: Approach to Hover, Insert, Slide, Turn P, then Turn Q. The core local X/q axis is based on world `-X` (a 180-degree yaw flip from the old C++ demo). Roll is sampled within +/-30 degrees, yaw within +/-80 degrees, and pitch from -30 to 0 degrees so the local X/q axis never points below the world horizontal plane and remains within 90 degrees of world `-X`. `p/up` are the other rotation-matrix columns.

The core is sampled in an absolute world-coordinate cuboid, independent of the TCP, with a fixed center at (0.70, 0.00, 0.40) m. Position jitter is +/-0.05 m in X and +/-0.10 m in Y and Z. The core local X/q axis points roughly along world -X. Regenerating the core changes its pose but does not move this world-coordinate frame.

### Type-III API

The task module lives in `src/plan/scripts/III_type_task/`. Use `TypeIIIPlanner` from `III_type_task_optimizer.py`:

```python
planner = TypeIIIPlanner(robot, ee_link_index)
result = planner.plan(start_q, target_positions, target_tcp_x_axes)
```

All supplied task samples participate in Cartesian position and TCP-X alignment costs. The demo interpolates Approach to Hover and optimizes it together with Insert, Slide, Turn P, and Turn Q. The Approach endpoint receives 10x position and TCP-X weights; Turn endpoints use the regular path-point weights.

### Normalized optimizer weights

All cost terms are dimensionless averages with interpretable reference scales: 1 cm TCP position error, 5 degree TCP-X error, 0.1 rad RMS joint step, 0.15 rad joint-limit margin, and 0.05 m^3 translational manipulability. Their GUI/CLI weights are relative multipliers, default `1.0` with a `0–10` range. A weight of `2` doubles that term's normalized contribution; changing waypoint count does not automatically scale it. Manipulability is a negative reward. The Status text and robustness CSV report each final weighted contribution so tuning can be based on actual objective shares.

### Batch robustness test

Run `python3 src/plan/scripts/III_type_task/end2end_robustness_test.py` to evaluate the complete interpolated Type-III target path for deterministic randomized core poses. Results go to `test_result_type_iii.csv`; the previous Type-III baseline remains in `test_result_baseline.csv`.

The saved start configuration is `joint2=1.9`, `joint3=0.5`, `joint5=-1.6` (other joints zero), but the user can drag the TCP start before planning. Hover is `core_center + hover_distance*q` with TCP-X aligned to `-q`; Insert moves along `-q` by the same configured distance to the core center; Slide moves along `+up` by its configured distance.

Turn P follows the C++ p/q/up convention: the initial radius is `radius*q`, and the sign of rotation about `p` is chosen so the initial tangent points toward `up`. Turn Q starts at the P endpoint and uses the P-terminal TCP-Z as its axis; its circle center is one radius along the P-terminal TCP-X, and positive rotation is counter-clockwise when viewed along +Z.

The demo interpolates Approach from the user-selected start to Hover, then optimizes Approach, Insert, Slide, Turn P, and Turn Q as one Type-III trajectory. Only the Hover endpoint receives increased position and TCP-X weights. Optimizer convergence does not guarantee every task is reachable.

Run it from the workspace root. A 3 cm radius is a practical interactive example:

```bash
python3 src/plan/scripts/III_type_task/end2end_plan_demo.py --radius 0.03 --steps 8
```

In Viser, drag the TCP position control to choose the task start, then click `Plan full pipeline`. The target orientation stays fixed while dragging. Approach is linearly interpolated from the current TCP pose to Hover, and the complete Approach/Insert/Slide/Turn P/Turn Q path is optimized in one Type-III call. Only the Hover endpoint receives the increased position and TCP-X weights; Turn P/Q endpoints remain regular path samples. Turn Q starts at the P endpoint: its circle center is displaced by `+radius * TCP-X`, its rotation axis is the terminal TCP-Z, and positive rotation is counter-clockwise when viewed along +Z. Turn P defaults to 90 degrees; Turn Q defaults to 60 degrees and is configurable with `--turn-q-sweep-deg`. Both segments share the configured radius. The P and Q targets are shown in orange and blue. After a successful plan, use `Pipeline sample` or `Play optimized arc`. A failed IK or unreachable path is reported in `Status`; the radius is never silently reduced. The current default radius is 20 cm; use `--radius 0.03` for the validated interactive example. `--sweep-deg`, `--steps`, cost weights, and viewer port are also configurable. This remains visualization-only and does not command hardware or check collisions.