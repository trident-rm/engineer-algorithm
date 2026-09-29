# III-Type Task Planning for RoboMaster Engineer Using PyRoki

## Overview

This document describes how to model and solve RoboMaster Engineer Type-III assembly tasks using optimization-based planning in PyRoki.

Typical Type-III tasks include:

- insertion
- constrained sliding

Type-III assembly tasks instead define a constrained motion manifold that the TCP must remain on while preserving task-specific geometric relationships.

The planner should optimize motion on this manifold rather than forcing the robot through a sequence of fixed 6-DOF waypoints.


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

## PyRoki Project Structure

planner/

├── robot_loader.py

├── costs/

│   ├── position_cost.py

│   ├── axis_alignment_cost.py

│   ├── smoothness_cost.py

│   ├── joint_limit_cost.py

│   ├── manipulability_cost.py

│   └── collision_cost.py

├── trajectory/

│   ├── task_generator.py

│   └── trajectory_optimizer.py

└── main.py

---

## Required Cost Implementations

Implement the following cost functions:

position_cost(q)

axis_alignment_cost(q)

smoothness_cost(q, q_prev)

joint_limit_cost(q)

manipulability_cost(q)

collision_cost(q)

total_cost(q)

---

## Sequential Optimization Strategy

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

Recommended as the first implementation.

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

Recommended long-term solution.

---

## Success Criteria

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

---

## Key Principle

Type-III Engineer tasks are NOT pose-tracking problems.

They are constrained-manifold optimization problems.

The planner must exploit remaining task redundancy rather than artificially removing it through exact Cartesian waypoint constraints.

---

## Executable Sequential Optimization Demo

`iii_type_task_demo.py` sends the TCP position along a 90-degree quarter-circle in the local TCP X-Z plane. The starting radius points along `-TCP-X` and the circle rotates around `+TCP-Y`; the initial tangent points up, and the arc midpoint lies above the chord to the endpoint. The center is `C = p_start + r * x_tcp`. Over the same progress, the desired TCP-X axis rotates around `+TCP-Y` toward `-TCP-Z`. Only the TCP-X direction is constrained; rotation around TCP-X remains free.

The reproducible demo start configuration is `joint2=1.9`, `joint3=0.5`, `joint5=-1.6` (other joints zero); it makes TCP-X point nearly along world forward and keeps this sample arc within the configured joint limits.

The optimizer first solves samples sequentially to construct a warm start. It then fixes `q_0` and jointly optimizes the flattened free trajectory `[q_1, ..., q_N]`, minimizing `sum_k J_total(q_k)` plus pairwise joint smoothness across the complete path. The objective contains weighted position, axis-alignment, soft joint-limit, and manipulability costs. Joint bounds are also enforced by the bounded optimizer. Collision costs are intentionally omitted in this test. The position and TCP-X residuals are reported after optimization; they are not used as hard rejection thresholds.

Run it from the workspace root. A 3 cm radius is a known-good interactive test setting:

```bash
python3 src/plan/scripts/III_type_task/iii_type_task_demo.py --radius 0.03
```

In Viser, drag the TCP position control to choose the task start, then click `Plan Turn P + Turn Q`. The target orientation stays fixed while dragging. Pyroki first optimizes Turn P to obtain its terminal TCP frame. Turn Q then starts at that terminal pose: its circle center is displaced by `+radius * TCP-X`, its rotation axis is the terminal TCP-Z, and the positive 60-degree rotation is counter-clockwise when viewed along +Z. Turn P defaults to 90 degrees; Turn Q defaults to 60 degrees and is configurable with `--turn-q-sweep-deg`. Both segments share the configured radius. The P and Q paths are shown in orange and blue, and the combined trajectory is globally optimized after constructing both task paths. After a successful plan, use `Arc sample` or `Play optimized arc`. A failed IK or unreachable arc is reported in `Status`; the radius is never silently reduced. The current default radius is 20 cm; use `--radius 0.03` for the validated interactive example. `--sweep-deg`, `--steps`, cost weights, and viewer port are also configurable. This remains visualization-only and does not command hardware or check collisions.