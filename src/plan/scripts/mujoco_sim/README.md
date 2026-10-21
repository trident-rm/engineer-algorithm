# MuJoCo End-to-End Simulation

This directory contains the MuJoCo model, mesh assets, and simulation tools. The Type-III planner remains in `../III_type_task`; the replay script imports it at runtime without placing simulation files in the planner directory.

## Run the replay

From the repository root:

```bash
python3 src/plan/scripts/mujoco_sim/end2end_mujoco_replay.py \
  --seed 20261019 --duration 10 \
  --output /tmp/type3_mujoco_replay.csv
```

The native Viewer opens by default. Press **N** to sample a new core pose and replan, or **R** to replay the current plan. Use `--headless` for batch runs. The CSV records per-step tracking and contact metrics; a sibling `.summary.json` contains aggregate metrics and replay history. The default output files are placed in this directory.

## Model

[`mujoco_assets/robot_scene.xml`](mujoco_assets/robot_scene.xml) is the canonical model loaded by the default replay. It includes robot bodies, inertias, joints, position actuators, TCP site, ground, and core geom. To regenerate the model after changing the URDF or STL assets:

```bash
python3 src/plan/scripts/mujoco_sim/generate_mujoco_model.py
```

The generator requires the packages listed in this directory's [`requirements.txt`](requirements.txt), in addition to the project's planner dependencies.

Visual meshes are simplified from the robot's STL files to at most 180,000 faces each for MuJoCo compatibility. Collision meshes are separate convex hulls generated from the URDF collision STLs, so their shapes do not exactly match the visible STL surface. Robot self-collision is disabled; robot/core and robot/ground contacts are enabled. The core is fixed during each replay and moves to its newly sampled pose after **N**. Contact measurements are approximate, and convex hulls can overestimate collisions.

The simulation uses idealized position actuators; gains and torque limits are not identified hardware parameters. It does not yet model gripper actuation, object grasping, insertion forces, or controller/communication latency, and it does not command physical hardware.
