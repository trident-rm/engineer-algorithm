#!/usr/bin/env python3
"""Pyroki + Viser demo for a sequentially optimized Type-III circular task."""

import argparse
import importlib
import os
import sys
import time
from pathlib import Path

import numpy as np


def _patch_numpy_asarray_copy_kwarg():
    try:
        np.asarray([0.0], copy=False)
        return
    except TypeError:
        pass

    original_asarray = np.asarray

    def compat_asarray(a, dtype=None, order=None, *, device=None, copy=None, like=None):
        if device is not None:
            raise NotImplementedError("numpy.asarray(device=...) requires NumPy >= 2.0")
        if like is not None:
            return np.array(a, dtype=dtype, order=order, copy=copy, like=like)
        if copy is True:
            return np.array(a, dtype=dtype, order=order, copy=True)
        return original_asarray(a, dtype=dtype, order=order)

    np.asarray = compat_asarray


def _make_package_filename_handler(search_roots):
    package_paths = {}
    for root in search_roots:
        if not root.is_dir():
            continue
        for path in root.iterdir():
            if path.is_dir():
                package_paths.setdefault(path.name, path)

    def handle_package_uri(fname):
        prefix = "package://"
        if not fname.startswith(prefix):
            return fname
        package, relative_path = fname[len(prefix) :].split("/", 1)
        if package not in package_paths:
            raise FileNotFoundError(f"URDF package '{package}' was not found")
        return str(package_paths[package] / relative_path)

    return handle_package_uri


def _load_robot(urdf_path):
    _patch_numpy_asarray_copy_kwarg()
    import pyroki as pk
    import yourdfpy

    pyroki_root = Path(pk.__file__).resolve().parents[2]
    examples_candidates = [
        Path(os.environ["PYROKI_EXAMPLES_PATH"])
        if os.environ.get("PYROKI_EXAMPLES_PATH")
        else None,
        Path("/home/hku/third_party/pyroki/examples"),
        pyroki_root / "examples",
    ]
    for examples_path in examples_candidates:
        if examples_path is not None and (examples_path / "pyroki_snippets").is_dir():
            if str(examples_path) not in sys.path:
                sys.path.insert(0, str(examples_path))
            break

    try:
        pyroki_snippets = importlib.import_module("pyroki_snippets")
    except ImportError as exc:
        raise RuntimeError(
            "Could not import pyroki_snippets; set PYROKI_EXAMPLES_PATH to the examples directory."
        ) from exc

    workspace_root = Path(__file__).resolve().parents[4]
    workspace_src = workspace_root / "src"
    search_roots = [
        workspace_src / "arms" / "combine_urdf",
        workspace_src / "arms",
        workspace_src,
        workspace_root / "install",
    ]
    urdf = yourdfpy.URDF.load(
        str(urdf_path),
        filename_handler=_make_package_filename_handler(search_roots),
        build_collision_scene_graph=False,
        load_meshes=True,
        load_collision_meshes=False,
    )
    return pk.Robot.from_urdf(urdf), urdf, pyroki_snippets


def _fk_pose(robot, link_index, configuration):
    link_transforms = np.asarray(robot.forward_kinematics(configuration), dtype=np.float64)
    return link_transforms[link_index, 4:], link_transforms[link_index, :4]


def _quaternion_error_rad(actual_wxyz, target_wxyz):
    actual = actual_wxyz / np.linalg.norm(actual_wxyz)
    target = target_wxyz / np.linalg.norm(target_wxyz)
    dot = float(np.clip(abs(np.dot(actual, target)), 0.0, 1.0))
    return 2.0 * np.arccos(dot)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--urdf",
        default=os.environ.get(
            "ROBO_ARM_URDF",
            "/home/hku/robo_arm/src/arms/combine_urdf/urdf/combine_urdf.urdf",
        ),
    )
    parser.add_argument("--ee-link", default=os.environ.get("ROBO_ARM_EE_LINK", "gripper"))
    parser.add_argument("--radius", type=float, default=0.1, help="Circular path radius in meters")
    parser.add_argument("--sweep-deg", type=float, default=90.0, help="Turn P sweep in degrees")
    parser.add_argument("--turn-q-sweep-deg", type=float, default=60.0, help="Turn Q sweep in degrees")
    parser.add_argument("--steps", type=int, default=40, help="Sequential optimization samples")
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument("--position-weight", type=float, default=10000.0)
    parser.add_argument("--axis-weight", type=float, default=5000.0)
    parser.add_argument("--smoothness-weight", type=float, default=10.0)
    parser.add_argument("--joint-limit-weight", type=float, default=5.0)
    parser.add_argument("--manipulability-weight", type=float, default=0.001)
    args = parser.parse_args()

    urdf_path = Path(args.urdf).expanduser().resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF file does not exist: {urdf_path}")
    if (
        args.steps < 1
        or args.radius <= 0.0
        or args.sweep_deg <= 0.0
        or args.turn_q_sweep_deg <= 0.0
    ):
        raise ValueError("steps, radius, and P/Q sweep angles must be positive")

    robot, urdf, pyroki_snippets = _load_robot(urdf_path)
    import jax.numpy as jnp
    import jaxlie

    from trajectory_optimizer import CostWeights, TypeIIITaskPlanner, make_circular_task_path

    joint_names = list(robot.joints.actuated_names)
    link_names = list(robot.links.names)
    if args.ee_link not in link_names:
        raise ValueError(f"End-effector link '{args.ee_link}' not found in the URDF")
    ee_index = link_names.index(args.ee_link)

    start_cfg = np.zeros(len(joint_names), dtype=np.float64)
    for index, joint_name in enumerate(joint_names):
        if joint_name == "joint2":
            start_cfg[index] = 1.9
        elif joint_name == "joint3":
            start_cfg[index] = 0.5
        elif joint_name == "joint5":
            start_cfg[index] = -1.6

    initial_position, initial_wxyz = _fk_pose(robot, ee_index, start_cfg)

    import viser
    from viser.extras import ViserUrdf

    server = viser.ViserServer(port=args.port)
    server.scene.add_grid("/ground", width=2.0, height=2.0)
    viser_urdf = ViserUrdf(server, urdf_or_path=urdf)
    viser_urdf.update_cfg(start_cfg)
    tcp_frame = server.scene.add_frame(
        "/iii_type_task/tcp_frame",
        axes_length=0.09,
        axes_radius=0.003,
        origin_radius=0.006,
        wxyz=initial_wxyz,
        position=initial_position,
    )
    start_pose_control = server.scene.add_transform_controls(
        "/iii_type_task/drag_start_position",
        scale=0.20,
        fixed=False,
        disable_rotations=True,
        depth_test=False,
        wxyz=initial_wxyz,
        position=initial_position,
    )
    status = server.gui.add_text(
        "Status",
        "Drag the TCP position target, then click Plan Turn P + Turn Q.",
        multiline=True,
        disabled=True,
    )
    radius_slider = server.gui.add_slider(
        "Radius (m)", min=0.005, max=0.20, step=0.005,
        initial_value=float(np.clip(args.radius, 0.005, 0.20)),
    )
    p_sweep_slider = server.gui.add_slider(
        "Turn P angle (deg)", min=5, max=180, step=5,
        initial_value=float(np.clip(args.sweep_deg, 5, 180)),
    )
    q_sweep_slider = server.gui.add_slider(
        "Turn Q angle (deg)", min=5, max=180, step=5,
        initial_value=float(np.clip(args.turn_q_sweep_deg, 5, 180)),
    )
    sample_count_slider = server.gui.add_slider(
        "Samples per turn", min=5, max=80, step=5,
        initial_value=float(np.clip(args.steps, 5, 80)),
    )
    position_weight_slider = server.gui.add_slider(
        "Position weight", min=1000, max=100000, step=1000,
        initial_value=float(np.clip(args.position_weight, 1000, 100000)),
    )
    axis_weight_slider = server.gui.add_slider(
        "TCP-X weight", min=500, max=50000, step=500,
        initial_value=float(np.clip(args.axis_weight, 500, 50000)),
    )
    smoothness_weight_slider = server.gui.add_slider(
        "Smoothness weight", min=0, max=100, step=1,
        initial_value=float(np.clip(args.smoothness_weight, 0, 100)),
    )
    joint_limit_weight_slider = server.gui.add_slider(
        "Joint-limit weight", min=0, max=50, step=1,
        initial_value=float(np.clip(args.joint_limit_weight, 0, 50)),
    )
    manipulability_weight_slider = server.gui.add_slider(
        "Manipulability weight", min=0, max=1, step=0.001,
        initial_value=float(np.clip(args.manipulability_weight, 0, 1)),
    )
    plan_button = server.gui.add_button("Plan Turn P + Turn Q")
    step_slider = server.gui.add_slider(
        "Arc sample", min=0, max=160, step=1, initial_value=0, disabled=True
    )
    play_button = server.gui.add_button("Play optimized arc", disabled=True)
    plan_state = {
        "joint_path": None,
        "turn_p_handle": None,
        "turn_q_handle": None,
        "target_p_handle": None,
        "target_q_handle": None,
        "endpoint_handle": None,
    }

    def show_sample(sample_index):
        joint_path = plan_state["joint_path"]
        if joint_path is None:
            return
        configuration = joint_path[int(sample_index)]
        viser_urdf.update_cfg(configuration)
        position, wxyz = _fk_pose(robot, ee_index, configuration)
        tcp_frame.position = position
        tcp_frame.wxyz = wxyz

    @start_pose_control.on_update
    def _(_event):
        status.value = "Target position changed. Click Plan quarter-circle to solve it."

    @step_slider.on_update
    def _(_event):
        show_sample(step_slider.value)

    @plan_button.on_click
    def _(_event):
        plan_button.disabled = True
        play_button.disabled = True
        step_slider.disabled = True
        status.value = "Solving dragged TCP start pose with Pyroki IK..."
        try:
            radius = float(radius_slider.value)
            p_sweep_deg = float(p_sweep_slider.value)
            q_sweep_deg = float(q_sweep_slider.value)
            sample_count = int(sample_count_slider.value)
            active_weights = CostWeights(
                position=float(position_weight_slider.value),
                axis_alignment=float(axis_weight_slider.value),
                smoothness=float(smoothness_weight_slider.value),
                joint_limit=float(joint_limit_weight_slider.value),
                manipulability=float(manipulability_weight_slider.value),
            )
            active_planner = TypeIIITaskPlanner(
                robot, ee_index, weights=active_weights
            )
            requested_position = np.asarray(start_pose_control.position, dtype=np.float64)
            requested_wxyz = np.asarray(start_pose_control.wxyz, dtype=np.float64)
            if not np.all(np.isfinite(requested_position)) or not np.all(
                np.isfinite(requested_wxyz)
            ):
                raise ValueError("Dragged TCP pose contains non-finite values")

            candidate_cfg = np.asarray(
                pyroki_snippets.solve_ik(
                    robot=robot,
                    target_link_name=args.ee_link,
                    target_position=requested_position,
                    target_wxyz=requested_wxyz,
                ),
                dtype=np.float64,
            )
            start_position, start_wxyz = _fk_pose(robot, ee_index, candidate_cfg)
            ik_position_error = float(np.linalg.norm(start_position - requested_position))
            ik_orientation_error = float(
                _quaternion_error_rad(start_wxyz, requested_wxyz)
            )
            if ik_position_error > 0.005 or ik_orientation_error > np.deg2rad(3.0):
                raise RuntimeError(
                    "Dragged pose has no accurate IK solution: "
                    f"position error={ik_position_error:.4f} m, "
                    f"orientation error={np.rad2deg(ik_orientation_error):.2f} deg"
                )

            start_rotation = jaxlie.SO3(jnp.asarray(start_wxyz))
            start_tcp_x = np.asarray(start_rotation.apply(jnp.array([1.0, 0.0, 0.0])))
            start_tcp_y = np.asarray(start_rotation.apply(jnp.array([0.0, 1.0, 0.0])))
            start_tcp_z = np.asarray(start_rotation.apply(jnp.array([0.0, 0.0, 1.0])))
            turn_p_center, turn_p_positions, turn_p_axes = make_circular_task_path(
                start_position=start_position,
                start_axis=start_tcp_x,
                rotation_axis=start_tcp_y,
                start_radial=-radius * start_tcp_x,
                sweep_rad=np.deg2rad(p_sweep_deg),
                samples=sample_count,
                orientation_axis=start_tcp_y,
                orientation_sweep_rad=np.deg2rad(p_sweep_deg),
            )

            turn_p_seed = active_planner.plan(
                candidate_cfg,
                turn_p_positions,
                turn_p_axes,
            )
            turn_p_end_cfg = turn_p_seed.joint_positions[-1]
            turn_p_end_actual_position, turn_p_end_wxyz = _fk_pose(
                robot, ee_index, turn_p_end_cfg
            )
            p_seed_endpoint_error = float(
                np.linalg.norm(turn_p_end_actual_position - turn_p_positions[-1])
            )
            turn_p_end_rotation = jaxlie.SO3(jnp.asarray(turn_p_end_wxyz))
            turn_p_end_z = np.asarray(
                turn_p_end_rotation.apply(jnp.array([0.0, 0.0, 1.0]))
            )
            turn_q_radial = -turn_p_axes[-1]
            turn_q_radial -= np.dot(turn_q_radial, turn_p_end_z) * turn_p_end_z
            turn_q_radial /= np.linalg.norm(turn_q_radial)
            turn_q_center, turn_q_positions, turn_q_axes = make_circular_task_path(
                start_position=turn_p_positions[-1],
                start_axis=turn_p_axes[-1],
                rotation_axis=turn_p_end_z,
                start_radial=radius * turn_q_radial,
                sweep_rad=np.deg2rad(q_sweep_deg),
                samples=sample_count,
                orientation_axis=turn_p_end_z,
                orientation_sweep_rad=np.deg2rad(q_sweep_deg),
            )
            target_positions = np.concatenate(
                [turn_p_positions, turn_q_positions[1:]], axis=0
            )
            target_axes = np.concatenate([turn_p_axes, turn_q_axes[1:]], axis=0)

            status.value = "Optimizing the chained Turn P + Turn Q path..."
            result = active_planner.plan(candidate_cfg, target_positions, target_axes)
            joint_path = result.joint_positions
            tcp_path = np.stack(
                [_fk_pose(robot, ee_index, configuration)[0] for configuration in joint_path]
            )

            if plan_state["turn_p_handle"] is not None:
                plan_state["turn_p_handle"].remove()
                plan_state["turn_q_handle"].remove()
                plan_state["target_p_handle"].remove()
                plan_state["target_q_handle"].remove()
                plan_state["endpoint_handle"].remove()
            p_joint_count = len(turn_p_positions)
            plan_state["turn_p_handle"] = server.scene.add_line_segments(
                "/iii_type_task/turn_p_path",
                points=np.stack(
                    [tcp_path[: p_joint_count - 1], tcp_path[1:p_joint_count]], axis=1
                ),
                colors=(0.92, 0.38, 0.12),
                line_width=3.0,
            )
            p_segment_count = p_joint_count - 1
            plan_state["turn_q_handle"] = server.scene.add_line_segments(
                "/iii_type_task/turn_q_path",
                points=np.stack([tcp_path[p_segment_count:-1], tcp_path[p_segment_count + 1 :]], axis=1),
                colors=(0.15, 0.55, 0.95),
                line_width=3.0,
            )
            plan_state["target_p_handle"] = server.scene.add_line_segments(
                "/iii_type_task/target_turn_p_path",
                points=np.stack([turn_p_positions[:-1], turn_p_positions[1:]], axis=1),
                colors=(1.0, 0.85, 0.2),
                line_width=1.5,
            )
            plan_state["target_q_handle"] = server.scene.add_line_segments(
                "/iii_type_task/target_turn_q_path",
                points=np.stack([turn_q_positions[:-1], turn_q_positions[1:]], axis=1),
                colors=(0.25, 0.95, 0.95),
                line_width=1.5,
            )
            plan_state["endpoint_handle"] = server.scene.add_point_cloud(
                "/iii_type_task/endpoints",
                points=np.stack([tcp_path[0], tcp_path[-1]]),
                colors=np.array([[0.02, 0.02, 0.02], [0.02, 0.02, 0.02]], dtype=np.float32),
                point_size=0.008,
            )
            plan_state["joint_path"] = joint_path
            viser_urdf.update_cfg(candidate_cfg)
            tcp_frame.position = start_position
            tcp_frame.wxyz = start_wxyz
            start_pose_control.position = start_position
            step_slider.value = 0
            step_slider.disabled = False
            play_button.disabled = False
            max_position_error = float(np.max(result.position_errors))
            max_axis_error_deg = float(np.rad2deg(np.max(result.axis_errors_rad)))
            optimizer_status = "converged" if result.optimizer_success else "stopped early"
            status.value = (
                f"Turn P {p_sweep_deg:.0f} deg -> Turn Q {q_sweep_deg:.0f} deg; "
                f"global optimization {optimizer_status}; "
                f"{result.optimizer_iterations} iterations; J={result.objective_value:.3f}.\n"
                f"{len(joint_path)} samples; "
                f"max position error {max_position_error * 1000.0:.1f} mm; "
                f"max TCP-X error {max_axis_error_deg:.2f} deg.\n"
                f"P warm-start endpoint residual {p_seed_endpoint_error * 1000.0:.1f} mm.\n"
                f"P center: {np.array2string(turn_p_center, precision=3)}; "
                f"Q center: {np.array2string(turn_q_center, precision=3)}"
            )
            print(status.value)
        except Exception as exc:
            status.value = f"Planning failed: {exc}"
            print(status.value)
        finally:
            plan_button.disabled = False

    @play_button.on_click
    def _(_event):
        joint_path = plan_state["joint_path"]
        if joint_path is None:
            return
        play_button.disabled = True
        try:
            for sample_index in range(len(joint_path)):
                show_sample(sample_index)
                step_slider.value = sample_index
                time.sleep(0.03)
        finally:
            play_button.disabled = False

    print(f"Viser is running at http://localhost:{args.port}")
    print("Drag the TCP position target, then click Plan Turn P + Turn Q.")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()