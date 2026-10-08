#!/usr/bin/env python3
"""Pyroki + Viser demo for the complete Type-III assembly task path."""

import argparse
import importlib
import os
import sys
import time
from pathlib import Path

import numpy as np

CORE_WORLD_CENTER = np.array([0.7, 0.0, 0.4], dtype=np.float64)
CORE_POSITION_HALF_RANGES = np.array([0.05, 0.1, 0.1], dtype=np.float64)


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


def _axis_rotation_matrix(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    cross = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    return np.eye(3) + np.sin(angle) * cross + (1.0 - np.cos(angle)) * (cross @ cross)


def _sample_random_core(rng):
    center = CORE_WORLD_CENTER + rng.uniform(
        -CORE_POSITION_HALF_RANGES, CORE_POSITION_HALF_RANGES
    )
    roll = rng.uniform(-np.pi / 6.0, np.pi / 6.0)
    pitch = rng.uniform(-np.pi / 6.0, 0.0)
    yaw = rng.uniform(-np.deg2rad(80.0), np.deg2rad(80.0))
    rotation_x = _axis_rotation_matrix([1.0, 0.0, 0.0], roll)
    rotation_y = _axis_rotation_matrix([0.0, 1.0, 0.0], pitch)
    rotation_z = _axis_rotation_matrix([0.0, 0.0, 1.0], yaw + np.pi)
    rotation = rotation_z @ rotation_y @ rotation_x
    return {
        "center": center,
        "q_axis": rotation[:, 0],
        "p_axis": rotation[:, 1],
        "up_axis": rotation[:, 2],
        "rotation": rotation,
    }


def _hover_tcp_x_frame(q_axis):
    hover_x = -np.asarray(q_axis, dtype=np.float64)
    hover_x /= np.linalg.norm(hover_x)
    world_y = np.array([0.0, 1.0, 0.0])
    world_z = np.array([0.0, 0.0, 1.0])
    auxiliary = world_y if abs(np.dot(hover_x, world_y)) < 0.9 else world_z
    hover_z = np.cross(hover_x, auxiliary)
    hover_z /= np.linalg.norm(hover_z)
    hover_y = np.cross(hover_z, hover_x)
    hover_y /= np.linalg.norm(hover_y)
    return hover_x, hover_y, hover_z


def _interpolate_unit_axes(start_axis, end_axis, samples):
    start_axis = np.asarray(start_axis, dtype=np.float64)
    end_axis = np.asarray(end_axis, dtype=np.float64)
    start_axis /= np.linalg.norm(start_axis)
    end_axis /= np.linalg.norm(end_axis)
    dot = float(np.clip(np.dot(start_axis, end_axis), -1.0, 1.0))
    progress_values = np.linspace(0.0, 1.0, samples + 1)
    if dot < -0.9995:
        reference = np.eye(3)[np.argmin(np.abs(start_axis))]
        tangent = np.cross(start_axis, reference)
        tangent /= np.linalg.norm(tangent)
        return np.stack(
            [np.cos(np.pi * t) * start_axis + np.sin(np.pi * t) * tangent for t in progress_values]
        )
    if dot > 0.9995:
        values = np.stack(
            [(1.0 - t) * start_axis + t * end_axis for t in progress_values]
        )
        return values / np.linalg.norm(values, axis=1, keepdims=True)
    angle = np.arccos(dot)
    sine = np.sin(angle)
    return np.stack(
        [
            (np.sin((1.0 - t) * angle) * start_axis + np.sin(t * angle) * end_axis)
            / sine
            for t in progress_values
        ]
    )


def _make_linear_task_segment(start_position, end_position, start_axis, end_axis, samples):
    positions = np.linspace(start_position, end_position, samples + 1)
    axes = _interpolate_unit_axes(start_axis, end_axis, samples)
    return positions, axes


def _combine_task_segments(segments):
    positions = []
    axes = []
    ranges = {}
    for name, segment_positions, segment_axes in segments:
        start_index = max(0, len(positions) - 1)
        if positions:
            positions.extend(segment_positions[1:])
            axes.extend(segment_axes[1:])
        else:
            positions.extend(segment_positions)
            axes.extend(segment_axes)
        ranges[name] = (start_index, len(positions) - 1)
    return np.asarray(positions), np.asarray(axes), ranges


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
    parser.add_argument("--steps", type=int, default=8, help="Samples per pipeline stage")
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument("--position-weight", type=float, default=1.0)
    parser.add_argument("--axis-weight", type=float, default=1.0)
    parser.add_argument("--smoothness-weight", type=float, default=1.0)
    parser.add_argument("--joint-limit-weight", type=float, default=1.0)
    parser.add_argument("--manipulability-weight", type=float, default=0.0)
    args = parser.parse_args()

    urdf_path = Path(args.urdf).expanduser().resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF file does not exist: {urdf_path}")
    if (
        args.steps < 3
        or args.radius <= 0.0
        or args.sweep_deg <= 0.0
        or args.turn_q_sweep_deg <= 0.0
    ):
        raise ValueError("steps must be at least 3; radius and P/Q sweep angles must be positive")
    configured_weights = (
        args.position_weight,
        args.axis_weight,
        args.smoothness_weight,
        args.joint_limit_weight,
        args.manipulability_weight,
    )
    if any(not np.isfinite(weight) or weight < 0.0 or weight > 10.0 for weight in configured_weights):
        raise ValueError("normalized weights must be finite and between 0 and 10")

    robot, urdf, pyroki_snippets = _load_robot(urdf_path)
    import jax.numpy as jnp
    import jaxlie

    from III_type_task_planner import CostWeights, TypeIIIPlanner, make_circular_task_path

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
    startup_weights = CostWeights(
        position=float(np.clip(args.position_weight, 0, 10)),
        axis_alignment=float(np.clip(args.axis_weight, 0, 10)),
        smoothness=float(np.clip(args.smoothness_weight, 0, 10)),
        joint_limit=float(np.clip(args.joint_limit_weight, 0, 10)),
        manipulability=float(np.clip(args.manipulability_weight, 0, 10)),
    )
    startup_planner = TypeIIIPlanner(robot, ee_index, weights=startup_weights)
    startup_started = time.perf_counter()
    startup_ik_solution = np.asarray(
        pyroki_snippets.solve_ik(
            robot=robot,
            target_link_name=args.ee_link,
            target_position=initial_position,
            target_wxyz=initial_wxyz,
        ),
        dtype=np.float64,
    )
    startup_planner.warm_up(
        start_cfg,
        sample_count=5 * int(np.clip(args.steps, 3, 20)) + 1,
    )
    startup_elapsed = time.perf_counter() - startup_started
    print(
        "Completed startup IK/JAX warm-up in "
        f"{startup_elapsed:.2f} s; first plan uses the warmed default settings."
    )
    planner_cache = {startup_weights: startup_planner}

    import viser
    from viser.extras import ViserUrdf

    server = viser.ViserServer(port=args.port)
    server.scene.add_grid("/ground", width=2.0, height=2.0)
    viser_urdf = ViserUrdf(server, urdf_or_path=urdf)
    viser_urdf.update_cfg(start_cfg)
    random_generator = np.random.default_rng()
    core_state = _sample_random_core(random_generator)
    core_wxyz = np.asarray(
        jaxlie.SO3.from_matrix(jnp.asarray(core_state["rotation"])).wxyz
    )
    core_box = server.scene.add_box(
        "/iii_type_task/random_core",
        color=(225, 137, 47),
        dimensions=(0.14, 0.06, 0.06),
        wxyz=core_wxyz,
        position=core_state["center"],
    )
    core_frame = server.scene.add_frame(
        "/iii_type_task/random_core_axes",
        axes_length=0.13,
        axes_radius=0.004,
        wxyz=core_wxyz,
        position=core_state["center"],
    )
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
        "Generate a random core, choose a TCP start, then plan Approach/Insert/Slide/P/Q.",
        multiline=True,
        disabled=True,
    )
    random_core_button = server.gui.add_button("Generate random core")
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
        "Samples per stage", min=3, max=20, step=1,
        initial_value=float(np.clip(args.steps, 3, 20)),
    )
    hover_distance_slider = server.gui.add_slider(
        "Hover / insert distance (m)", min=0.05, max=0.35, step=0.01,
        initial_value=0.20,
    )
    slide_distance_slider = server.gui.add_slider(
        "Slide distance (m)", min=0.01, max=0.20, step=0.01,
        initial_value=0.10,
    )
    position_weight_slider = server.gui.add_slider(
        "Position weight (normalized)", min=0, max=10, step=0.1,
        initial_value=float(np.clip(args.position_weight, 0, 10)),
    )
    axis_weight_slider = server.gui.add_slider(
        "TCP-X weight (normalized)", min=0, max=10, step=0.1,
        initial_value=float(np.clip(args.axis_weight, 0, 10)),
    )
    smoothness_weight_slider = server.gui.add_slider(
        "Smoothness weight (normalized)", min=0, max=10, step=0.1,
        initial_value=float(np.clip(args.smoothness_weight, 0, 10)),
    )
    joint_limit_weight_slider = server.gui.add_slider(
        "Joint-limit weight (normalized)", min=0, max=10, step=0.1,
        initial_value=float(np.clip(args.joint_limit_weight, 0, 10)),
    )
    manipulability_weight_slider = server.gui.add_slider(
        "Manipulability weight (normalized)", min=0, max=10, step=0.1,
        initial_value=float(np.clip(args.manipulability_weight, 0, 10)),
    )
    plan_button = server.gui.add_button("Plan full pipeline")
    step_slider = server.gui.add_slider(
        "Pipeline sample", min=0, max=100, step=1, initial_value=0, disabled=True
    )
    play_button = server.gui.add_button("Play optimized arc", disabled=True)
    plan_state = {
        "joint_path": None,
        "stage_ranges": {},
        "actual_path_handles": [],
        "target_path_handles": [],
        "endpoint_handle": None,
    }

    def clear_plan():
        for handle in plan_state["actual_path_handles"] + plan_state["target_path_handles"]:
            handle.remove()
        if plan_state["endpoint_handle"] is not None:
            plan_state["endpoint_handle"].remove()
        plan_state["joint_path"] = None
        plan_state["stage_ranges"] = {}
        plan_state["actual_path_handles"] = []
        plan_state["target_path_handles"] = []
        plan_state["endpoint_handle"] = None
        step_slider.disabled = True
        play_button.disabled = True

    def show_sample(sample_index):
        joint_path = plan_state["joint_path"]
        if joint_path is None:
            return
        index = int(np.clip(int(sample_index), 0, len(joint_path) - 1))
        configuration = joint_path[index]
        viser_urdf.update_cfg(configuration)
        position, wxyz = _fk_pose(robot, ee_index, configuration)
        tcp_frame.position = position
        tcp_frame.wxyz = wxyz

    @start_pose_control.on_update
    def _(_event):
        clear_plan()
        status.value = "TCP start changed. Click Plan full pipeline when ready."

    @random_core_button.on_click
    def _(_event):
        nonlocal core_state
        core_state = _sample_random_core(random_generator)
        core_wxyz = np.asarray(
            jaxlie.SO3.from_matrix(jnp.asarray(core_state["rotation"])).wxyz
        )
        core_box.position = core_state["center"]
        core_box.wxyz = core_wxyz
        core_frame.position = core_state["center"]
        core_frame.wxyz = core_wxyz
        clear_plan()
        status.value = (
            "Random core regenerated at "
            f"{np.array2string(core_state['center'], precision=3)}"
        )

    @step_slider.on_update
    def _(_event):
        show_sample(step_slider.value)

    @plan_button.on_click
    def _(_event):
        planning_started = time.perf_counter()
        plan_button.disabled = True
        clear_plan()
        status.value = "Solving start IK and generating Hover/Insert/Slide/P/Q targets..."
        try:
            radius = float(radius_slider.value)
            p_sweep_deg = float(p_sweep_slider.value)
            q_sweep_deg = float(q_sweep_slider.value)
            sample_count = int(sample_count_slider.value)
            hover_distance = float(hover_distance_slider.value)
            insert_distance = float(hover_distance_slider.value)
            slide_distance = float(slide_distance_slider.value)
            active_weights = CostWeights(
                position=float(position_weight_slider.value),
                axis_alignment=float(axis_weight_slider.value),
                smoothness=float(smoothness_weight_slider.value),
                joint_limit=float(joint_limit_weight_slider.value),
                manipulability=float(manipulability_weight_slider.value),
            )
            active_planner = planner_cache.get(active_weights)
            if active_planner is None:
                active_planner = TypeIIIPlanner(robot, ee_index, weights=active_weights)
                planner_cache[active_weights] = active_planner
            requested_position = np.asarray(start_pose_control.position, dtype=np.float64)
            requested_wxyz = np.asarray(start_pose_control.wxyz, dtype=np.float64)
            if not np.all(np.isfinite(requested_position)) or not np.all(
                np.isfinite(requested_wxyz)
            ):
                raise ValueError("Dragged TCP pose contains non-finite values")

            if np.allclose(requested_position, initial_position, atol=1e-8, rtol=0.0) and np.allclose(
                requested_wxyz, initial_wxyz, atol=1e-8, rtol=0.0
            ):
                candidate_cfg = startup_ik_solution.copy()
            else:
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

            core_center = core_state["center"]
            q_axis = core_state["q_axis"]
            p_axis = core_state["p_axis"]
            up_axis = core_state["up_axis"]
            start_rotation = jaxlie.SO3(jnp.asarray(start_wxyz))
            start_tcp_x = np.asarray(start_rotation.apply(jnp.array([1.0, 0.0, 0.0])))
            hover_tcp_x, hover_tcp_y, hover_tcp_z = _hover_tcp_x_frame(q_axis)
            hover_position = core_center + hover_distance * q_axis
            insert_position = hover_position - insert_distance * q_axis
            slide_position = insert_position + slide_distance * up_axis
            approach_positions, approach_axes = _make_linear_task_segment(
                start_position,
                hover_position,
                start_tcp_x,
                hover_tcp_x,
                sample_count,
            )
            insert_positions, insert_axes = _make_linear_task_segment(
                hover_position,
                insert_position,
                hover_tcp_x,
                hover_tcp_x,
                sample_count,
            )
            slide_positions, slide_axes = _make_linear_task_segment(
                insert_position,
                slide_position,
                hover_tcp_x,
                hover_tcp_x,
                sample_count,
            )
            turn_p_start = slide_position
            radius_vector_p = radius * q_axis
            p_direction = (
                1.0
                if np.dot(np.cross(p_axis, radius_vector_p), up_axis) >= 0.0
                else -1.0
            )
            turn_p_axis = p_direction * p_axis
            turn_p_center, turn_p_positions, turn_p_axes = make_circular_task_path(
                start_position=turn_p_start,
                start_axis=-q_axis,
                rotation_axis=turn_p_axis,
                start_radial=radius_vector_p,
                sweep_rad=np.deg2rad(p_sweep_deg),
                samples=sample_count,
                orientation_axis=turn_p_axis,
                orientation_sweep_rad=np.deg2rad(p_sweep_deg),
            )
            turn_p_end_x = turn_p_axes[-1]
            turn_p_end_y = -p_axis
            turn_p_end_z = np.cross(turn_p_end_x, turn_p_end_y)
            turn_p_end_z /= np.linalg.norm(turn_p_end_z)
            turn_p_end_y = np.cross(turn_p_end_z, turn_p_end_x)
            turn_p_end_y /= np.linalg.norm(turn_p_end_y)
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
            target_segments = [
                ("Approach", approach_positions, approach_axes),
                ("Insert", insert_positions, insert_axes),
                ("Slide", slide_positions, slide_axes),
                ("Turn P", turn_p_positions, turn_p_axes),
                ("Turn Q", turn_q_positions, turn_q_axes),
            ]
            target_positions, target_axes, stage_ranges = _combine_task_segments(
                target_segments
            )
            position_weights = np.ones(len(target_positions), dtype=np.float64)
            axis_weights = np.ones(len(target_axes), dtype=np.float64)
            approach_start, approach_end = stage_ranges["Approach"]
            position_weights[approach_end] = 10.0
            axis_weights[approach_end] = 10.0
            status.value = "Optimizing the interpolated Type-III task path..."
            result = active_planner.plan(
                candidate_cfg,
                target_positions,
                target_axes,
                position_weights=position_weights,
                axis_weights=axis_weights,
            )
            joint_path = result.joint_positions
            position_errors = result.position_errors
            axis_errors_rad = result.axis_errors_rad
            tcp_path = np.stack(
                [_fk_pose(robot, ee_index, configuration)[0] for configuration in joint_path]
            )
            planning_elapsed = time.perf_counter() - planning_started

            clear_plan()
            stage_colors = {
                "Approach": ((0.75, 0.75, 0.75), (0.95, 0.95, 0.95)),
                "Insert": ((0.8, 0.3, 0.85), (0.95, 0.65, 1.0)),
                "Slide": ((0.15, 0.7, 0.45), (0.5, 1.0, 0.7)),
                "Turn P": ((0.92, 0.38, 0.12), (1.0, 0.85, 0.2)),
                "Turn Q": ((0.15, 0.55, 0.95), (0.25, 0.95, 0.95)),
            }
            plan_state["stage_ranges"] = stage_ranges
            for stage_name, segment_positions, _ in target_segments:
                range_start, range_end = stage_ranges[stage_name]
                actual_points = tcp_path[range_start : range_end + 1]
                target_points = segment_positions
                actual_color, target_color = stage_colors[stage_name]
                stage_key = stage_name.lower().replace(" ", "_")
                plan_state["actual_path_handles"].append(
                    server.scene.add_line_segments(
                        f"/iii_type_task/actual_{stage_key}",
                        points=np.stack([actual_points[:-1], actual_points[1:]], axis=1),
                        colors=actual_color,
                        line_width=3.0,
                    )
                )
                if stage_name != "Approach":
                    plan_state["target_path_handles"].append(
                        server.scene.add_line_segments(
                            f"/iii_type_task/target_{stage_key}",
                            points=np.stack([target_points[:-1], target_points[1:]], axis=1),
                            colors=target_color,
                            line_width=1.5,
                        )
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
            step_slider.max = len(joint_path) - 1
            step_slider.disabled = False
            play_button.disabled = False
            max_position_error = float(np.nanmax(position_errors))
            max_axis_error_deg = float(np.rad2deg(np.nanmax(axis_errors_rad)))
            optimizer_status = "converged" if result.optimizer_success else "stopped early"
            status.value = (
                f"Random core: H {hover_distance:.2f} m -> Insert {insert_distance:.2f} m "
                f"-> Slide {slide_distance:.2f} m -> P {p_sweep_deg:.0f} deg "
                f"-> Q {q_sweep_deg:.0f} deg;\n"
                f"Type-III optimization {optimizer_status}; "
                f"{result.optimizer_iterations} iterations; "
                f"J={result.objective_value:.3f}; "
                f"planning time {planning_elapsed:.2f} s.\n"
                f"{len(joint_path)} samples; "
                f"max position error {max_position_error * 1000.0:.1f} mm; "
                f"max TCP-X error {max_axis_error_deg:.2f} deg.\n"
                "Weighted normalized costs: "
                + ", ".join(
                    f"{name}={value:.3f}"
                    for name, value in result.weighted_cost_components.items()
                )
                + ".\n"
                f"Core center: {np.array2string(core_center, precision=3)}"
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
    print("Generate a random core, choose a TCP start, then click Plan full pipeline.")
    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()