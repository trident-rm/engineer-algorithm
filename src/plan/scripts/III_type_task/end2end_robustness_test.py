#!/usr/bin/env python3
"""Run reproducible Type-III task trajectory robustness tests."""

import argparse
import csv
import json
import os
import tempfile
import time
from pathlib import Path

import numpy as np


INPUT_FIELDS = [
    "case_id",
    "seed",
    "core_position_parameterization",
    "orientation_parameterization",
    "theta_rad",
    "phi_rad",
    "alpha_rad",
    "core_pose",
    "core_x_m",
    "core_y_m",
    "core_z_m",
    "start_joint_positions",
    "radius_m",
    "hover_distance_m",
    "slide_distance_m",
    "turn_p_sweep_deg",
    "turn_q_sweep_deg",
    "steps_per_stage",
    "position_weight",
    "axis_weight",
    "smoothness_weight",
    "joint_limit_weight",
    "manipulability_weight",
    "position_tolerance_mm",
    "tcp_x_tolerance_deg",
]
STAGE_NAMES = ("approach", "insert", "slide", "turn_p", "turn_q")
CORE_POSITION_PARAMETERIZATION = "robomaster_core_center_xyz_mm_v1"
ORIENTATION_PARAMETERIZATION = "robomaster_theta_phi_alpha_axis_sequence_ccw_v4"
OUTPUT_FIELDS = [
    "status",
    "optimizer_success",
    "average_quality_pass",
    "peak_quality_pass",
    "quality_pass",
    "optimizer_iterations",
    "sample_count",
    "objective_value",
    "elapsed_s",
    "mean_position_error_mm",
    "p95_position_error_mm",
    "max_position_error_mm",
    "mean_tcp_x_error_deg",
    "p95_tcp_x_error_deg",
    "max_tcp_x_error_deg",
    "max_joint_step_deg",
    "min_joint_limit_margin_deg",
    "mean_manipulability_m3",
    "min_manipulability_m3",
    "cost_position",
    "cost_axis",
    "cost_smoothness",
    "cost_joint_limit",
    "cost_manipulability",
    "failure_reason",
]
for _stage_name in STAGE_NAMES:
    OUTPUT_FIELDS.extend(
        [
            f"{_stage_name}_max_position_error_mm",
            f"{_stage_name}_max_tcp_x_error_deg",
        ]
    )
CSV_FIELDS = INPUT_FIELDS + OUTPUT_FIELDS


def _json(value):
    return json.dumps(value, separators=(",", ":"), allow_nan=False)


def _write_cases(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", newline="", encoding="utf-8", dir=path.parent, delete=False
    ) as output:
        temporary_path = Path(output.name)
        writer = csv.DictWriter(output, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
        output.flush()
        os.fsync(output.fileno())
    os.replace(temporary_path, path)


def _write_summary(path, rows, args):
    completed = [row for row in rows if row.get("status") not in ("", "pending")]
    successes = [row for row in completed if row.get("status") == "success"]
    average_quality_passes = [
        row for row in completed if row.get("average_quality_pass") == "true"
    ]
    peak_quality_passes = [
        row for row in completed if row.get("peak_quality_pass") == "true"
    ]
    quality_passes = peak_quality_passes
    metrics = [
        ("requested_cases", args.cases, "cases"),
        ("completed_cases", len(completed), "cases"),
        ("optimizer_converged_cases", len(successes), "cases"),
        (
            "optimizer_convergence_rate_pct",
            100.0 * len(successes) / len(completed) if completed else 0.0,
            "%",
        ),
        ("planning_error_cases", sum(row.get("status") == "planning_error" for row in completed), "cases"),
        ("average_error_pass_cases", len(average_quality_passes), "cases"),
        (
            "average_error_pass_rate_pct",
            100.0 * len(average_quality_passes) / len(completed) if completed else 0.0,
            "%",
        ),
        ("peak_error_pass_cases", len(peak_quality_passes), "cases"),
        (
            "peak_error_pass_rate_pct",
            100.0 * len(peak_quality_passes) / len(completed) if completed else 0.0,
            "%",
        ),
        ("task_quality_pass_cases", len(quality_passes), "cases"),
        (
            "task_quality_pass_rate_pct",
            100.0 * len(quality_passes) / len(completed) if completed else 0.0,
            "%",
        ),
        ("position_tolerance_mm", args.position_tolerance_mm, "mm"),
        ("tcp_x_tolerance_deg", args.tcp_x_tolerance_deg, "deg"),
        ("position_weight", args.position_weight, ""),
        ("axis_weight", args.axis_weight, ""),
        ("core_position_parameterization", CORE_POSITION_PARAMETERIZATION, ""),
        ("orientation_parameterization", ORIENTATION_PARAMETERIZATION, ""),
        ("smoothness_weight", args.smoothness_weight, ""),
        ("joint_limit_weight", args.joint_limit_weight, ""),
        ("manipulability_weight", args.manipulability_weight, ""),
        ("seed", args.seed, ""),
        ("steps_per_stage", args.steps, "samples"),
    ]
    metric_fields = [
        ("elapsed_s", "runtime_s"),
        ("max_position_error_mm", "max_position_error_mm"),
        ("p95_position_error_mm", "p95_position_error_mm"),
        ("max_tcp_x_error_deg", "max_tcp_x_error_deg"),
        ("p95_tcp_x_error_deg", "p95_tcp_x_error_deg"),
        ("max_joint_step_deg", "max_joint_step_deg"),
        ("min_joint_limit_margin_deg", "min_joint_limit_margin_deg"),
        ("mean_manipulability_m3", "mean_manipulability_m3"),
        ("min_manipulability_m3", "min_manipulability_m3"),
        ("cost_position", "cost_position"),
        ("cost_axis", "cost_axis"),
        ("cost_smoothness", "cost_smoothness"),
        ("cost_joint_limit", "cost_joint_limit"),
        ("cost_manipulability", "cost_manipulability"),
    ]
    for field, label in metric_fields:
        values = np.asarray(
            [float(row[field]) for row in completed if row.get(field) not in ("", None)],
            dtype=np.float64,
        )
        if values.size:
            metrics.extend(
                [
                    (f"{label}_mean", float(np.mean(values)), ""),
                    (f"{label}_median", float(np.median(values)), ""),
                    (f"{label}_p95", float(np.percentile(values, 95)), ""),
                    (f"{label}_min", float(np.min(values)), ""),
                    (f"{label}_max", float(np.max(values)), ""),
                ]
            )
    for stage_name in STAGE_NAMES:
        for suffix, label in (
            ("max_position_error_mm", "position_error_mm"),
            ("max_tcp_x_error_deg", "tcp_x_error_deg"),
        ):
            field = f"{stage_name}_{suffix}"
            values = np.asarray(
                [float(row[field]) for row in completed if row.get(field) not in ("", None)],
                dtype=np.float64,
            )
            if values.size:
                metrics.extend(
                    [
                        (f"{stage_name}_{label}_mean_case_max", float(np.mean(values)), ""),
                        (f"{stage_name}_{label}_p95_case_max", float(np.percentile(values, 95)), ""),
                        (f"{stage_name}_{label}_worst", float(np.max(values)), ""),
                    ]
                )
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w", newline="", encoding="utf-8", dir=path.parent, delete=False
    ) as output:
        temporary_path = Path(output.name)
        writer = csv.writer(output)
        writer.writerow(["metric", "value", "unit"])
        writer.writerows(metrics)
        output.flush()
        os.fsync(output.fileno())
    os.replace(temporary_path, path)


def _read_cases(path):
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        rows = [
            {(key or "").strip(): value for key, value in row.items() if key}
            for row in reader
        ]
    if rows and not {"case_id", "core_pose"}.issubset(rows[0]):
        raise ValueError(
            f"{path} contains data rows without case_id/core_pose; move or convert it before testing"
        )
    for row in rows:
        for field in CSV_FIELDS:
            row.setdefault(field, "")
    return rows


def _validate_case_configuration(rows, args):
    expected = {
        "core_position_parameterization": CORE_POSITION_PARAMETERIZATION,
        "orientation_parameterization": ORIENTATION_PARAMETERIZATION,
        "steps_per_stage": float(args.steps),
        "radius_m": args.radius,
        "hover_distance_m": args.hover_distance,
        "slide_distance_m": args.slide_distance,
        "turn_p_sweep_deg": args.turn_p_sweep_deg,
        "turn_q_sweep_deg": args.turn_q_sweep_deg,
        "position_weight": args.position_weight,
        "axis_weight": args.axis_weight,
        "smoothness_weight": args.smoothness_weight,
        "joint_limit_weight": args.joint_limit_weight,
        "manipulability_weight": args.manipulability_weight,
    }
    mismatches = []
    for row in rows:
        for field, expected_value in expected.items():
            raw_value = row.get(field, "")
            matches = (
                raw_value == expected_value
                if field == "orientation_parameterization"
                else raw_value not in ("", None)
                and np.isclose(
                    float(raw_value), expected_value, rtol=0.0, atol=1e-12
                )
            )
            if not matches:
                mismatches.append(field)
                break
        if mismatches:
            break
    if mismatches:
        raise ValueError(
            f"{args.csv} contains cases with missing or different planning settings "
            f"(first mismatch: {mismatches[0]}). Use a new --csv path for this "
            "configuration; existing results were not changed."
        )


def _make_cases(demo, count, seed, start_cfg, args):
    rng = np.random.default_rng(seed)
    cases = []
    for case_index in range(count):
        core = demo._sample_random_core(rng)
        core_pose = {
            "position_world_m": core["center"].tolist(),
            "rotation_matrix": core["rotation"].tolist(),
        }
        cases.append(
            {
                "case_id": f"case-{case_index + 1:04d}",
                "seed": seed,
                "core_position_parameterization": CORE_POSITION_PARAMETERIZATION,
                "orientation_parameterization": ORIENTATION_PARAMETERIZATION,
                "theta_rad": core["theta"],
                "phi_rad": core["phi"],
                "alpha_rad": core["alpha"],
                "core_pose": _json(core_pose),
                "core_x_m": core["center"][0],
                "core_y_m": core["center"][1],
                "core_z_m": core["center"][2],
                "start_joint_positions": _json(start_cfg.tolist()),
                "radius_m": args.radius,
                "hover_distance_m": args.hover_distance,
                "slide_distance_m": args.slide_distance,
                "turn_p_sweep_deg": args.turn_p_sweep_deg,
                "turn_q_sweep_deg": args.turn_q_sweep_deg,
                "steps_per_stage": args.steps,
                "position_weight": args.position_weight,
                "axis_weight": args.axis_weight,
                "smoothness_weight": args.smoothness_weight,
                "joint_limit_weight": args.joint_limit_weight,
                "manipulability_weight": args.manipulability_weight,
                "position_tolerance_mm": args.position_tolerance_mm,
                "tcp_x_tolerance_deg": args.tcp_x_tolerance_deg,
                "status": "pending",
            }
        )
    return cases


def _make_targets(
    demo, make_circular_task_path, core, case
):
    core_center = np.asarray(core["center"], dtype=np.float64)
    q_axis = np.asarray(core["q_axis"], dtype=np.float64)
    p_axis = np.asarray(core["p_axis"], dtype=np.float64)
    up_axis = np.asarray(core["up_axis"], dtype=np.float64)
    hover_tcp_x, _, _ = demo._hover_tcp_x_frame(q_axis)

    hover_position = core_center + float(case["hover_distance_m"]) * q_axis
    insert_position = hover_position - float(case["hover_distance_m"]) * q_axis
    slide_position = insert_position + float(case["slide_distance_m"]) * up_axis
    steps = int(case["steps_per_stage"])
    insert_positions, insert_axes = demo._make_linear_task_segment(
        hover_position, insert_position, hover_tcp_x, hover_tcp_x, steps
    )
    slide_positions, slide_axes = demo._make_linear_task_segment(
        insert_position, slide_position, hover_tcp_x, hover_tcp_x, steps
    )

    radius = float(case["radius_m"])
    radius_vector_p = radius * q_axis
    p_direction = (
        1.0
        if np.dot(np.cross(p_axis, radius_vector_p), up_axis) >= 0.0
        else -1.0
    )
    turn_p_axis = p_direction * p_axis
    _, turn_p_positions, turn_p_axes = make_circular_task_path(
        start_position=slide_position,
        start_axis=-q_axis,
        rotation_axis=turn_p_axis,
        start_radial=radius_vector_p,
        sweep_rad=np.deg2rad(float(case["turn_p_sweep_deg"])),
        samples=steps,
        orientation_axis=turn_p_axis,
        orientation_sweep_rad=np.deg2rad(float(case["turn_p_sweep_deg"])),
    )

    turn_p_end_x = turn_p_axes[-1]
    turn_p_end_y = -p_axis
    turn_p_end_z = np.cross(turn_p_end_x, turn_p_end_y)
    turn_p_end_z /= np.linalg.norm(turn_p_end_z)
    turn_q_radial = -turn_p_axes[-1]
    turn_q_radial -= np.dot(turn_q_radial, turn_p_end_z) * turn_p_end_z
    turn_q_radial /= np.linalg.norm(turn_q_radial)
    _, turn_q_positions, turn_q_axes = make_circular_task_path(
        start_position=turn_p_positions[-1],
        start_axis=turn_p_axes[-1],
        rotation_axis=turn_p_end_z,
        start_radial=radius * turn_q_radial,
        sweep_rad=np.deg2rad(float(case["turn_q_sweep_deg"])),
        samples=steps,
        orientation_axis=turn_p_end_z,
        orientation_sweep_rad=np.deg2rad(float(case["turn_q_sweep_deg"])),
    )

    segments = [
        ("insert", insert_positions, insert_axes),
        ("slide", slide_positions, slide_axes),
        ("turn_p", turn_p_positions, turn_p_axes),
        ("turn_q", turn_q_positions, turn_q_axes),
    ]
    return hover_position, hover_tcp_x, segments


def _measure_manipulability(jax, jnp, robot, link_index, joint_path):
    jacobian_fn = jax.jacfwd(
        lambda configuration: robot.forward_kinematics(configuration)[link_index, 4:]
    )
    jacobians = jax.vmap(jacobian_fn)(jnp.asarray(joint_path))
    grams = jacobians @ jnp.swapaxes(jacobians, -1, -2)
    values = jnp.sqrt(jnp.maximum(jnp.linalg.det(grams), 0.0) + 1e-12)
    return np.asarray(values, dtype=np.float64)


def _run_case(
    row,
    demo,
    make_circular_task_path,
    jax,
    jaxlie,
    jnp,
    robot,
    link_index,
    planner,
):
    started = time.perf_counter()
    pose = json.loads(row["core_pose"])
    rotation = np.asarray(pose["rotation_matrix"], dtype=np.float64)
    core = {
        "center": np.asarray(pose["position_world_m"], dtype=np.float64),
        "rotation": rotation,
        "q_axis": rotation[:, 0],
        "p_axis": rotation[:, 1],
        "up_axis": rotation[:, 2],
    }
    start_cfg = np.asarray(json.loads(row["start_joint_positions"]), dtype=np.float64)
    start_position, start_wxyz = demo._fk_pose(robot, link_index, start_cfg)
    start_tcp_x = np.asarray(
        jaxlie.SO3(jnp.asarray(start_wxyz)).apply(jnp.array([1.0, 0.0, 0.0]))
    )
    hover_position, hover_tcp_x, type_iii_segments = _make_targets(
        demo,
        make_circular_task_path,
        core,
        row,
    )
    approach_positions, approach_axes = demo._make_linear_task_segment(
        start_position,
        hover_position,
        start_tcp_x,
        hover_tcp_x,
        int(row["steps_per_stage"]),
    )
    segments = [("approach", approach_positions, approach_axes), *type_iii_segments]
    target_positions, target_axes, stage_ranges = demo._combine_task_segments(
        segments
    )
    position_weights = np.ones(len(target_positions), dtype=np.float64)
    axis_weights = np.ones(len(target_axes), dtype=np.float64)
    _, hover_index = stage_ranges["approach"]
    position_weights[hover_index] = 10.0
    axis_weights[hover_index] = 10.0
    result = planner.plan(
        start_cfg,
        target_positions,
        target_axes,
        position_weights=position_weights,
        axis_weights=axis_weights,
    )
    joint_path = result.joint_positions
    position_errors = result.position_errors
    axis_errors_deg = np.rad2deg(result.axis_errors_rad)
    joint_steps_deg = np.rad2deg(np.abs(np.diff(joint_path, axis=0)))
    margins = np.concatenate(
        [
            joint_path - planner.lower_limits,
            planner.upper_limits - joint_path,
        ],
        axis=1,
    )
    manipulability = _measure_manipulability(jax, jnp, robot, link_index, joint_path)

    average_quality_pass = (
        np.mean(position_errors) * 1000.0
        <= float(row["position_tolerance_mm"])
        and np.mean(axis_errors_deg) <= float(row["tcp_x_tolerance_deg"])
        and np.min(margins) >= -1e-8
    )
    peak_quality_pass = (
        np.max(position_errors) * 1000.0
        <= float(row["position_tolerance_mm"])
        and np.max(axis_errors_deg) <= float(row["tcp_x_tolerance_deg"])
        and np.min(margins) >= -1e-8
    )
    row.update(
        {
            "status": "success" if result.optimizer_success else "optimizer_stopped",
            "optimizer_success": str(bool(result.optimizer_success)).lower(),
            "average_quality_pass": str(bool(average_quality_pass)).lower(),
            "peak_quality_pass": str(bool(peak_quality_pass)).lower(),
            "quality_pass": str(bool(peak_quality_pass)).lower(),
            "optimizer_iterations": result.optimizer_iterations,
            "sample_count": len(joint_path),
            "objective_value": result.objective_value,
            "elapsed_s": time.perf_counter() - started,
            "mean_position_error_mm": np.mean(position_errors) * 1000.0,
            "p95_position_error_mm": np.percentile(position_errors, 95) * 1000.0,
            "max_position_error_mm": np.max(position_errors) * 1000.0,
            "mean_tcp_x_error_deg": np.mean(axis_errors_deg),
            "p95_tcp_x_error_deg": np.percentile(axis_errors_deg, 95),
            "max_tcp_x_error_deg": np.max(axis_errors_deg),
            "max_joint_step_deg": np.max(joint_steps_deg),
            "min_joint_limit_margin_deg": np.rad2deg(np.min(margins)),
            "mean_manipulability_m3": np.mean(manipulability),
            "min_manipulability_m3": np.min(manipulability),
            "failure_reason": "" if result.optimizer_success else "optimizer stopped without convergence",
        }
    )
    cost_fields = {
        "position": "cost_position",
        "axis": "cost_axis",
        "smoothness": "cost_smoothness",
        "joint_limit": "cost_joint_limit",
        "manipulability": "cost_manipulability",
    }
    for name, field in cost_fields.items():
        row[field] = result.weighted_cost_components[name]
    for stage_name, (start_index, end_index) in stage_ranges.items():
        row[f"{stage_name}_max_position_error_mm"] = (
            np.nanmax(position_errors[start_index : end_index + 1]) * 1000.0
        )
        row[f"{stage_name}_max_tcp_x_error_deg"] = np.max(
            np.nan_to_num(
                axis_errors_deg[start_index : end_index + 1], nan=-np.inf
            )
        )
    return row


def _print_summary(rows):
    completed = [row for row in rows if row.get("status") not in ("", "pending")]
    successes = [row for row in completed if row.get("status") == "success"]
    errors = [row for row in completed if row.get("status") == "planning_error"]
    print(f"Cases completed: {len(completed)}/{len(rows)}")
    if not completed:
        return
    print(
        f"Optimizer converged: {len(successes)}/{len(completed)} "
        f"({100.0 * len(successes) / len(completed):.1f}%); "
        f"planning errors: {len(errors)}"
    )
    average_quality_passes = [
        row for row in completed if row.get("average_quality_pass") == "true"
    ]
    peak_quality_passes = [
        row for row in completed if row.get("peak_quality_pass") == "true"
    ]
    print(
        f"Average-error pass (position/orientation/limits): "
        f"{len(average_quality_passes)}/{len(completed)} "
        f"({100.0 * len(average_quality_passes) / len(completed):.1f}%)"
    )
    print(
        f"Peak-error pass (position/orientation/limits): "
        f"{len(peak_quality_passes)}/{len(completed)} "
        f"({100.0 * len(peak_quality_passes) / len(completed):.1f}%)"
    )
    for field, label in (
        ("elapsed_s", "runtime seconds"),
        ("max_position_error_mm", "max position error mm"),
        ("p95_position_error_mm", "p95 position error mm"),
        ("max_tcp_x_error_deg", "max TCP-X error degrees"),
        ("p95_tcp_x_error_deg", "p95 TCP-X error degrees"),
        ("max_joint_step_deg", "max joint step degrees"),
        ("min_joint_limit_margin_deg", "minimum joint-limit margin degrees"),
        ("min_manipulability_m3", "minimum manipulability m^3"),
    ):
        values = np.asarray(
            [float(row[field]) for row in completed if row.get(field) not in ("", None)],
            dtype=np.float64,
        )
        if values.size:
            print(
                f"{label}: mean={np.mean(values):.4g}, "
                f"p95={np.percentile(values, 95):.4g}, max={np.max(values):.4g}"
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path(__file__).with_name("test_result_type_iii_demo_aligned.csv"),
    )
    parser.add_argument("--cases", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=20260930)
    parser.add_argument("--steps", type=int, default=8, help="samples per path stage")
    parser.add_argument("--radius", type=float, default=0.1)
    parser.add_argument("--hover-distance", type=float, default=0.20)
    parser.add_argument("--slide-distance", type=float, default=0.10)
    parser.add_argument("--turn-p-sweep-deg", type=float, default=90.0)
    parser.add_argument("--turn-q-sweep-deg", type=float, default=60.0)
    parser.add_argument("--position-weight", type=float, default=1.0)
    parser.add_argument("--axis-weight", type=float, default=1.55)
    parser.add_argument("--smoothness-weight", type=float, default=0.0)
    parser.add_argument("--joint-limit-weight", type=float, default=0.0)
    parser.add_argument("--manipulability-weight", type=float, default=0.0)
    parser.add_argument("--position-tolerance-mm", type=float, default=50.0)
    parser.add_argument("--tcp-x-tolerance-deg", type=float, default=30.0)
    parser.add_argument("--ee-link", default="gripper")
    parser.add_argument("--urdf", type=Path, default=Path(
        "/home/hku/robo_arm/src/arms/combine_urdf/urdf/combine_urdf.urdf"
    ))
    parser.add_argument("--limit", type=int, help="run at most this many pending cases")
    parser.add_argument("--checkpoint-every", type=int, default=10)
    parser.add_argument("--retry-failed", action="store_true")
    args = parser.parse_args()
    if args.cases <= 0 or not 3 <= args.steps <= 20 or args.checkpoint_every <= 0:
        parser.error(
            "cases/checkpoint-every must be positive and steps must be between 3 and 20"
        )
    if args.limit is not None and args.limit <= 0:
        parser.error("limit must be positive")
    configured_weights = (
        args.position_weight,
        args.axis_weight,
        args.smoothness_weight,
        args.joint_limit_weight,
        args.manipulability_weight,
    )
    if any(
        not np.isfinite(weight) or weight < 0.0 or weight > 10.0
        for weight in configured_weights
    ):
        parser.error("normalized weights must be finite and between 0 and 10")

    import end2end_plan_demo as demo

    demo._patch_numpy_asarray_copy_kwarg()
    robot, _, pyroki_snippets = demo._load_robot(args.urdf.expanduser().resolve())
    import jax
    import jax.numpy as jnp
    import jaxlie

    from III_type_task_planner import CostWeights, TypeIIIPlanner, make_circular_task_path

    joint_names = list(robot.joints.actuated_names)
    link_names = list(robot.links.names)
    if args.ee_link not in link_names:
        parser.error(f"end-effector link '{args.ee_link}' is not in the URDF")
    link_index = link_names.index(args.ee_link)
    start_cfg = np.zeros(len(joint_names), dtype=np.float64)
    for index, joint_name in enumerate(joint_names):
        if joint_name == "joint2":
            start_cfg[index] = 1.9
        elif joint_name == "joint3":
            start_cfg[index] = 0.5
        elif joint_name == "joint5":
            start_cfg[index] = -1.6
    start_position, start_wxyz = demo._fk_pose(robot, link_index, start_cfg)
    start_cfg = np.asarray(
        pyroki_snippets.solve_ik(
            robot=robot,
            target_link_name=args.ee_link,
            target_position=start_position,
            target_wxyz=start_wxyz,
        ),
        dtype=np.float64,
    )
    resolved_start_position, resolved_start_wxyz = demo._fk_pose(
        robot, link_index, start_cfg
    )
    start_position_error = float(
        np.linalg.norm(resolved_start_position - start_position)
    )
    start_orientation_error = float(
        demo._quaternion_error_rad(resolved_start_wxyz, start_wxyz)
    )
    if start_position_error > 0.005 or start_orientation_error > np.deg2rad(3.0):
        raise RuntimeError(
            "Default start pose has no accurate IK solution: "
            f"position error={start_position_error:.4f} m, "
            f"orientation error={np.rad2deg(start_orientation_error):.2f} deg"
        )

    rows = _read_cases(args.csv)
    if not rows:
        rows = _make_cases(demo, args.cases, args.seed, start_cfg, args)
        _write_cases(args.csv, rows)
        print(f"Generated {len(rows)} deterministic core-pose cases (seed={args.seed}).")
    else:
        _validate_case_configuration(rows, args)
    for row in rows:
        row["position_tolerance_mm"] = args.position_tolerance_mm
        row["tcp_x_tolerance_deg"] = args.tcp_x_tolerance_deg
        if row.get("max_position_error_mm") not in ("", None):
            average_quality_pass = (
                float(row["mean_position_error_mm"])
                <= float(row["position_tolerance_mm"])
                and float(row["mean_tcp_x_error_deg"])
                <= float(row["tcp_x_tolerance_deg"])
                and float(row["min_joint_limit_margin_deg"]) >= -1e-8
            )
            peak_quality_pass = (
                float(row["max_position_error_mm"])
                <= float(row["position_tolerance_mm"])
                and float(row["max_tcp_x_error_deg"])
                <= float(row["tcp_x_tolerance_deg"])
                and float(row["min_joint_limit_margin_deg"]) >= -1e-8
            )
            row["average_quality_pass"] = str(average_quality_pass).lower()
            row["peak_quality_pass"] = str(peak_quality_pass).lower()
            row["quality_pass"] = str(peak_quality_pass).lower()
    planner = TypeIIIPlanner(
        robot,
        link_index,
        weights=CostWeights(
            position=args.position_weight,
            axis_alignment=args.axis_weight,
            smoothness=args.smoothness_weight,
            joint_limit=args.joint_limit_weight,
            manipulability=args.manipulability_weight,
        ),
    )
    retryable_statuses = {"optimizer_stopped", "planning_error"}
    pending = [
        row
        for row in rows
        if row.get("status") in ("", "pending")
        or (args.retry_failed and row.get("status") in retryable_statuses)
    ]
    if args.limit is not None:
        pending = pending[: args.limit]

    warmup_started = time.perf_counter()
    for steps_per_stage in sorted(
        {int(row["steps_per_stage"]) for row in pending}
    ):
        planner.warm_up(start_cfg, sample_count=5 * steps_per_stage + 1)
    if pending:
        print(
            "Completed planner JAX warm-up in "
            f"{time.perf_counter() - warmup_started:.2f} s."
        )

    print(f"Running {len(pending)} cases; each path uses {args.steps} samples per stage.")
    try:
        for completed_count, row in enumerate(pending, start=1):
            case_started = time.perf_counter()
            try:
                _run_case(
                    row,
                    demo,
                    make_circular_task_path,
                    jax,
                    jaxlie,
                    jnp,
                    robot,
                    link_index,
                    planner,
                )
            except Exception as exc:
                row["status"] = "planning_error"
                row["elapsed_s"] = time.perf_counter() - case_started
                row["failure_reason"] = f"{type(exc).__name__}: {exc}"[:500]
            if completed_count % args.checkpoint_every == 0:
                _write_cases(args.csv, rows)
            if completed_count == 1 or completed_count % 10 == 0 or completed_count == len(pending):
                print(
                    f"[{completed_count}/{len(pending)}] {row['case_id']}: "
                    f"{row['status']} ({float(row.get('elapsed_s') or 0):.2f}s)"
                )
    except KeyboardInterrupt:
        print("Interrupted; saving completed case results before exit.")
    finally:
        _write_cases(args.csv, rows)

    _print_summary(rows)
    summary_path = args.csv.with_name(f"{args.csv.stem}_summary.csv")
    _write_summary(summary_path, rows, args)
    print(f"Summary written to {summary_path.resolve()}")
    print(f"Results written to {args.csv.resolve()}")


if __name__ == "__main__":
    main()