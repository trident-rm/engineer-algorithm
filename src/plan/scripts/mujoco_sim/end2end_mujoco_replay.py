#!/usr/bin/env python3
"""Plan a Type-III task path and replay it with MuJoCo position actuators."""

import argparse
import csv
import json
import os
import queue
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from contextlib import nullcontext
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
PLANNER_DIRECTORY = SCRIPT_DIRECTORY.parent / "III_type_task"
CANONICAL_URDF_PATH = (
    SCRIPT_DIRECTORY.parents[2]
    / "arms"
    / "combine_urdf"
    / "urdf"
    / "combine_urdf.urdf"
)
MUJOCO_MODEL_PATH = SCRIPT_DIRECTORY / "mujoco_assets" / "robot_scene.xml"
if str(PLANNER_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(PLANNER_DIRECTORY))


def _mesh_path(filename, urdf_path):
    path = Path(filename)
    if path.is_absolute():
        return path
    prefix = "package://"
    if filename.startswith(prefix):
        package_path = filename[len(prefix) :]
        package, relative_path = package_path.split("/", 1)
        package_root = urdf_path.parent.parent
        if package_root.name == package:
            return package_root / relative_path
    return urdf_path.parent / Path(filename).name


def _root_link_name(root):
    links = {link.get("name") for link in root.findall("link")}
    child_links = {
        child.get("link")
        for joint in root.findall("joint")
        if (child := joint.find("child")) is not None
    }
    roots = links - child_links
    if len(roots) != 1:
        raise ValueError(f"Expected one URDF root link, found {sorted(roots)}")
    return roots.pop()


def _make_hull_mesh(source, destination):
    mesh = trimesh.load_mesh(source, process=False)
    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        raise ValueError(f"Expected a triangle mesh with faces: {source}")
    hull = mesh.convex_hull
    if len(hull.faces) == 0:
        raise ValueError(f"Could not construct a collision hull for {source}")
    hull.export(destination, file_type="stl")


def _write_collision_urdf(urdf_path, core, dimensions, asset_directory, output_path):
    root = ET.parse(urdf_path).getroot()
    mesh_directory = asset_directory / "hulls"
    mesh_directory.mkdir()

    for link in root.findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("./geometry/mesh")
            if mesh is None or not mesh.get("filename"):
                continue
            source = _mesh_path(mesh.get("filename"), urdf_path).resolve()
            if not source.is_file():
                raise FileNotFoundError(
                    f"URDF visual mesh does not exist: {source}"
                )
            mesh.set("filename", str(source))

        collisions = link.findall("collision")
        collision_origins = [
            ET.tostring(origin, encoding="unicode")
            for collision in collisions
            if (origin := collision.find("origin")) is not None
        ]
        mesh_filename = None
        for collision in collisions:
            mesh = collision.find("./geometry/mesh")
            if mesh is not None and mesh.get("filename"):
                mesh_filename = mesh.get("filename")
                break
        for element in list(link):
            if element.tag == "collision":
                link.remove(element)

        if mesh_filename is None:
            continue
        source = _mesh_path(mesh_filename, urdf_path)
        if not source.is_file():
            raise FileNotFoundError(f"URDF collision mesh does not exist: {source}")
        hull_name = f"{link.attrib['name']}.stl"
        hull_path = mesh_directory / hull_name
        _make_hull_mesh(source, hull_path)
        collision = ET.SubElement(link, "collision")
        collision.set("name", f"{link.attrib['name']}_hull")
        if collision_origins:
            collision.append(ET.fromstring(collision_origins[0]))
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "mesh", filename=hull_name)

    mujoco_extension = root.find("mujoco")
    if mujoco_extension is None:
        mujoco_extension = ET.SubElement(root, "mujoco")
    compiler = mujoco_extension.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(mujoco_extension, "compiler")
    compiler.set("meshdir", str(mesh_directory))

    root_link = _root_link_name(root)
    core_link = ET.SubElement(root, "link", name="simulation_core")
    core_collision = ET.SubElement(core_link, "collision", name="simulation_core_box")
    core_geometry = ET.SubElement(core_collision, "geometry")
    ET.SubElement(
        core_geometry,
        "box",
        size=" ".join(f"{value:.9g}" for value in dimensions),
    )
    core_origin = Rotation.from_matrix(core["rotation"]).as_euler("xyz")
    core_joint = ET.SubElement(root, "joint", name="simulation_core_fixed", type="fixed")
    ET.SubElement(
        core_joint,
        "origin",
        xyz=" ".join(f"{value:.9g}" for value in core["center"]),
        rpy=" ".join(f"{value:.12g}" for value in core_origin),
    )
    ET.SubElement(core_joint, "parent", link=root_link)
    ET.SubElement(core_joint, "child", link="simulation_core")

    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)


def _tcp_fixed_transform(urdf_path, ee_link):
    root = ET.parse(urdf_path).getroot()
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is None or child.get("link") != ee_link:
            continue
        if joint.get("type") != "fixed":
            raise ValueError(f"MuJoCo TCP link '{ee_link}' must use a fixed joint")
        parent = joint.find("parent")
        if parent is None or parent.get("link") is None:
            raise ValueError(f"TCP joint for '{ee_link}' has no parent link")
        origin = joint.find("origin")
        if origin is None:
            return parent.get("link"), np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        xyzw = Rotation.from_euler("xyz", rpy).as_quat()
        return parent.get("link"), xyz, xyzw[[3, 0, 1, 2]]
    raise ValueError(f"Could not find a fixed URDF joint for TCP link '{ee_link}'")


def _add_simulation_elements(
    mjcf_path,
    model_path,
    joint_names,
    lower,
    upper,
    tcp_parent_link,
    tcp_transform,
    args,
):
    root = ET.parse(mjcf_path).getroot()
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", f"{args.timestep:.9g}")
    option.set("gravity", "0 0 -9.81")
    option.set("integrator", "implicitfast")

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MuJoCo model has no worldbody")
    tcp_parent_body = worldbody.find(f".//body[@name='{tcp_parent_link}']")
    if tcp_parent_body is None:
        raise ValueError(f"MuJoCo model has no TCP parent body '{tcp_parent_link}'")
    ET.SubElement(
        tcp_parent_body,
        "site",
        name="tcp_site",
        pos=" ".join(f"{value:.12g}" for value in tcp_transform[0]),
        quat=" ".join(f"{value:.12g}" for value in tcp_transform[1]),
        size="0.004",
    )
    ET.SubElement(
        worldbody,
        "geom",
        name="simulation_ground",
        type="plane",
        size="2 2 0.1",
        pos="0 0 -0.25",
        friction="0.8 0.02 0.002",
        rgba="0.35 0.35 0.35 1",
    )
    for geom in worldbody.findall(".//geom"):
        name = geom.get("name", "")
        if name == "simulation_core_box":
            geom.set("contype", "2")
            geom.set("conaffinity", "1")
        elif name == "simulation_ground":
            geom.set("contype", "0")
            geom.set("conaffinity", "1")
        elif name.endswith("_hull"):
            geom.set("contype", "1")
            geom.set("conaffinity", "0")

    actuators = root.find("actuator")
    if actuators is None:
        actuators = ET.SubElement(root, "actuator")
    for index, joint_name in enumerate(joint_names):
        ET.SubElement(
            actuators,
            "position",
            name=f"{joint_name}_position",
            joint=joint_name,
            kp=f"{args.kp:.9g}",
            kv=f"{args.kv:.9g}",
            ctrlrange=f"{lower[index]:.12g} {upper[index]:.12g}",
            ctrllimited="true",
            forcerange=f"{-args.max_torque:.9g} {args.max_torque:.9g}",
            forcelimited="true",
        )
    ET.ElementTree(root).write(model_path, encoding="unicode", xml_declaration=True)


def _build_model(mujoco, urdf_path, core, args, joint_names, lower, upper):
    if urdf_path.resolve() == CANONICAL_URDF_PATH.resolve():
        if not MUJOCO_MODEL_PATH.is_file():
            raise FileNotFoundError(
                f"Canonical MuJoCo model is missing: {MUJOCO_MODEL_PATH}. "
                "Generate it with generate_mujoco_model.py."
            )
        model = mujoco.MjModel.from_xml_path(str(MUJOCO_MODEL_PATH))
    else:
        temporary_directory = tempfile.TemporaryDirectory(prefix="type3_mujoco_")
        asset_directory = Path(temporary_directory.name)
        collision_urdf = asset_directory / "robot_collision.urdf"
        imported_mjcf = asset_directory / "imported.xml"
        controlled_mjcf = asset_directory / "controlled.xml"
        _write_collision_urdf(
            urdf_path,
            core,
            np.asarray(args.core_dimensions, dtype=np.float64),
            asset_directory,
            collision_urdf,
        )
        tcp_parent_link, tcp_position, tcp_quaternion = _tcp_fixed_transform(
            urdf_path, args.ee_link
        )
        imported_model = mujoco.MjModel.from_xml_path(str(collision_urdf))
        mujoco.mj_saveLastXML(str(imported_mjcf), imported_model)
        _add_simulation_elements(
            imported_mjcf,
            controlled_mjcf,
            joint_names,
            lower,
            upper,
            tcp_parent_link,
            (tcp_position, tcp_quaternion),
            args,
        )
        model = mujoco.MjModel.from_xml_path(str(controlled_mjcf))

    model.opt.timestep = args.timestep
    core_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "simulation_core_box"
    )
    if core_geom_id < 0:
        raise ValueError("MuJoCo model is missing the simulation core geom")
    model.geom_pos[core_geom_id] = core["center"]
    model.geom_quat[core_geom_id] = Rotation.from_matrix(core["rotation"]).as_quat()[
        [3, 0, 1, 2]
    ]
    model.geom_size[core_geom_id] = np.asarray(args.core_dimensions) / 2.0
    for joint_name in joint_names:
        actuator_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{joint_name}_position"
        )
        if actuator_id < 0:
            raise ValueError(f"MuJoCo model is missing actuator for {joint_name}")
        model.actuator_gainprm[actuator_id, 0] = args.kp
        model.actuator_biasprm[actuator_id, 1] = -args.kp
        model.actuator_biasprm[actuator_id, 2] = -args.kv
        model.actuator_forcerange[actuator_id] = (-args.max_torque, args.max_torque)
    if urdf_path.resolve() == CANONICAL_URDF_PATH.resolve():
        return None, model
    return temporary_directory, model


def _interpolate_rows(times, values, time_value, normalize=False):
    right = int(np.searchsorted(times, time_value, side="right"))
    if right <= 0:
        return values[0].copy()
    if right >= len(times):
        return values[-1].copy()
    left = right - 1
    fraction = (time_value - times[left]) / (times[right] - times[left])
    value = values[left] + fraction * (values[right] - values[left])
    if normalize:
        norm = np.linalg.norm(value)
        if norm > 1e-12:
            value /= norm
    return value


def _rotation_error_deg(first, second):
    dot = float(np.clip(np.dot(first, second), -1.0, 1.0))
    return float(np.rad2deg(np.arccos(dot)))


def _plan_core(
    demo,
    make_circular_task_path,
    planner,
    start_cfg,
    start_position,
    start_tcp_x,
    core,
    args,
):
    core_center = np.asarray(core["center"], dtype=np.float64)
    q_axis = np.asarray(core["q_axis"], dtype=np.float64)
    p_axis = np.asarray(core["p_axis"], dtype=np.float64)
    up_axis = np.asarray(core["up_axis"], dtype=np.float64)
    hover_position = core_center + args.hover_distance * q_axis
    insert_position = hover_position - args.hover_distance * q_axis
    slide_position = insert_position + args.slide_distance * up_axis
    hover_tcp_x, _, _ = demo._hover_tcp_x_frame(q_axis)
    steps = args.steps
    approach_positions, approach_axes = demo._make_linear_task_segment(
        start_position, hover_position, start_tcp_x, hover_tcp_x, steps
    )
    insert_positions, insert_axes = demo._make_linear_task_segment(
        hover_position, insert_position, hover_tcp_x, hover_tcp_x, steps
    )
    slide_positions, slide_axes = demo._make_linear_task_segment(
        insert_position, slide_position, hover_tcp_x, hover_tcp_x, steps
    )
    radius_vector_p = args.radius * q_axis
    p_direction = (
        1.0 if np.dot(np.cross(p_axis, radius_vector_p), up_axis) >= 0.0 else -1.0
    )
    turn_p_axis = p_direction * p_axis
    _, turn_p_positions, turn_p_axes = make_circular_task_path(
        start_position=slide_position,
        start_axis=-q_axis,
        rotation_axis=turn_p_axis,
        start_radial=radius_vector_p,
        sweep_rad=np.deg2rad(args.sweep_deg),
        samples=steps,
        orientation_axis=turn_p_axis,
        orientation_sweep_rad=np.deg2rad(args.sweep_deg),
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
    _, turn_q_positions, turn_q_axes = make_circular_task_path(
        start_position=turn_p_positions[-1],
        start_axis=turn_p_axes[-1],
        rotation_axis=turn_p_end_z,
        start_radial=args.radius * turn_q_radial,
        sweep_rad=np.deg2rad(args.turn_q_sweep_deg),
        samples=steps,
        orientation_axis=turn_p_end_z,
        orientation_sweep_rad=np.deg2rad(args.turn_q_sweep_deg),
    )
    target_positions, target_axes, stage_ranges = demo._combine_task_segments(
        [
            ("Approach", approach_positions, approach_axes),
            ("Insert", insert_positions, insert_axes),
            ("Slide", slide_positions, slide_axes),
            ("Turn P", turn_p_positions, turn_p_axes),
            ("Turn Q", turn_q_positions, turn_q_axes),
        ]
    )
    position_weights = np.ones(len(target_positions), dtype=np.float64)
    axis_weights = np.ones(len(target_axes), dtype=np.float64)
    hover_index = stage_ranges["Approach"][1]
    position_weights[hover_index] = 10.0
    axis_weights[hover_index] = 10.0
    result = planner.plan(
        start_cfg,
        target_positions,
        target_axes,
        position_weights=position_weights,
        axis_weights=axis_weights,
    )
    if not np.all(np.isfinite(result.joint_positions)):
        raise RuntimeError("Planner returned non-finite joint values")
    return {
        "core": core,
        "positions": target_positions,
        "axes": target_axes,
        "result": result,
        "stage_ranges": stage_ranges,
        "core_center": core_center,
    }


def _replay(
    mujoco,
    model,
    joint_names,
    plan,
    args,
    output_path,
    replan_callback,
    rng,
):
    data = mujoco.MjData(model)
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        for name in joint_names
    ]
    if any(joint_id < 0 for joint_id in joint_ids):
        raise ValueError("MuJoCo model is missing one or more robot joints")
    qpos_addresses = np.asarray(
        [model.jnt_qposadr[joint_id] for joint_id in joint_ids], dtype=int
    )
    actuator_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_position")
        for name in joint_names
    ]
    if any(actuator_id < 0 for actuator_id in actuator_ids):
        raise ValueError("MuJoCo model is missing one or more joint actuators")
    tcp_site_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_SITE, f"tcp_site_{args.ee_link}"
    )
    if tcp_site_id < 0:
        tcp_site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "tcp_site"
        )
    core_geom_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "simulation_core_box"
    )
    if tcp_site_id < 0 or core_geom_id < 0:
        raise ValueError("MuJoCo model is missing the TCP site or simulation core geom")

    waypoint_times = np.linspace(
        0.0, args.duration, len(plan["result"].joint_positions)
    )
    simulation_steps = int(np.ceil(args.duration / args.timestep))
    sample_rows = []
    replay_history = []
    replay_index = 0
    total_contact_count = 0
    core_contact_count = 0
    contact_steps = 0
    core_contact_steps = 0
    max_contact_force = 0.0
    max_penetration = 0.0
    max_position_error = 0.0
    max_axis_error = 0.0
    max_joint_error = 0.0
    joint_squared_error = 0.0
    position_squared_error = 0.0
    axis_squared_error = 0.0
    max_force = 0.0
    saturated_steps = 0
    start_position_error = 0.0
    start_axis_error = 0.0

    def record_sample(time_value):
        nonlocal total_contact_count, core_contact_count
        nonlocal contact_steps, core_contact_steps
        nonlocal max_contact_force, max_penetration
        nonlocal max_position_error, max_axis_error, max_joint_error
        nonlocal joint_squared_error, position_squared_error, axis_squared_error
        nonlocal max_force, saturated_steps

        joint_path = plan["result"].joint_positions
        positions = plan["positions"]
        axes = plan["axes"]
        target_joints = _interpolate_rows(waypoint_times, joint_path, time_value)
        target_position = _interpolate_rows(waypoint_times, positions, time_value)
        target_axis = _interpolate_rows(
            waypoint_times, axes, time_value, normalize=True
        )

        actual_joints = data.qpos[qpos_addresses].copy()
        actual_position = data.site_xpos[tcp_site_id].copy()
        actual_axis = data.site_xmat[tcp_site_id].reshape(3, 3)[:, 0].copy()
        position_error = float(np.linalg.norm(actual_position - target_position))
        axis_error = _rotation_error_deg(actual_axis, target_axis)
        joint_errors = actual_joints - target_joints
        joint_error = float(np.max(np.abs(joint_errors)))
        max_position_error = max(max_position_error, position_error)
        max_axis_error = max(max_axis_error, axis_error)
        max_joint_error = max(max_joint_error, joint_error)
        position_squared_error += position_error**2
        axis_squared_error += axis_error**2
        joint_squared_error += float(np.mean(joint_errors**2))

        force_values = np.abs(data.actuator_force[actuator_ids])
        max_force = max(max_force, float(np.max(force_values)))
        saturated_steps += int(np.any(force_values >= args.max_torque * 0.999))

        contacts = []
        core_contacts = 0
        step_max_force = 0.0
        step_max_penetration = 0.0
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            is_core_contact = core_geom_id in (contact.geom1, contact.geom2)
            core_contacts += int(is_core_contact)
            contact_force = np.zeros(6, dtype=np.float64)
            mujoco.mj_contactForce(model, data, contact_index, contact_force)
            normal_force = float(abs(contact_force[0]))
            penetration = max(0.0, -float(contact.dist))
            contacts.append(
                {
                    "geom1": mujoco.mj_id2name(
                        model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
                    ),
                    "geom2": mujoco.mj_id2name(
                        model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
                    ),
                    "normal_force_n": normal_force,
                    "penetration_m": penetration,
                    "core_contact": is_core_contact,
                }
            )
            step_max_force = max(step_max_force, normal_force)
            step_max_penetration = max(step_max_penetration, penetration)

        total_contact_count += data.ncon
        core_contact_count += core_contacts
        contact_steps += int(data.ncon > 0)
        core_contact_steps += int(core_contacts > 0)
        max_contact_force = max(max_contact_force, step_max_force)
        max_penetration = max(max_penetration, step_max_penetration)
        sample_rows.append(
            {
                "replay_index": replay_index + 1,
                "time_s": time_value,
                "target_position_x_m": target_position[0],
                "target_position_y_m": target_position[1],
                "target_position_z_m": target_position[2],
                "actual_position_x_m": actual_position[0],
                "actual_position_y_m": actual_position[1],
                "actual_position_z_m": actual_position[2],
                "position_error_m": position_error,
                "target_tcp_x_x": target_axis[0],
                "target_tcp_x_y": target_axis[1],
                "target_tcp_x_z": target_axis[2],
                "actual_tcp_x_x": actual_axis[0],
                "actual_tcp_x_y": actual_axis[1],
                "actual_tcp_x_z": actual_axis[2],
                "tcp_x_error_deg": axis_error,
                "joint_error_max_rad": joint_error,
                "actuator_force_max_nm": float(np.max(force_values)),
                "contact_count": data.ncon,
                "core_contact_count": core_contacts,
                "contact_max_normal_force_n": step_max_force,
                "contact_max_penetration_m": step_max_penetration,
                "contacts_json": json.dumps(contacts, separators=(",", ":")),
            }
        )

    def run_segment(viewer):
        nonlocal replay_index, waypoint_times
        nonlocal start_position_error, start_axis_error
        joint_path = plan["result"].joint_positions
        waypoint_times = np.linspace(0.0, args.duration, len(joint_path))
        positions = plan["positions"]
        axes = plan["axes"]
        with viewer.lock() if viewer is not None else nullcontext():
            data.time = 0.0
            data.qpos[qpos_addresses] = joint_path[0]
            data.qvel[:] = 0.0
            data.ctrl[actuator_ids] = joint_path[0]
            mujoco.mj_forward(model, data)
            initial_position = data.site_xpos[tcp_site_id].copy()
            initial_axis = data.site_xmat[tcp_site_id].reshape(3, 3)[:, 0].copy()
        segment_start_position_error = float(
            np.linalg.norm(initial_position - positions[0])
        )
        segment_start_axis_error = _rotation_error_deg(initial_axis, axes[0])
        if replay_index == 0:
            start_position_error = segment_start_position_error
            start_axis_error = segment_start_axis_error
        if segment_start_position_error > 0.01 or segment_start_axis_error > 5.0:
            raise RuntimeError(
                "MuJoCo/Planner FK mismatch at replay start: "
                f"position={segment_start_position_error:.4f} m, "
                f"TCP-X={segment_start_axis_error:.2f} deg"
            )
        if viewer is not None:
            viewer.sync()
        simulation_wall_start = time.perf_counter()
        with viewer.lock() if viewer is not None else nullcontext():
            record_sample(0.0)
        for step in range(1, simulation_steps + 1):
            if viewer is not None and not viewer.is_running():
                break
            time_value = min(step * args.timestep, args.duration)
            command_time = max(0.0, time_value - args.timestep)
            with viewer.lock() if viewer is not None else nullcontext():
                data.ctrl[actuator_ids] = _interpolate_rows(
                    waypoint_times, joint_path, command_time
                )
                mujoco.mj_step(model, data)
                record_sample(time_value)
            if viewer is not None:
                viewer.sync()
                remaining = (
                    simulation_wall_start + time_value - time.perf_counter()
                )
                if remaining > 0:
                    time.sleep(remaining)
        replay_index += 1

    if args.viewer:
        import mujoco.viewer

        pending_actions = queue.Queue()

        def key_callback(key):
            if key in (ord("n"), ord("N")):
                pending_actions.put("new_pose")
            elif key in (ord("r"), ord("R")):
                pending_actions.put("replay")

        viewer_context = mujoco.viewer.launch_passive(
            model, data, key_callback=key_callback
        )
        print(
            "MuJoCo Viewer opened. Press N to randomize the core and replan; "
            "press R to replay the current plan; close the window to exit."
        )
    else:
        pending_actions = None
        viewer_context = nullcontext(None)

    replay_history.append(
        {
            "replay_index": 1,
            "action": "initial_plan",
            "core_center_m": plan["core_center"].tolist(),
            "theta_deg": float(np.rad2deg(plan["core"]["theta"])),
            "phi_deg": float(np.rad2deg(plan["core"]["phi"])),
            "alpha_deg": float(np.rad2deg(plan["core"]["alpha"])),
        }
    )
    with viewer_context as viewer:
        run_segment(viewer)
        while viewer is not None and viewer.is_running():
            try:
                action = pending_actions.get(timeout=0.03)
            except queue.Empty:
                viewer.sync()
                continue
            if action == "new_pose":
                new_plan = replan_callback(rng)
                core = new_plan["core"]
                with viewer.lock():
                    model.geom_pos[core_geom_id] = core["center"]
                    xyzw = Rotation.from_matrix(core["rotation"]).as_quat()
                    model.geom_quat[core_geom_id] = xyzw[[3, 0, 1, 2]]
                    mujoco.mj_forward(model, data)
                plan.clear()
                plan.update(new_plan)
                core_position = plan["core_center"]
                replay_history.append(
                    {
                        "replay_index": replay_index + 1,
                        "action": "new_pose_and_replan",
                        "core_center_m": core_position.tolist(),
                        "theta_deg": float(np.rad2deg(plan["core"]["theta"])),
                        "phi_deg": float(np.rad2deg(plan["core"]["phi"])),
                        "alpha_deg": float(np.rad2deg(plan["core"]["alpha"])),
                    }
                )
                print(
                    "Replanned new core pose: "
                    f"center={np.array2string(core_position, precision=3)}, "
                    f"theta/phi/alpha="
                    f"{replay_history[-1]['theta_deg']:.1f}/"
                    f"{replay_history[-1]['phi_deg']:.1f}/"
                    f"{replay_history[-1]['alpha_deg']:.1f} deg"
                )
                run_segment(viewer)
            elif action == "replay":
                replay_history.append(
                    {"replay_index": replay_index + 1, "action": "replay_current_plan"}
                )
                print("Replaying the current plan.")
                run_segment(viewer)
            viewer.sync()

    fieldnames = list(sample_rows[0])
    with output_path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(sample_rows)

    sample_count = len(sample_rows)
    return {
        "simulation_duration_s": args.duration,
        "simulation_timestep_s": args.timestep,
        "simulation_samples": sample_count,
        "replay_count": replay_index,
        "replay_history": replay_history,
        "controller": {
            "type": "MuJoCo position actuators",
            "kp": args.kp,
            "kv": args.kv,
            "max_torque_nm": args.max_torque,
            "trajectory_duration_s": args.duration,
        },
        "initial_fk_consistency": {
            "position_error_m": start_position_error,
            "tcp_x_error_deg": start_axis_error,
        },
        "tracking": {
            "position_error_rms_m": float(np.sqrt(position_squared_error / sample_count)),
            "position_error_max_m": max_position_error,
            "tcp_x_error_rms_deg": float(np.sqrt(axis_squared_error / sample_count)),
            "tcp_x_error_max_deg": max_axis_error,
            "joint_error_rms_rad": float(np.sqrt(joint_squared_error / sample_count)),
            "joint_error_max_rad": max_joint_error,
            "actuator_force_max_nm": max_force,
            "torque_saturated_samples": saturated_steps,
        },
        "contacts": {
            "total_contacts_over_all_steps": total_contact_count,
            "steps_with_contact": contact_steps,
            "core_contacts_over_all_steps": core_contact_count,
            "steps_with_core_contact": core_contact_steps,
            "max_normal_force_n": max_contact_force,
            "max_penetration_m": max_penetration,
        },
        "time_series_csv": str(output_path),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--urdf",
        type=Path,
        default=CANONICAL_URDF_PATH,
    )
    parser.add_argument("--ee-link", default="gripper")
    parser.add_argument("--seed", type=int, default=20261019)
    parser.add_argument("--theta-deg", type=float)
    parser.add_argument("--phi-deg", type=float)
    parser.add_argument("--alpha-deg", type=float)
    parser.add_argument("--radius", type=float, default=0.1)
    parser.add_argument("--steps", type=int, default=8)
    parser.add_argument("--sweep-deg", type=float, default=90.0)
    parser.add_argument("--turn-q-sweep-deg", type=float, default=60.0)
    parser.add_argument("--hover-distance", type=float, default=0.20)
    parser.add_argument("--slide-distance", type=float, default=0.10)
    parser.add_argument("--position-weight", type=float, default=1.0)
    parser.add_argument("--axis-weight", type=float, default=1.55)
    parser.add_argument("--smoothness-weight", type=float, default=0.0)
    parser.add_argument("--joint-limit-weight", type=float, default=0.0)
    parser.add_argument("--manipulability-weight", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--timestep", type=float, default=0.002)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run without opening a MuJoCo Viewer window",
    )
    parser.add_argument("--kp", type=float, default=100.0)
    parser.add_argument("--kv", type=float, default=20.0)
    parser.add_argument("--max-torque", type=float, default=40.0)
    parser.add_argument(
        "--core-dimensions",
        type=float,
        nargs=3,
        default=(0.14, 0.06, 0.06),
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).with_name("mujoco_type_iii_replay.csv"),
    )
    args = parser.parse_args()
    args.viewer = not args.headless
    explicit_angles = (args.theta_deg, args.phi_deg, args.alpha_deg)
    if any(value is not None for value in explicit_angles) and not all(
        value is not None for value in explicit_angles
    ):
        parser.error("--theta-deg, --phi-deg, and --alpha-deg must be provided together")
    positive_settings = (
        args.radius,
        args.sweep_deg,
        args.turn_q_sweep_deg,
        args.hover_distance,
        args.slide_distance,
        args.duration,
        args.timestep,
    )
    if (
        not 3 <= args.steps <= 20
        or any(not np.isfinite(value) or value <= 0 for value in positive_settings)
    ):
        parser.error(
            "steps must be 3..20; path lengths, sweeps, duration, and timestep "
            "must be positive"
        )
    if not np.isclose(args.duration / args.timestep, round(args.duration / args.timestep)):
        parser.error("duration must be an integer multiple of timestep")
    if (
        not np.isfinite(args.kp)
        or not np.isfinite(args.kv)
        or not np.isfinite(args.max_torque)
        or args.kp <= 0
        or args.kv < 0
        or args.max_torque <= 0
    ):
        parser.error("kp and max-torque must be positive and kv must be non-negative")
    if any(value <= 0 or not np.isfinite(value) for value in args.core_dimensions):
        parser.error("core dimensions must be finite and positive")
    weights = (
        args.position_weight,
        args.axis_weight,
        args.smoothness_weight,
        args.joint_limit_weight,
        args.manipulability_weight,
    )
    if any(not np.isfinite(weight) or not 0 <= weight <= 10 for weight in weights):
        parser.error("normalized weights must be finite and between 0 and 10")
    if not args.urdf.is_file():
        parser.error(f"URDF file does not exist: {args.urdf}")
    if (
        not args.headless
        and sys.platform.startswith("linux")
        and not os.environ.get("DISPLAY")
        and not os.environ.get("WAYLAND_DISPLAY")
    ):
        parser.error(
            "MuJoCo Viewer needs a graphical display (DISPLAY or WAYLAND_DISPLAY); "
            "run from a desktop session or pass --headless"
        )

    import mujoco

    import end2end_plan_demo as demo

    demo._patch_numpy_asarray_copy_kwarg()
    robot, _, pyroki_snippets = demo._load_robot(args.urdf.resolve())
    import jax.numpy as jnp
    import jaxlie

    from III_type_task_planner import CostWeights, TypeIIIPlanner, make_circular_task_path

    joint_names = list(robot.joints.actuated_names)
    lower = np.asarray(robot.joints.lower_limits, dtype=np.float64)
    upper = np.asarray(robot.joints.upper_limits, dtype=np.float64)
    link_names = list(robot.links.names)
    if args.ee_link not in link_names:
        parser.error(f"end-effector link '{args.ee_link}' is not in the URDF")
    link_index = link_names.index(args.ee_link)

    start_cfg = np.zeros(len(joint_names), dtype=np.float64)
    for index, name in enumerate(joint_names):
        if name == "joint2":
            start_cfg[index] = 1.9
        elif name == "joint3":
            start_cfg[index] = 0.5
        elif name == "joint5":
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
    resolved_position, resolved_wxyz = demo._fk_pose(robot, link_index, start_cfg)
    if (
        np.linalg.norm(resolved_position - start_position) > 0.005
        or demo._quaternion_error_rad(resolved_wxyz, start_wxyz) > np.deg2rad(3.0)
    ):
        raise RuntimeError("Could not resolve the planner's default start pose through IK")

    rng = np.random.default_rng(args.seed)
    core = demo._sample_random_core(rng)
    if all(value is not None for value in explicit_angles):
        theta_deg, phi_deg, alpha_deg = explicit_angles
        if not (-90 <= theta_deg <= 90 and 0 <= phi_deg <= 90 and -45 <= alpha_deg <= 45):
            parser.error("core angles must be within theta [-90,90], phi [0,90], alpha [-45,45]")
        core = demo._core_from_parameters(
            core["center"],
            np.deg2rad(theta_deg),
            np.deg2rad(phi_deg),
            np.deg2rad(alpha_deg),
        )

    core_center = np.asarray(core["center"], dtype=np.float64)
    q_axis = np.asarray(core["q_axis"], dtype=np.float64)
    p_axis = np.asarray(core["p_axis"], dtype=np.float64)
    up_axis = np.asarray(core["up_axis"], dtype=np.float64)
    hover_position = core_center + args.hover_distance * q_axis
    insert_position = hover_position - args.hover_distance * q_axis
    slide_position = insert_position + args.slide_distance * up_axis
    start_rotation = jaxlie.SO3(jnp.asarray(start_wxyz))
    start_tcp_x = np.asarray(start_rotation.apply(jnp.array([1.0, 0.0, 0.0])))
    hover_tcp_x, _, _ = demo._hover_tcp_x_frame(q_axis)
    steps = args.steps
    approach_positions, approach_axes = demo._make_linear_task_segment(
        start_position, hover_position, start_tcp_x, hover_tcp_x, steps
    )
    insert_positions, insert_axes = demo._make_linear_task_segment(
        hover_position, insert_position, hover_tcp_x, hover_tcp_x, steps
    )
    slide_positions, slide_axes = demo._make_linear_task_segment(
        insert_position, slide_position, hover_tcp_x, hover_tcp_x, steps
    )
    radius_vector_p = args.radius * q_axis
    p_direction = (
        1.0 if np.dot(np.cross(p_axis, radius_vector_p), up_axis) >= 0.0 else -1.0
    )
    turn_p_axis = p_direction * p_axis
    _, turn_p_positions, turn_p_axes = make_circular_task_path(
        start_position=slide_position,
        start_axis=-q_axis,
        rotation_axis=turn_p_axis,
        start_radial=radius_vector_p,
        sweep_rad=np.deg2rad(args.sweep_deg),
        samples=steps,
        orientation_axis=turn_p_axis,
        orientation_sweep_rad=np.deg2rad(args.sweep_deg),
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
    _, turn_q_positions, turn_q_axes = make_circular_task_path(
        start_position=turn_p_positions[-1],
        start_axis=turn_p_axes[-1],
        rotation_axis=turn_p_end_z,
        start_radial=args.radius * turn_q_radial,
        sweep_rad=np.deg2rad(args.turn_q_sweep_deg),
        samples=steps,
        orientation_axis=turn_p_end_z,
        orientation_sweep_rad=np.deg2rad(args.turn_q_sweep_deg),
    )
    target_positions, target_axes, stage_ranges = demo._combine_task_segments(
        [
            ("Approach", approach_positions, approach_axes),
            ("Insert", insert_positions, insert_axes),
            ("Slide", slide_positions, slide_axes),
            ("Turn P", turn_p_positions, turn_p_axes),
            ("Turn Q", turn_q_positions, turn_q_axes),
        ]
    )
    position_weights = np.ones(len(target_positions), dtype=np.float64)
    axis_weights = np.ones(len(target_axes), dtype=np.float64)
    hover_index = stage_ranges["Approach"][1]
    position_weights[hover_index] = 10.0
    axis_weights[hover_index] = 10.0

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
    planner.warm_up(start_cfg, len(target_positions))
    result = planner.plan(
        start_cfg,
        target_positions,
        target_axes,
        position_weights=position_weights,
        axis_weights=axis_weights,
    )
    if not np.all(np.isfinite(result.joint_positions)):
        raise RuntimeError("Planner returned non-finite joint values")
    plan = {
        "core": core,
        "positions": target_positions,
        "axes": target_axes,
        "result": result,
        "stage_ranges": stage_ranges,
        "core_center": core_center,
    }

    def replan_core(candidate_rng):
        candidate_core = demo._sample_random_core(candidate_rng)
        return _plan_core(
            demo,
            make_circular_task_path,
            planner,
            start_cfg,
            start_position,
            start_tcp_x,
            candidate_core,
            args,
        )

    args.output = args.output.expanduser().resolve()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    summary_path = args.output.with_suffix(".summary.json")
    temporary_directory, model = _build_model(
        mujoco, args.urdf.resolve(), core, args, joint_names, lower, upper
    )
    try:
        summary = _replay(
            mujoco,
            model,
            joint_names,
            plan,
            args,
            args.output,
            replan_core,
            rng,
        )
    finally:
        if temporary_directory is not None:
            temporary_directory.cleanup()

    summary.update(
        {
            "seed": args.seed,
            "core": {
                "center_m": plan["core_center"].tolist(),
                "theta_deg": float(np.rad2deg(plan["core"]["theta"])),
                "phi_deg": float(np.rad2deg(plan["core"]["phi"])),
                "alpha_deg": float(np.rad2deg(plan["core"]["alpha"])),
                "dimensions_m": list(args.core_dimensions),
            },
            "trajectory": {
                "sample_count": len(plan["positions"]),
                "stage_count": len(plan["stage_ranges"]),
                "planner_success": bool(plan["result"].optimizer_success),
                "planner_iterations": plan["result"].optimizer_iterations,
                "planner_objective": plan["result"].objective_value,
            },
            "weights": {
                "position": args.position_weight,
                "axis_alignment": args.axis_weight,
                "smoothness": args.smoothness_weight,
                "joint_limit": args.joint_limit_weight,
                "manipulability": args.manipulability_weight,
                "hover_endpoint_multiplier": 10.0,
            },
        }
    )
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"Trajectory samples: {len(plan['positions'])}")
    print(
        "Core pose: "
        f"center={np.array2string(plan['core_center'], precision=3)}, "
        f"theta/phi/alpha={summary['core']['theta_deg']:.1f}/"
        f"{summary['core']['phi_deg']:.1f}/{summary['core']['alpha_deg']:.1f} deg"
    )
    print(
        "Planner task-space errors: "
        f"position max={np.max(plan['result'].position_errors) * 1000.0:.2f} mm, "
        f"TCP-X max={np.rad2deg(np.max(plan['result'].axis_errors_rad)):.2f} deg"
    )
    print(
        "MuJoCo tracking: "
        f"position RMS={summary['tracking']['position_error_rms_m'] * 1000.0:.2f} mm, "
        f"max={summary['tracking']['position_error_max_m'] * 1000.0:.2f} mm; "
        f"TCP-X RMS={summary['tracking']['tcp_x_error_rms_deg']:.2f} deg, "
        f"max={summary['tracking']['tcp_x_error_max_deg']:.2f} deg"
    )
    print(
        "Contacts: "
        f"{summary['contacts']['total_contacts_over_all_steps']} total, "
        f"{summary['contacts']['steps_with_core_contact']} steps with core contact; "
        f"max penetration={summary['contacts']['max_penetration_m'] * 1000.0:.2f} mm"
    )
    print(f"Time series: {args.output}")
    print(f"Summary: {summary_path}")


if __name__ == "__main__":
    main()
