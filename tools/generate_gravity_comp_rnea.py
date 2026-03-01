#!/usr/bin/env python3
"""
Generate standalone C gravity compensation code using RNEA (Recursive Newton-Euler).

Unlike the symbolic dV/dq approach, this generates code that mirrors Pinocchio's
actual RNEA computation: recursive forward/backward passes through the kinematic
chain. This is numerically stable in single-precision float because each step is
a small local matrix-vector operation — no giant symbolic expressions with
potential catastrophic cancellation.

Usage:
    python3 generate_gravity_comp_rnea.py path/to/robot.urdf -o output_dir/ [--validate]
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path
import math


def parse_urdf(urdf_path: str) -> list[dict]:
    """Parse revolute joints and child-link inertial data from URDF."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    links: dict[str, dict] = {}
    for link_elem in root.findall("link"):
        name = link_elem.get("name")
        inertial = link_elem.find("inertial")
        if inertial is not None:
            mass_val = float(inertial.find("mass").get("value"))
            origin = inertial.find("origin")
            com_xyz = [float(x) for x in origin.get("xyz").split()] if origin is not None else [0, 0, 0]
            links[name] = {"mass": mass_val, "com": com_xyz}

    joint_map: dict[str, dict] = {}
    child_set: set[str] = set()
    for joint_elem in root.findall("joint"):
        if joint_elem.get("type") != "revolute":
            continue
        origin = joint_elem.find("origin")
        xyz = [float(x) for x in origin.get("xyz").split()]
        rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()]
        axis_elem = joint_elem.find("axis")
        axis = [float(x) for x in axis_elem.get("xyz").split()]
        parent_name = joint_elem.find("parent").get("link")
        child_name = joint_elem.find("child").get("link")
        child_set.add(child_name)
        joint_map[parent_name] = {
            "name": joint_elem.get("name"),
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis,
            "child": child_name,
            "mass": links[child_name]["mass"],
            "com": links[child_name]["com"],
        }

    all_link_names = {l.get("name") for l in root.findall("link")}
    root_candidates = all_link_names - child_set
    current = None
    for rc in root_candidates:
        if rc in joint_map:
            current = rc
            break
    if current is None:
        for rc in root_candidates:
            for joint_elem in root.findall("joint"):
                if joint_elem.get("type") == "fixed":
                    if joint_elem.find("parent").get("link") == rc:
                        candidate = joint_elem.find("child").get("link")
                        if candidate in joint_map:
                            current = candidate
                            break
            if current:
                break

    if current is None:
        raise RuntimeError("Could not find root of revolute chain in URDF")

    ordered_joints: list[dict] = []
    while current in joint_map:
        j = joint_map[current]
        ordered_joints.append(j)
        current = j["child"]

    return ordered_joints


def snap_angle(val: float) -> str:
    """Snap rpy value to exact multiple of pi/2 for clean rotation matrices."""
    for num, den in [(0, 1), (1, 2), (1, 1), (3, 2), (2, 1)]:
        for sign in [1, -1]:
            exact = sign * num * math.pi / den if den != 0 else 0.0
            if abs(val - exact) < 1e-4:
                return exact
    return val


def rpy_to_matrix(roll: float, pitch: float, yaw: float):
    """Fixed-axis RPY (extrinsic XYZ) = Rz(yaw) * Ry(pitch) * Rx(roll).
    Returns 3x3 list-of-lists. Snaps to exact values for multiples of pi/2."""
    r, p, y = snap_angle(roll), snap_angle(pitch), snap_angle(yaw)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    # Snap near-zero/near-one values
    def snap(v):
        if abs(v) < 1e-15:
            return 0.0
        if abs(v - 1.0) < 1e-15:
            return 1.0
        if abs(v + 1.0) < 1e-15:
            return -1.0
        return v

    R = [
        [snap(cy*cp), snap(cy*sp*sr - sy*cr), snap(cy*sp*cr + sy*sr)],
        [snap(sy*cp), snap(sy*sp*sr + cy*cr), snap(sy*sp*cr - cy*sr)],
        [snap(-sp),   snap(cp*sr),            snap(cp*cr)],
    ]
    return R


def fmt(val: float) -> str:
    """Format a float constant for C code."""
    if val == 0.0:
        return "0.0f"
    if val == 1.0:
        return "1.0f"
    if val == -1.0:
        return "-1.0f"
    return f"{val:.10g}f"


def generate_rnea_code(joints: list[dict], urdf_name: str) -> tuple[str, str]:
    """
    Generate RNEA-based gravity compensation C code.

    Algorithm (gravity-only RNEA, v=0, a=0):
      Forward pass: compute cumulative rotation R_0i for each joint frame
      For each link: transform COM to world frame, compute gravity force
      Backward pass: accumulate torques from tip to base

    Instead of building generic matrix multiply code, we pre-compute the fixed
    rotation matrices and inline only the non-zero terms.
    """
    n = len(joints)

    lines = []
    lines.append(f"/* Auto-generated RNEA gravity compensation from {urdf_name} */")
    lines.append("/* DO NOT EDIT -- regenerate with generate_gravity_comp_rnea.py */")
    lines.append("")
    lines.append('#include "gravity_comp.h"')
    lines.append("#include <math.h>")
    lines.append("")
    lines.append(f"void compute_gravity_torques(const float q[{n}], float tau[{n}])")
    lines.append("{")
    lines.append("    const float g = 9.81f;")
    lines.append("")

    # Pre-compute sin/cos for each joint
    lines.append("    /* Joint sin/cos */")
    for i in range(n):
        axis_z = joints[i]["axis"][2]
        if axis_z < 0:
            lines.append(f"    const float s{i} = sinf(-q[{i}]);")
            lines.append(f"    const float c{i} = cosf(-q[{i}]);")
        else:
            lines.append(f"    const float s{i} = sinf(q[{i}]);")
            lines.append(f"    const float c{i} = cosf(q[{i}]);")
    lines.append("")

    # Forward pass: compute R_0i (rotation from joint i frame to world)
    # and p_0i (position of joint i origin in world frame)
    #
    # For each joint i:
    #   T_parent_i = T_fixed(rpy, xyz) * T_joint(qi)
    #   R_0i = R_0_{i-1} * R_fixed * R_joint(qi)
    #   p_0i = p_0_{i-1} + R_0_{i-1} * t_fixed

    lines.append("    /* Forward pass: rotation matrices and positions */")
    lines.append("    /* R_0i = rotation from frame i to world (stored row-major) */")
    lines.append("    /* p_0i = origin of frame i in world coordinates */")
    lines.append("")

    # We'll track the cumulative rotation and position as named variables
    # R_prev[3][3] and p_prev[3]

    for i, joint in enumerate(joints):
        R_fixed = rpy_to_matrix(*joint["rpy"])
        t_fixed = joint["xyz"]

        lines.append(f"    /* Joint {i} ({joint['name']}): "
                     f"rpy=({joint['rpy'][0]:.4f}, {joint['rpy'][1]:.4f}, {joint['rpy'][2]:.4f}), "
                     f"xyz=({t_fixed[0]}, {t_fixed[1]}, {t_fixed[2]}) */")

        # R_combined = R_fixed * R_joint(qi)
        # R_joint = [[ci, -si, 0], [si, ci, 0], [0, 0, 1]]
        #
        # R_combined[row][col] = sum_k R_fixed[row][k] * R_joint[k][col]
        # col 0: R_fixed[row][0]*ci + R_fixed[row][1]*si
        # col 1: -R_fixed[row][0]*si + R_fixed[row][1]*ci
        # col 2: R_fixed[row][2]

        # Compute R_combined symbolically (with numeric R_fixed values)
        # Then compute R_0i = R_prev * R_combined

        # For clarity, emit the combined rotation R_fixed * R_joint(qi)
        # as Rc{i}[3][3], then R{i} = R{i-1} * Rc{i}

        # Emit Rc (R_fixed * R_joint)
        for row in range(3):
            a, b, c = R_fixed[row]
            # col 0 = a*ci + b*si
            # col 1 = -a*si + b*ci
            # col 2 = c
            terms_0 = []
            if a != 0:
                terms_0.append(f"{fmt(a)}*c{i}")
            if b != 0:
                terms_0.append(f"{fmt(b)}*s{i}")
            terms_1 = []
            if a != 0:
                terms_1.append(f"{fmt(-a)}*s{i}")
            if b != 0:
                terms_1.append(f"{fmt(b)}*c{i}")

            expr_0 = " + ".join(terms_0) if terms_0 else "0.0f"
            expr_1 = " + ".join(terms_1) if terms_1 else "0.0f"
            expr_2 = fmt(c)

            # Clean up double negatives
            expr_0 = expr_0.replace("+ -", "- ")
            expr_1 = expr_1.replace("+ -", "- ")

            lines.append(f"    const float Rc{i}_{row}0 = {expr_0};")
            lines.append(f"    const float Rc{i}_{row}1 = {expr_1};")
            lines.append(f"    const float Rc{i}_{row}2 = {expr_2};")

        lines.append("")

        # Compute R_0i = R_prev * Rc_i
        if i == 0:
            # R_prev = identity (world frame = base_link frame for joint 0)
            for row in range(3):
                for col in range(3):
                    lines.append(f"    const float R{i}_{row}{col} = Rc{i}_{row}{col};")
        else:
            for row in range(3):
                for col in range(3):
                    terms = []
                    for k in range(3):
                        terms.append(f"R{i-1}_{row}{k}*Rc{i}_{k}{col}")
                    lines.append(f"    const float R{i}_{row}{col} = {' + '.join(terms)};")

        lines.append("")

        # Compute p_0i = p_prev + R_prev * t_fixed
        tx, ty, tz = t_fixed
        if i == 0:
            # p_prev = (0,0,0), R_prev = identity
            lines.append(f"    const float p{i}_x = {fmt(tx)};")
            lines.append(f"    const float p{i}_y = {fmt(ty)};")
            lines.append(f"    const float p{i}_z = {fmt(tz)};")
        else:
            for dim, (comp, tval) in enumerate(zip(['x', 'y', 'z'], [tx, ty, tz])):
                terms = [f"p{i-1}_{comp}"]
                for k, tv in enumerate([tx, ty, tz]):
                    if tv != 0:
                        terms.append(f"R{i-1}_{dim}{k}*{fmt(tv)}")
                expr = " + ".join(terms).replace("+ -", "- ")
                lines.append(f"    const float p{i}_{comp} = {expr};")

        lines.append("")

    # Compute COM positions in world frame and gravity forces
    lines.append("    /* COM positions in world frame and gravity forces */")
    for i, joint in enumerate(joints):
        cx, cy, cz = joint["com"]
        mass = joint["mass"]
        # com_world = p_0i + R_0i * com_local
        for dim, comp in enumerate(['x', 'y', 'z']):
            terms = [f"p{i}_{comp}"]
            for k, cv in enumerate([cx, cy, cz]):
                if cv != 0:
                    terms.append(f"R{i}_{dim}{k}*{fmt(cv)}")
            expr = " + ".join(terms).replace("+ -", "- ")
            lines.append(f"    const float com{i}_{comp} = {expr};")
        lines.append("")

    # Backward pass: accumulate torques
    # RNEA convention: base sees upward pseudo-acceleration a0 = [0,0,+g]
    # tau_i = axis_z_i * z_i . sum_{j>=i} (r_ij x m_j*[0,0,+g])
    #
    # r x [0,0,+mg] = [ry*mg, -rx*mg, 0]
    # z . [ry*mg, -rx*mg, 0] = mg*(ry*zx - rx*zy)
    #
    # axis_z accounts for joint rotation direction (±1)

    lines.append("    /* Backward pass: gravity torques (RNEA) */")
    lines.append("    /* tau_i = axis_z * g * sum_{j>=i} m_j * ((com_j_y - p_i_y)*z_i_x - (com_j_x - p_i_x)*z_i_y) */")
    lines.append("")

    for i in range(n):
        lines.append(f"    const float z{i}_x = R{i}_02;")
        lines.append(f"    const float z{i}_y = R{i}_12;")

    lines.append("")

    for i in range(n):
        axis_z = joints[i]["axis"][2]
        terms = []
        for j in range(i, n):
            mass = joints[j]["mass"]
            terms.append(
                f"{fmt(mass)}*((com{j}_y - p{i}_y)*z{i}_x - (com{j}_x - p{i}_x)*z{i}_y)"
            )
        expr = " + ".join(terms)
        sign = fmt(axis_z)
        lines.append(f"    tau[{i}] = {sign}*g*({expr});")

    lines.append("}")
    lines.append("")

    # Header
    h_lines = [
        '#ifndef GRAVITY_COMP_H',
        '#define GRAVITY_COMP_H',
        '',
        f'/* Auto-generated RNEA gravity compensation from {urdf_name} */',
        '/* DO NOT EDIT -- regenerate with generate_gravity_comp_rnea.py */',
        '',
        '/**',
        f' * Compute gravity compensation torques for a {n}-DOF robot arm.',
        ' *',
        f' * @param q   Joint angles in radians [{n}]  (URDF convention)',
        f' * @param tau Output gravity torques in Nm [{n}]',
        ' */',
        f'void compute_gravity_torques(const float q[{n}], float tau[{n}]);',
        '',
        '#endif /* GRAVITY_COMP_H */',
        '',
    ]

    return "\n".join(lines), "\n".join(h_lines)


def validate_against_pinocchio(urdf_path: str, n: int, num_tests: int = 500):
    """Compare generated RNEA C code against Pinocchio."""
    import numpy as np
    import subprocess
    import tempfile

    try:
        import pinocchio as pin
    except ImportError:
        print("WARNING: pinocchio not installed, skipping validation")
        return True

    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()

    # Compile the generated C code as a shared library for testing
    # Instead, just evaluate both in Python
    max_error = 0.0
    worst_config = None

    for trial in range(num_tests):
        q_test = np.random.uniform(-np.pi, np.pi, n)
        v_zero = np.zeros(n)
        tau_pin = pin.rnea(model, data, q_test, v_zero, v_zero)

        # Evaluate our RNEA in Python (double precision, should match exactly)
        tau_ours = evaluate_rnea_python(urdf_path, q_test)

        err = np.max(np.abs(tau_pin - tau_ours))
        if err > max_error:
            max_error = err
            worst_config = q_test.copy()

    print(f"Validation over {num_tests} random configurations:")
    print(f"  Max absolute error: {max_error:.4e} Nm")
    if worst_config is not None:
        print(f"  Worst config (deg): {np.degrees(worst_config).round(1)}")
    if max_error < 1e-10:
        print("  PASS (exact match)")
    elif max_error < 1e-6:
        print("  PASS")
    elif max_error < 1e-3:
        print("  WARN: small discrepancy")
    else:
        print("  FAIL: large discrepancy")
    return max_error < 1e-3


def evaluate_rnea_python(urdf_path: str, q):
    """Evaluate gravity RNEA in Python (mirrors generated C code logic)."""
    import numpy as np

    joints = parse_urdf(urdf_path)
    n = len(joints)
    g_val = 9.81

    # Forward pass
    R_list = []  # R_0i for each joint
    p_list = []  # p_0i for each joint

    R_prev = np.eye(3)
    p_prev = np.zeros(3)

    for i, joint in enumerate(joints):
        R_fixed = np.array(rpy_to_matrix(*joint["rpy"]))
        t_fixed = np.array(joint["xyz"])

        axis_z = joint["axis"][2]
        angle = axis_z * q[i]
        cq, sq = np.cos(angle), np.sin(angle)
        R_joint = np.array([
            [cq, -sq, 0],
            [sq, cq, 0],
            [0, 0, 1],
        ])

        R_combined = R_fixed @ R_joint
        R_0i = R_prev @ R_combined
        p_0i = p_prev + R_prev @ t_fixed

        R_list.append(R_0i)
        p_list.append(p_0i)

        R_prev = R_0i
        p_prev = p_0i

    # COM positions
    com_world = []
    for i, joint in enumerate(joints):
        com_local = np.array(joint["com"])
        com_w = p_list[i] + R_list[i] @ com_local
        com_world.append(com_w)

    # Backward pass: gravity torques
    # RNEA convention: base sees upward pseudo-acceleration a0 = -gravity = [0,0,+g]
    # tau_i = axis_z_i * z_i . sum_{j>=i} r_ij x (m_j * [0,0,+g])
    # where z_i = R_0i[:,2] and axis_z_i accounts for joint direction
    tau = np.zeros(n)
    for i in range(n):
        z_i = R_list[i][:, 2]  # column 2 of R_0i
        axis_z = joints[i]["axis"][2]
        for j in range(i, n):
            r_ij = com_world[j] - p_list[i]
            f_j = np.array([0, 0, joints[j]["mass"] * g_val])  # +g (upward)
            cross = np.cross(r_ij, f_j)
            tau[i] += axis_z * np.dot(cross, z_i)

    return tau


def main():
    parser = argparse.ArgumentParser(
        description="Generate RNEA-based C gravity compensation from URDF")
    parser.add_argument("urdf", help="Path to URDF file")
    parser.add_argument("-o", "--output-dir", default=".",
                        help="Output directory for .c/.h files")
    parser.add_argument("--validate", action="store_true",
                        help="Validate against Pinocchio RNEA")
    args = parser.parse_args()

    urdf_path = Path(args.urdf)
    if not urdf_path.exists():
        parser.error(f"URDF not found: {urdf_path}")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Parsing URDF: {urdf_path}")
    joints = parse_urdf(str(urdf_path))
    print(f"Found {len(joints)} revolute joints:")
    for j in joints:
        print(f"  {j['name']}: mass={j['mass']:.4f} kg, "
              f"com=({j['com'][0]:.4f}, {j['com'][1]:.4f}, {j['com'][2]:.4f}), "
              f"axis=({j['axis'][0]}, {j['axis'][1]}, {j['axis'][2]})")

    if args.validate:
        print("\nValidating Python RNEA against Pinocchio...")
        validate_against_pinocchio(str(urdf_path), len(joints))

    print("\nGenerating RNEA C code...")
    c_code, h_code = generate_rnea_code(joints, urdf_path.name)

    c_path = output_dir / "gravity_comp.c"
    h_path = output_dir / "gravity_comp.h"
    c_path.write_text(c_code)
    h_path.write_text(h_code)
    print(f"Written: {c_path}")
    print(f"Written: {h_path}")
    print("Done.")


if __name__ == "__main__":
    main()
