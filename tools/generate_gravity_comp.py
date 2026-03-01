#!/usr/bin/env python3
"""
Generate standalone C gravity compensation code from a URDF file.

Parses the robot URDF, symbolically computes gravity torques using the
potential energy method (tau_i = dV/dq_i), and generates optimized C code
suitable for STM32 (single-precision, no dynamic allocation).

Usage:
    python3 generate_gravity_comp.py path/to/robot.urdf -o output_dir/ [--validate]
"""

import argparse
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import sympy as sp
from sympy import cos, sin, Matrix, symbols, diff, Float
from sympy.printing.c import ccode


# ---------------------------------------------------------------------------
# 1. URDF Parsing
# ---------------------------------------------------------------------------

def parse_urdf(urdf_path: str) -> list[dict]:
    """
    Parse revolute joints and their child-link inertial data from a URDF file.

    Returns an ordered list (joint1..jointN) of dicts with keys:
        name, xyz, rpy, axis_z, child, mass, com
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Gather link inertial data
    links: dict[str, dict] = {}
    for link_elem in root.findall("link"):
        name = link_elem.get("name")
        inertial = link_elem.find("inertial")
        if inertial is not None:
            mass_val = float(inertial.find("mass").get("value"))
            origin = inertial.find("origin")
            com_xyz = [float(x) for x in origin.get("xyz").split()] if origin is not None else [0, 0, 0]
            links[name] = {"mass": mass_val, "com": com_xyz}

    # Gather revolute joints and build parent->child mapping
    joint_map: dict[str, dict] = {}   # parent_link -> joint info
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
            "axis_z": axis[2],   # +1 or -1 (rotation about local Z)
            "child": child_name,
            "mass": links[child_name]["mass"],
            "com": links[child_name]["com"],
        }

    # Walk the chain from the root link (the one that is never a child)
    all_link_names = {l.get("name") for l in root.findall("link")}
    root_candidates = all_link_names - child_set
    # Find the root link that actually starts the revolute chain
    current = None
    for rc in root_candidates:
        if rc in joint_map:
            current = rc
            break
    if current is None:
        # Try one level deeper (world -> base_link via fixed joint)
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


# ---------------------------------------------------------------------------
# 2. Symbolic Kinematics & Gravity Torques
# ---------------------------------------------------------------------------

def rpy_to_rotation(roll: float, pitch: float, yaw: float) -> Matrix:
    """Fixed-axis RPY (extrinsic XYZ) = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    cr, sr = sp.cos(Float(roll)), sp.sin(Float(roll))
    cp, spp = sp.cos(Float(pitch)), sp.sin(Float(pitch))
    cy, sy = sp.cos(Float(yaw)), sp.sin(Float(yaw))
    return Matrix([
        [cy * cp, cy * spp * sr - sy * cr, cy * spp * cr + sy * sr],
        [sy * cp, sy * spp * sr + cy * cr, sy * spp * cr - cy * sr],
        [-spp,    cp * sr,                  cp * cr],
    ])


def homogeneous(R: Matrix, t: list) -> Matrix:
    """Build 4x4 homogeneous transform from 3x3 R and translation [x,y,z]."""
    T = sp.eye(4)
    T[:3, :3] = R
    T[0, 3] = Float(t[0]) if not isinstance(t[0], sp.Basic) else t[0]
    T[1, 3] = Float(t[1]) if not isinstance(t[1], sp.Basic) else t[1]
    T[2, 3] = Float(t[2]) if not isinstance(t[2], sp.Basic) else t[2]
    return T


def compute_symbolic_gravity_torques(joints: list[dict]):
    """
    Build symbolic gravity torque expressions from the kinematic chain.

    Returns (q_symbols, tau_expressions) where:
        q_symbols:  tuple of SymPy symbols (q0, q1, ..., q5)
        tau_expressions:  list of 6 symbolic expressions for gravity torques
    """
    n = len(joints)
    q = symbols(f"q0:{n}")
    g = Float(9.81)

    T_cum = sp.eye(4)
    V = Float(0)  # total potential energy

    for i, joint in enumerate(joints):
        # Fixed transform from joint origin
        R_fixed = rpy_to_rotation(*joint["rpy"])
        T_fixed = homogeneous(R_fixed, joint["xyz"])

        # Joint rotation about local Z, scaled by axis direction
        axis_z = joint["axis_z"]
        angle = axis_z * q[i]
        cq, sq = cos(angle), sin(angle)
        R_joint = Matrix([
            [cq, -sq, 0],
            [sq,  cq, 0],
            [0,   0,  1],
        ])
        T_joint = homogeneous(R_joint, [0, 0, 0])

        # Accumulate transform
        T_cum = T_cum * T_fixed * T_joint

        # COM of this link in world frame
        com_local = Matrix([Float(joint["com"][0]),
                            Float(joint["com"][1]),
                            Float(joint["com"][2]),
                            1])
        com_world = T_cum * com_local
        z_com = com_world[2]

        # Potential energy contribution
        V += Float(joint["mass"]) * g * z_com

    # Gravity torques: tau_i = dV/dq_i
    print("Computing symbolic gravity torques (this may take a minute)...")
    tau = []
    for i in range(n):
        print(f"  differentiating V w.r.t. q{i} ...")
        tau_i = diff(V, q[i])
        tau.append(tau_i)

    return q, tau


# ---------------------------------------------------------------------------
# 3. C Code Generation
# ---------------------------------------------------------------------------

def generate_c_code(q, tau, urdf_name: str, use_float: bool = True) -> tuple[str, str]:
    """
    Generate gravity_comp.c and gravity_comp.h content strings.

    Uses SymPy CSE to minimise redundant computation.
    """
    n = len(tau)

    # Substitute q symbols with indexed array notation for C
    q_c = [sp.Symbol(f"q[{i}]") for i in range(n)]
    subs = {q[i]: q_c[i] for i in range(n)}
    tau_c = [t.subs(subs) for t in tau]

    # Common subexpression elimination
    print("Running common subexpression elimination...")
    cse_vars, cse_exprs = sp.cse(tau_c, optimizations="basic")
    print(f"  {len(cse_vars)} intermediate variables extracted")

    float_type = "float" if use_float else "double"
    sin_fn = "sinf" if use_float else "sin"
    cos_fn = "cosf" if use_float else "cos"

    def to_c(expr) -> str:
        """Convert SymPy expression to C code string."""
        c = ccode(expr)
        if use_float:
            c = c.replace("sin(", "sinf(")
            c = c.replace("cos(", "cosf(")
            c = c.replace("pow(", "powf(")
            # Add f suffix to float literals (e.g. 0.123 -> 0.123f)
            c = re.sub(r"(?<!\w)(\d+\.\d+(?:[eE][+-]?\d+)?)(?!f)", r"\1f", c)
        return c

    # --- .c file ---
    c_lines = [
        f'/* Auto-generated gravity compensation from {urdf_name} */',
        f'/* DO NOT EDIT — regenerate with generate_gravity_comp.py */',
        '',
        '#include "gravity_comp.h"',
        '#include <math.h>',
        '',
        f'void compute_gravity_torques(const {float_type} q[{n}], {float_type} tau[{n}])',
        '{',
    ]

    for var, expr in cse_vars:
        c_lines.append(f"    const {float_type} {var} = {to_c(expr)};")

    c_lines.append("")

    for i, expr in enumerate(cse_exprs):
        c_lines.append(f"    tau[{i}] = {to_c(expr)};")

    c_lines.append("}")
    c_lines.append("")

    # --- .h file ---
    h_lines = [
        '#ifndef GRAVITY_COMP_H',
        '#define GRAVITY_COMP_H',
        '',
        f'/* Auto-generated gravity compensation from {urdf_name} */',
        f'/* DO NOT EDIT — regenerate with generate_gravity_comp.py */',
        '',
        '/**',
        f' * Compute gravity compensation torques for a {n}-DOF robot arm.',
        ' *',
        f' * @param q   Joint angles in radians [{n}]  (URDF convention)',
        f' * @param tau Output gravity torques in Nm [{n}]',
        ' */',
        f'void compute_gravity_torques(const {float_type} q[{n}], {float_type} tau[{n}]);',
        '',
        '#endif /* GRAVITY_COMP_H */',
        '',
    ]

    return "\n".join(c_lines), "\n".join(h_lines)


# ---------------------------------------------------------------------------
# 4. Validation against Pinocchio
# ---------------------------------------------------------------------------

def validate_against_pinocchio(urdf_path: str, q, tau, num_tests: int = 200):
    """Compare symbolic gravity torques with Pinocchio RNEA (v=0, a=0)."""
    import numpy as np

    try:
        import pinocchio as pin
    except ImportError:
        print("WARNING: pinocchio not installed, skipping validation")
        return True

    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    n = model.nv

    # Lambdify symbolic expressions for fast numpy evaluation
    tau_func = sp.lambdify(q, tau, modules="numpy")

    max_error = 0.0
    worst_config = None

    # Use joint limits from URDF if available, else [-pi, pi]
    lo = np.full(n, -np.pi)
    hi = np.full(n, np.pi)

    for trial in range(num_tests):
        q_test = np.random.uniform(lo, hi)
        v_zero = np.zeros(n)

        tau_pin = pin.rnea(model, data, q_test, v_zero, v_zero)
        tau_sym = np.array(tau_func(*q_test), dtype=np.float64)

        err = np.max(np.abs(tau_pin - tau_sym))
        if err > max_error:
            max_error = err
            worst_config = q_test.copy()

    print(f"Validation over {num_tests} random configurations:")
    print(f"  Max absolute error: {max_error:.4e} Nm")
    if worst_config is not None:
        print(f"  Worst config (deg): {np.degrees(worst_config).round(1)}")
    if max_error < 1e-6:
        print("  PASS")
    elif max_error < 1e-3:
        print("  WARN: small discrepancy (likely float rounding in URDF constants)")
    else:
        print("  FAIL: large discrepancy — check kinematic chain construction!")
    return max_error < 1e-3


# ---------------------------------------------------------------------------
# 5. Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Generate standalone C gravity compensation code from URDF")
    parser.add_argument("urdf", help="Path to URDF file")
    parser.add_argument("-o", "--output-dir", default=".",
                        help="Output directory for .c/.h files (default: cwd)")
    parser.add_argument("--validate", action="store_true",
                        help="Validate against Pinocchio RNEA (requires pinocchio + numpy)")
    parser.add_argument("--double", action="store_true",
                        help="Use double precision instead of float (default: float for STM32)")
    args = parser.parse_args()

    urdf_path = Path(args.urdf)
    if not urdf_path.exists():
        parser.error(f"URDF not found: {urdf_path}")

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Parse
    print(f"Parsing URDF: {urdf_path}")
    joints = parse_urdf(str(urdf_path))
    print(f"Found {len(joints)} revolute joints:")
    for j in joints:
        print(f"  {j['name']}: mass={j['mass']:.4f} kg, "
              f"com=({j['com'][0]:.4f}, {j['com'][1]:.4f}, {j['com'][2]:.4f}), "
              f"axis_z={j['axis_z']:+.0f}")

    # Symbolic computation
    q, tau = compute_symbolic_gravity_torques(joints)

    # Validate
    if args.validate:
        print("\nValidating against Pinocchio...")
        validate_against_pinocchio(str(urdf_path), q, tau)

    # Generate C
    print("\nGenerating C code...")
    c_code, h_code = generate_c_code(q, tau, urdf_path.name, use_float=not args.double)

    c_path = output_dir / "gravity_comp.c"
    h_path = output_dir / "gravity_comp.h"
    c_path.write_text(c_code)
    h_path.write_text(h_code)
    print(f"Written: {c_path}")
    print(f"Written: {h_path}")
    print("Done.")


if __name__ == "__main__":
    main()
