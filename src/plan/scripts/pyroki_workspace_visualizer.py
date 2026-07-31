#!/usr/bin/env python3
"""Sample a 3D box, solve IK for each point, and visualize the reachable workspace.

The script reuses the same URDF loading and IK validation flow as the existing
pyroki unit test node, but instead of solving a single target pose it scans a
user-defined bounding box and renders the reachable samples as a point cloud.
"""

from __future__ import annotations

import argparse
import csv
import importlib
import logging
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any

import numpy as np
import viser

try:
    import pyroki as pk
except ModuleNotFoundError:
    pk = None

try:
    import yourdfpy
except ModuleNotFoundError:
    yourdfpy = None

try:
    from viser.extras import ViserUrdf
except ModuleNotFoundError:
    ViserUrdf = None


def _patch_numpy_asarray_copy_kwarg() -> None:
    """Provide NumPy 2.x-style copy= support for older NumPy releases."""
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


def _import_pyroki_snippets():
    """Import pyroki_snippets with optional path fallback."""
    try:
        return importlib.import_module("pyroki_snippets")
    except ModuleNotFoundError:
        candidate_paths = []

        env_path = os.environ.get("PYROKI_EXAMPLES_PATH")
        if env_path:
            candidate_paths.append(env_path)

        # Common local checkout path used in this workspace.
        candidate_paths.append("/home/hku/third_party/pyroki/examples")

        for path in candidate_paths:
            if not path or not os.path.isdir(path):
                continue
            if path not in sys.path:
                sys.path.insert(0, path)
            try:
                return importlib.import_module("pyroki_snippets")
            except ModuleNotFoundError:
                continue

        raise ModuleNotFoundError(
            "Cannot import 'pyroki_snippets'. Set PYTHONPATH or PYROKI_EXAMPLES_PATH "
            "to your pyroki examples directory (for example: /home/hku/third_party/pyroki/examples)."
        )


def _make_package_filename_handler():
    """Resolve package:// URDF resources by searching workspace install/source roots."""
    search_roots = [
        "/home/hku/robo_arm/src/arms",
        "/home/hku/robo_arm/src",
        "/home/hku/robo_arm/install",
    ]
    pkg_map = {}
    for root in search_roots:
        if not os.path.isdir(root):
            continue
        for entry in os.listdir(root):
            full = os.path.join(root, entry)
            if os.path.isdir(full) and entry not in pkg_map:
                pkg_map[entry] = full

    def handler(fname: str) -> str:
        assert fname.startswith("package://"), fname
        rest = fname[len("package://") :]
        pkg, rel = rest.split("/", 1)
        if pkg not in pkg_map:
            raise FileNotFoundError(f"package '{pkg}' not found in {search_roots}")
        return os.path.join(pkg_map[pkg], rel)

    return handler


_patch_numpy_asarray_copy_kwarg()

URDF_PATH = os.environ.get(
    "ROBO_ARM_URDF",
    "/home/hku/robo_arm/src/arms/robo_arm13/urdf/robo_arm13.urdf",
)
EE_LINK = os.environ.get("ROBO_ARM_EE_LINK", "gripper")
DEFAULT_OUTPUT_NPZ = "/home/hku/robo_arm/output/workspace_scan.npz"
DEFAULT_CFG = np.zeros((6,), dtype=np.float64)

LOGGER = logging.getLogger("pyroki_workspace_visualizer")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Sample a 3D box, solve IK, and visualize the reachable workspace."
    )
    parser.add_argument(
        "--bounds",
        nargs=6,
        type=float,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        default=[-0.6, 0.6, -0.6, 0.6, 0.1, 0.9],
        help="Voxel region bounds in meters.",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=0.03,
        help="Voxel size in meters.",
    )
    parser.add_argument(
        "--ik-pos-tol",
        type=float,
        default=0.003,
        help="IK acceptance position tolerance in meters.",
    )
    parser.add_argument(
        "--ik-ori-tol-deg",
        type=float,
        default=3.0,
        help="IK acceptance orientation tolerance in degrees.",
    )
    parser.add_argument(
        "--max-seed-jitter",
        type=float,
        default=0.08,
        help="Standard deviation for random IK seed perturbations.",
    )
    parser.add_argument(
        "--seed-count",
        type=int,
        default=6,
        help="Number of random seed perturbations tried per point.",
    )
    parser.add_argument(
        "--direction-samples",
        type=int,
        default=64,
        help="Number of Fibonacci-sphere directions per voxel.",
    )
    parser.add_argument(
        "--roll-samples",
        type=int,
        default=4,
        help="Discrete roll samples around tool Z for each direction.",
    )
    parser.add_argument(
        "--fixed-wxyz",
        nargs=4,
        type=float,
        metavar=("W", "X", "Y", "Z"),
        default=None,
        help="Fixed end-effector orientation. Defaults to the nominal pose orientation.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="Viser server port.",
    )
    parser.add_argument(
        "--output-npz",
        type=str,
        default=DEFAULT_OUTPUT_NPZ,
        help=(
            "Output .npz path for the export bundle. "
            f"Defaults to {DEFAULT_OUTPUT_NPZ} and also writes a sibling .csv file."
        ),
    )
    parser.add_argument(
        "--input-npz",
        type=str,
        default=None,
        help="Load a previously saved workspace_scan.npz bundle and reopen the slicing GUI without rerunning IK.",
    )
    parser.add_argument(
        "--point-size-scale",
        type=float,
        default=0.6,
        help="Scale factor applied to rendered point sizes.",
    )
    parser.add_argument(
        "--orientation-demo",
        action="store_true",
        help="Visualize the current orientation candidate set as pose frames scattered inside a sphere.",
    )
    parser.add_argument(
        "--demo-center",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=[0.75, -0.45, 0.55],
        help="Center of the orientation demo sphere in meters.",
    )
    parser.add_argument(
        "--demo-sphere-radius",
        type=float,
        default=0.25,
        help="Sphere radius in meters for orientation demo visualization.",
    )
    parser.add_argument(
        "--demo-frame-length",
        type=float,
        default=0.025,
        help="Axis length in meters for each orientation demo pose frame.",
    )
    parser.add_argument(
        "--demo-sphere-opacity",
        type=float,
        default=0.08,
        help="Opacity of the orientation demo sphere shell.",
    )
    return parser.parse_args()


class WorkspaceReachabilityVisualizer:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.logger = LOGGER

        self.robot = None
        self.pks = None
        self.urdf = None
        self.viser_urdf = None
        self.pk_joint_names = []
        self.cfg = np.array(DEFAULT_CFG, dtype=np.float64, copy=True)
        self.nominal_ee_pos = np.zeros((3,), dtype=np.float64)
        self.nominal_ee_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.fixed_wxyz = None
        self.orientation_candidates = []
        self._scan_thread = None
        self._loaded_scan_bundle = self._read_scan_bundle(args.input_npz) if args.input_npz else None

        self.server = viser.ViserServer(port=args.port)
        self.server.scene.add_grid("/ground", width=2, height=2)

        self.status_text = self.server.gui.add_text("Scan status", "Starting...", disabled=True)
        self.summary_text = self.server.gui.add_text("Workspace summary", "Waiting...", disabled=True)
        self.sample_count_text = self.server.gui.add_text("Sample count", "0", disabled=True)
        self.legend_html = self.server.gui.add_html(self._build_legend_html("waiting for scan"))
        self.enable_z_slice = self.server.gui.add_checkbox("Enable Z slice", False)
        initial_bounds = self._get_initial_bounds()
        zmin_default = float(initial_bounds[4])
        zmax_default = float(initial_bounds[5])
        zstep = max(0.001, self._get_initial_step())
        self.z_slice_min = self.server.gui.add_slider("Z slice min", zmin_default, zmax_default, zstep, zmin_default)
        self.z_slice_max = self.server.gui.add_slider("Z slice max", zmin_default, zmax_default, zstep, zmax_default)
        self.show_unreachable = self.server.gui.add_checkbox("Show unreachable voxels", True)

        self._reachable_handle = None
        self._unreachable_handle = None
        self._scan_voxel_arr = None
        self._scan_voxel_ri_arr = None
        self._scan_point_size = max(0.003, float(self.args.step) * float(self.args.point_size_scale))

        self.enable_z_slice.on_update(lambda _: self._refresh_workspace_render())
        self.z_slice_min.on_update(lambda _: self._refresh_workspace_render())
        self.z_slice_max.on_update(lambda _: self._refresh_workspace_render())
        self.show_unreachable.on_update(lambda _: self._refresh_workspace_render())

        self.server.scene.add_frame(
            "/workspace_reference",
            position=(0.0, 0.0, 0.0),
            wxyz=(1.0, 0.0, 0.0, 0.0),
            axes_length=0.12,
            axes_radius=0.004,
        )

        if self._loaded_scan_bundle is not None:
            self._initialize_robot_visual()
            self._load_saved_scan(self._loaded_scan_bundle)
            return

        self._initialize_robot_runtime()

        self.orientation_candidates = self._build_orientation_candidates()
        if self.args.orientation_demo:
            self._render_orientation_demo()
            return

        self.logger.info(
            "Workspace scan ready: bounds=%s, step=%.3f m, fixed_wxyz=%s, orientation_candidates=%d",
            args.bounds,
            args.step,
            self.fixed_wxyz.tolist(),
            len(self.orientation_candidates),
        )

        self._scan_thread = threading.Thread(target=self._run_scan, daemon=True)
        self._scan_thread.start()

    def _initialize_robot_runtime(self) -> None:
        if pk is None:
            raise ModuleNotFoundError("pyroki is required for scan mode but is not installed")
        if yourdfpy is None:
            raise ModuleNotFoundError("yourdfpy is required for scan mode but is not installed")
        if ViserUrdf is None:
            raise ModuleNotFoundError("viser.extras.ViserUrdf is required for scan mode but is not installed")

        self.pks = _import_pyroki_snippets()

        self.logger.info("Loading URDF: %s", URDF_PATH)
        self.urdf = yourdfpy.URDF.load(
            URDF_PATH,
            filename_handler=_make_package_filename_handler(),
            build_collision_scene_graph=True,
            load_meshes=True,
            load_collision_meshes=True,
        )
        self.robot = pk.Robot.from_urdf(self.urdf)
        self.pk_joint_names = list(self.robot.joints.actuated_names)
        self.logger.info("pyroki actuated joints: %s", self.pk_joint_names)

        self.nominal_ee_pos, self.nominal_ee_wxyz = self._fk_ee(self.cfg)
        self.fixed_wxyz = self._resolve_fixed_wxyz(self.args.fixed_wxyz)

        self._initialize_robot_visual()

    def _initialize_robot_visual(self) -> None:
        if yourdfpy is None or ViserUrdf is None:
            self.logger.warning("Skipping robot visualization because yourdfpy or ViserUrdf is unavailable")
            return

        if self.urdf is None:
            self.logger.info("Loading URDF for visualization: %s", URDF_PATH)
            self.urdf = yourdfpy.URDF.load(
                URDF_PATH,
                filename_handler=_make_package_filename_handler(),
                build_collision_scene_graph=True,
                load_meshes=True,
                load_collision_meshes=True,
            )

        if self.viser_urdf is None:
            self.viser_urdf = ViserUrdf(self.server, urdf_or_path=self.urdf)

        self.viser_urdf.update_cfg(self.cfg)

    def _get_initial_bounds(self) -> np.ndarray:
        if self._loaded_scan_bundle is not None:
            bounds = self._loaded_scan_bundle.get("bounds")
            if bounds is not None and np.asarray(bounds).shape == (6,):
                return np.asarray(bounds, dtype=np.float64)
        return np.asarray(self.args.bounds, dtype=np.float64)

    def _get_initial_step(self) -> float:
        if self._loaded_scan_bundle is not None:
            return float(self._loaded_scan_bundle.get("step", self.args.step))
        return float(self.args.step)

    def _read_scan_bundle(self, input_npz: str | None) -> dict[str, Any] | None:
        if not input_npz:
            return None

        input_path = Path(input_npz).expanduser()
        if not input_path.is_file():
            raise FileNotFoundError(f"Saved scan bundle not found: {input_path}")

        with np.load(input_path) as bundle:
            if "voxel_points" not in bundle or "ri_values" not in bundle:
                raise KeyError(
                    f"Saved scan bundle {input_path} is missing required arrays 'voxel_points' and/or 'ri_values'"
                )

            data = {
                "path": str(input_path),
                "voxel_points": np.asarray(bundle["voxel_points"], dtype=np.float64),
                "ri_values": np.asarray(bundle["ri_values"], dtype=np.float32),
                "pose_success_counts": np.asarray(bundle["pose_success_counts"], dtype=np.int32)
                if "pose_success_counts" in bundle
                else None,
                "total_pose_per_voxel": int(bundle["total_pose_per_voxel"])
                if "total_pose_per_voxel" in bundle
                else None,
                "bounds": np.asarray(bundle["bounds"], dtype=np.float64) if "bounds" in bundle else None,
                "step": float(bundle["step"]) if "step" in bundle else float(self.args.step),
                "direction_samples": int(bundle["direction_samples"]) if "direction_samples" in bundle else None,
                "roll_samples": int(bundle["roll_samples"]) if "roll_samples" in bundle else None,
                "fixed_wxyz": np.asarray(bundle["fixed_wxyz"], dtype=np.float64) if "fixed_wxyz" in bundle else None,
            }

        if data["voxel_points"].ndim != 2 or data["voxel_points"].shape[1] != 3:
            raise ValueError(f"Saved scan bundle {input_path} has invalid voxel_points shape {data['voxel_points'].shape}")
        if data["ri_values"].ndim != 1 or data["ri_values"].shape[0] != data["voxel_points"].shape[0]:
            raise ValueError(
                f"Saved scan bundle {input_path} has mismatched ri_values shape {data['ri_values'].shape} "
                f"for voxel_points shape {data['voxel_points'].shape}"
            )

        if data["bounds"] is None and data["voxel_points"].shape[0] > 0:
            mins = np.min(data["voxel_points"], axis=0) - 0.5 * data["step"]
            maxs = np.max(data["voxel_points"], axis=0) + 0.5 * data["step"]
            data["bounds"] = np.array([mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2]], dtype=np.float64)

        return data

    def _load_saved_scan(self, bundle: dict[str, Any]) -> None:
        voxel_arr = bundle["voxel_points"]
        voxel_ri_arr = bundle["ri_values"]

        self._scan_voxel_arr = voxel_arr
        self._scan_voxel_ri_arr = voxel_ri_arr
        self._scan_point_size = max(0.003, float(bundle["step"]) * float(self.args.point_size_scale))
        self.fixed_wxyz = bundle.get("fixed_wxyz")

        if voxel_arr.shape[0] > 0:
            self._refresh_workspace_render()

        reachable_count = int(np.count_nonzero(voxel_ri_arr > 0.0))
        total = int(voxel_arr.shape[0])
        success_rate = 100.0 * reachable_count / max(1, total)
        pose_success_counts = bundle.get("pose_success_counts")
        total_pose_per_voxel = bundle.get("total_pose_per_voxel")
        pose_summary = ""
        if pose_success_counts is not None and total_pose_per_voxel is not None:
            pose_success_total = int(np.sum(pose_success_counts))
            total_pose_tests = int(total_pose_per_voxel) * total
            pose_success_rate = 100.0 * pose_success_total / max(1, total_pose_tests)
            pose_summary = (
                f", pose-hit={pose_success_total}/{total_pose_tests} ({pose_success_rate:.1f}%)"
            )

        orientation_summary = ""
        if bundle.get("direction_samples") is not None and bundle.get("roll_samples") is not None:
            orientation_summary = (
                f", orientations={int(bundle['direction_samples'])}x{int(bundle['roll_samples'])}"
            )

        summary = (
            f"loaded saved scan: reachable={reachable_count}/{total} ({success_rate:.1f}%)"
            f"{pose_summary}{orientation_summary}"
        )
        self.status_text.value = "Saved scan loaded"
        self.summary_text.value = summary
        self.sample_count_text.value = str(total)
        self.logger.info("Reloaded saved workspace scan from %s", bundle["path"])
        self.logger.info(summary)

    def _resolve_fixed_wxyz(self, fixed_wxyz_arg):
        if fixed_wxyz_arg is None:
            return np.array(self.nominal_ee_wxyz, dtype=np.float64, copy=True)
        fixed_wxyz = np.array(fixed_wxyz_arg, dtype=np.float64)
        norm = float(np.linalg.norm(fixed_wxyz))
        if norm < 1e-8:
            raise ValueError("fixed quaternion norm is zero")
        return fixed_wxyz / norm

    @staticmethod
    def _quat_multiply_wxyz(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
        w1, x1, y1, z1 = [float(v) for v in lhs]
        w2, x2, y2, z2 = [float(v) for v in rhs]
        return np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _axis_angle_to_wxyz(axis: np.ndarray, angle_rad: float) -> np.ndarray:
        axis = np.array(axis, dtype=np.float64)
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1e-8:
            raise ValueError("rotation axis norm is zero")
        axis = axis / axis_norm
        half = 0.5 * float(angle_rad)
        sin_half = float(np.sin(half))
        return np.array([float(np.cos(half)), *(axis * sin_half)], dtype=np.float64)

    @staticmethod
    def _normalize_unit_quaternions(quaternions: np.ndarray) -> np.ndarray:
        quats = np.asarray(quaternions, dtype=np.float64)
        if quats.ndim != 2 or quats.shape[1] != 4:
            raise ValueError(f"quaternions must have shape (N, 4), got {quats.shape}")
        if quats.shape[0] == 0:
            return quats.reshape(0, 4)

        norms = np.linalg.norm(quats, axis=1, keepdims=True)
        safe_norms = np.where(norms < 1e-12, 1.0, norms)
        normalized = quats / safe_norms
        normalized[normalized[:, 0] < 0.0] *= -1.0
        return normalized

    @staticmethod
    def _so3_geodesic_distance(q1: np.ndarray, q2: np.ndarray) -> float:
        q1 = np.asarray(q1, dtype=np.float64)
        q2 = np.asarray(q2, dtype=np.float64)
        cosine = float(np.clip(np.abs(np.dot(q1, q2)), 0.0, 1.0))
        return float(np.arccos(cosine))

    @staticmethod
    def _pairwise_so3_distances(quaternions: np.ndarray) -> np.ndarray:
        quats = WorkspaceReachabilityVisualizer._normalize_unit_quaternions(quaternions)
        if quats.shape[0] <= 1:
            return np.zeros((quats.shape[0], quats.shape[0]), dtype=np.float64)

        dot_products = np.abs(quats @ quats.T)
        dot_products = np.clip(dot_products, 0.0, 1.0)
        distances = np.arccos(dot_products)
        np.fill_diagonal(distances, 0.0)
        return distances

    @staticmethod
    def _estimate_so3_dispersion(quaternions: np.ndarray, candidate_count: int = 20000) -> float:
        quats = WorkspaceReachabilityVisualizer._normalize_unit_quaternions(quaternions)
        if quats.shape[0] == 0:
            return float("inf")
        if quats.shape[0] == 1:
            return 0.0

        rng = np.random.default_rng(0)
        candidate_quats = rng.normal(size=(candidate_count, 4))
        candidate_quats = WorkspaceReachabilityVisualizer._normalize_unit_quaternions(candidate_quats)
        dot_products = np.abs(candidate_quats @ quats.T)
        dot_products = np.clip(dot_products, 0.0, 1.0)
        distances = np.arccos(dot_products)
        return float(np.max(np.min(distances, axis=1)))

    @staticmethod
    def _test_uniformity(quaternions: np.ndarray | None = None) -> dict[str, Any]:
        """Quantify the uniformity of SO(3) samples with dispersion and L-uniformity.

        The metric uses the natural geodesic distance on SO(3) induced by unit quaternions:
        d(q1, q2) = arccos(|q1 · q2|).

        - dispersion: the largest geodesic distance from a dense reference set to the nearest sample.
        - separation distance: half the minimum pairwise sample distance.
        - L-uniformity: dispersion / separation_distance.

        Smaller values indicate a more uniform covering of SO(3).
        """
        if quaternions is None:
            quats = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float64)
        else:
            quats = np.asarray(quaternions, dtype=np.float64)
            if quats.ndim != 2 or quats.shape[1] != 4:
                raise ValueError(f"quaternions must have shape (N, 4), got {quats.shape}")

        if quats.shape[0] == 0:
            return {
                "sample_count": 0,
                "dispersion": float("inf"),
                "separation_distance": float("inf"),
                "l_uniformity": float("inf"),
            }

        quats = WorkspaceReachabilityVisualizer._normalize_unit_quaternions(quats)
        distances = WorkspaceReachabilityVisualizer._pairwise_so3_distances(quats)

        if quats.shape[0] <= 1:
            separation_distance = float("inf")
        else:
            upper = np.triu_indices(quats.shape[0], k=1)
            min_pair_distance = float(np.min(distances[upper])) if upper[0].size else float("inf")
            separation_distance = 0.5 * min_pair_distance if np.isfinite(min_pair_distance) else float("inf")

        dispersion = WorkspaceReachabilityVisualizer._estimate_so3_dispersion(
            quats,
            candidate_count=max(10000, 50 * quats.shape[0]),
        )
        l_uniformity = float(dispersion / separation_distance) if np.isfinite(separation_distance) and separation_distance > 0.0 else float("inf")

        return {
            "sample_count": int(quats.shape[0]),
            "dispersion": dispersion,
            "separation_distance": separation_distance,
            "l_uniformity": l_uniformity,
        }

    @staticmethod
    def _fibonacci_sphere_directions(sample_count: int) -> np.ndarray:
        if sample_count <= 0:
            raise ValueError("--direction-samples must be positive")
        if sample_count == 1:
            return np.array([[0.0, 0.0, 1.0]], dtype=np.float64)

        indices = np.arange(sample_count, dtype=np.float64)
        golden_angle = np.pi * (3.0 - np.sqrt(5.0))
        z = 1.0 - 2.0 * (indices + 0.5) / sample_count
        radius = np.sqrt(np.maximum(0.0, 1.0 - z * z))
        theta = golden_angle * indices
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        return np.stack([x, y, z], axis=1).astype(np.float64)

    @staticmethod
    def _rotmat_to_wxyz(rotation: np.ndarray) -> np.ndarray:
        r = np.asarray(rotation, dtype=np.float64)
        trace = float(np.trace(r))
        if trace > 0.0:
            s = np.sqrt(trace + 1.0) * 2.0
            w = 0.25 * s
            x = (r[2, 1] - r[1, 2]) / s
            y = (r[0, 2] - r[2, 0]) / s
            z = (r[1, 0] - r[0, 1]) / s
        elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = np.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) * 2.0
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = np.sqrt(max(0.0, 1.0 + r[1, 1] - r[0, 0] - r[2, 2])) * 2.0
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = np.sqrt(max(0.0, 1.0 + r[2, 2] - r[0, 0] - r[1, 1])) * 2.0
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s
        q = np.array([w, x, y, z], dtype=np.float64)
        q_norm = float(np.linalg.norm(q))
        if q_norm < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        return q / q_norm

    @staticmethod
    def _build_alignment_rotation(direction: np.ndarray) -> np.ndarray:
        z_axis = np.asarray(direction, dtype=np.float64)
        z_norm = float(np.linalg.norm(z_axis))
        if z_norm < 1e-8:
            raise ValueError("direction norm is zero")
        z_axis = z_axis / z_norm

        ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        if abs(float(np.dot(ref, z_axis))) > 0.95:
            ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)

        x_axis = np.cross(ref, z_axis)
        x_axis /= max(float(np.linalg.norm(x_axis)), 1e-12)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= max(float(np.linalg.norm(y_axis)), 1e-12)

        return np.column_stack([x_axis, y_axis, z_axis])

    def _build_orientation_candidates(self):
        direction_samples = int(self.args.direction_samples)
        roll_samples = int(self.args.roll_samples)
        if direction_samples <= 0:
            raise ValueError("--direction-samples must be positive")
        if roll_samples <= 0:
            raise ValueError("--roll-samples must be positive")

        directions = self._fibonacci_sphere_directions(direction_samples)
        roll_angles = np.linspace(0.0, 2.0 * np.pi, roll_samples, endpoint=False, dtype=np.float64)
        candidates = []

        for direction in directions:
            align_rot = self._build_alignment_rotation(direction)
            for roll_angle in roll_angles:
                roll_rot = np.array(
                    [
                        [np.cos(roll_angle), -np.sin(roll_angle), 0.0],
                        [np.sin(roll_angle), np.cos(roll_angle), 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                    dtype=np.float64,
                )
                rotation = align_rot @ roll_rot
                candidate_wxyz = self._rotmat_to_wxyz(rotation)
                candidates.append(candidate_wxyz)

        uniformity_report = self._test_uniformity(np.asarray(candidates, dtype=np.float64))
        self.logger.info(
            "SO(3) orientation uniformity report: %s",
            {
                "direction_samples": direction_samples,
                "roll_samples": roll_samples,
                **uniformity_report,
            },
        )
        return candidates

    def _build_demo_origins(self, count: int, center: np.ndarray, radius: float) -> np.ndarray:
        if count <= 0:
            return np.zeros((0, 3), dtype=np.float64)

        directions = self._fibonacci_sphere_directions(count)
        return np.asarray(center, dtype=np.float64).reshape(1, 3) + directions * float(radius)

    def _render_orientation_demo(self) -> None:
        center = np.asarray(self.args.demo_center, dtype=np.float64)
        sphere_radius = max(0.02, float(self.args.demo_sphere_radius))
        frame_length = max(0.005, float(self.args.demo_frame_length))
        frame_radius = max(0.0008, 0.055 * frame_length)
        pose_count = len(self.orientation_candidates)
        origins = self._build_demo_origins(pose_count, center, sphere_radius)

        unit_vertices, unit_faces = self._build_unit_uv_sphere_mesh()
        self.server.scene.add_mesh_simple(
            "/orientation_demo/sphere",
            unit_vertices,
            unit_faces,
            color=(80, 155, 245),
            wireframe=True,
            opacity=float(np.clip(self.args.demo_sphere_opacity, 0.0, 1.0)),
            material="standard",
            flat_shading=False,
            side="double",
            scale=sphere_radius,
            position=center.astype(np.float32),
            cast_shadow=False,
            receive_shadow=False,
        )
        self.server.scene.add_point_cloud(
            "/orientation_demo/origins",
            origins.astype(np.float32),
            np.tile(np.array([[255, 120, 70]], dtype=np.uint8), (pose_count, 1)),
            point_size=max(0.004, 0.16 * frame_length),
            point_shape="rounded",
            precision="float32",
        )

        for idx, (origin, wxyz) in enumerate(zip(origins, self.orientation_candidates)):
            self.server.scene.add_frame(
                f"/orientation_demo/pose_{idx:03d}",
                position=tuple(origin.tolist()),
                wxyz=tuple(np.asarray(wxyz, dtype=np.float64).tolist()),
                axes_length=frame_length,
                axes_radius=frame_radius,
            )

        summary = (
            f"orientation demo: poses={pose_count}, directions={int(self.args.direction_samples)}, "
            f"rolls={int(self.args.roll_samples)}, sphere_radius={sphere_radius:.3f} m"
        )
        self.status_text.value = "Orientation demo ready"
        self.summary_text.value = summary
        self.sample_count_text.value = str(pose_count)
        self.legend_html.content = self._build_legend_html("N/A (orientation demo)")
        self.logger.info(summary)

    def _build_grid(self):
        xmin, xmax, ymin, ymax, zmin, zmax = [float(v) for v in self.args.bounds]
        step = float(self.args.step)
        if step <= 0.0:
            raise ValueError("--step must be positive")

        # Sample voxel centers instead of box corners.
        xs = np.arange(xmin + 0.5 * step, xmax - 0.5 * step + 1e-12, step, dtype=np.float64)
        ys = np.arange(ymin + 0.5 * step, ymax - 0.5 * step + 1e-12, step, dtype=np.float64)
        zs = np.arange(zmin + 0.5 * step, zmax - 0.5 * step + 1e-12, step, dtype=np.float64)
        mesh = np.meshgrid(xs, ys, zs, indexing="ij")
        points = np.stack(mesh, axis=-1).reshape(-1, 3)
        return points

    def _build_grid_axes(self):
        xmin, xmax, ymin, ymax, zmin, zmax = [float(v) for v in self.args.bounds]
        step = float(self.args.step)
        if step <= 0.0:
            raise ValueError("--step must be positive")

        xs = np.arange(xmin + 0.5 * step, xmax - 0.5 * step + 1e-12, step, dtype=np.float64)
        ys = np.arange(ymin + 0.5 * step, ymax - 0.5 * step + 1e-12, step, dtype=np.float64)
        zs = np.arange(zmin + 0.5 * step, zmax - 0.5 * step + 1e-12, step, dtype=np.float64)
        return xs, ys, zs

    @staticmethod
    def _build_transform(position: np.ndarray, wxyz: np.ndarray) -> np.ndarray:
        w, x, y, z = [float(v) for v in wxyz]
        rot = np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rot
        transform[:3, 3] = np.asarray(position, dtype=np.float64)
        return transform

    @staticmethod
    def _quat_angle_error_rad(target_wxyz: np.ndarray, actual_wxyz: np.ndarray) -> float:
        t = np.array(target_wxyz, dtype=np.float64)
        a = np.array(actual_wxyz, dtype=np.float64)
        if np.linalg.norm(t) < 1e-8 or np.linalg.norm(a) < 1e-8:
            return np.pi
        t = t / np.linalg.norm(t)
        a = a / np.linalg.norm(a)
        dot = float(np.clip(np.abs(np.dot(t, a)), 0.0, 1.0))
        return float(2.0 * np.arccos(dot))

    @staticmethod
    def _build_unit_icosahedron_mesh() -> tuple[np.ndarray, np.ndarray]:
        """Return a unit-radius icosahedron mesh as (vertices, faces)."""
        phi = (1.0 + np.sqrt(5.0)) * 0.5
        vertices = np.array(
            [
                (-1.0, phi, 0.0),
                (1.0, phi, 0.0),
                (-1.0, -phi, 0.0),
                (1.0, -phi, 0.0),
                (0.0, -1.0, phi),
                (0.0, 1.0, phi),
                (0.0, -1.0, -phi),
                (0.0, 1.0, -phi),
                (phi, 0.0, -1.0),
                (phi, 0.0, 1.0),
                (-phi, 0.0, -1.0),
                (-phi, 0.0, 1.0),
            ],
            dtype=np.float32,
        )
        vertices /= np.maximum(np.linalg.norm(vertices, axis=1, keepdims=True), 1e-12)
        faces = np.array(
            [
                (0, 11, 5),
                (0, 5, 1),
                (0, 1, 7),
                (0, 7, 10),
                (0, 10, 11),
                (1, 5, 9),
                (5, 11, 4),
                (11, 10, 2),
                (10, 7, 6),
                (7, 1, 8),
                (3, 9, 4),
                (3, 4, 2),
                (3, 2, 6),
                (3, 6, 8),
                (3, 8, 9),
                (4, 9, 5),
                (2, 4, 11),
                (6, 2, 10),
                (8, 6, 7),
                (9, 8, 1),
            ],
            dtype=np.uint32,
        )
        return vertices, faces

    @staticmethod
    def _build_unit_uv_sphere_mesh(latitude_segments: int = 32, longitude_segments: int = 64) -> tuple[np.ndarray, np.ndarray]:
        latitude_segments = max(8, int(latitude_segments))
        longitude_segments = max(12, int(longitude_segments))

        vertices = []
        for lat_idx in range(latitude_segments + 1):
            phi = np.pi * float(lat_idx) / float(latitude_segments)
            sin_phi = float(np.sin(phi))
            cos_phi = float(np.cos(phi))
            for lon_idx in range(longitude_segments):
                theta = 2.0 * np.pi * float(lon_idx) / float(longitude_segments)
                vertices.append((sin_phi * np.cos(theta), sin_phi * np.sin(theta), cos_phi))

        faces = []
        for lat_idx in range(latitude_segments):
            row_start = lat_idx * longitude_segments
            next_row_start = (lat_idx + 1) * longitude_segments
            for lon_idx in range(longitude_segments):
                next_lon_idx = (lon_idx + 1) % longitude_segments
                a = row_start + lon_idx
                b = row_start + next_lon_idx
                c = next_row_start + lon_idx
                d = next_row_start + next_lon_idx
                if lat_idx > 0:
                    faces.append((a, c, b))
                if lat_idx < latitude_segments - 1:
                    faces.append((b, c, d))

        return np.asarray(vertices, dtype=np.float32), np.asarray(faces, dtype=np.uint32)

    @staticmethod
    def _viridis_colormap(values: np.ndarray) -> np.ndarray:
        """Map normalized values in [0, 1] to viridis RGB colors."""
        v = np.clip(np.asarray(values, dtype=np.float32), 0.0, 1.0)
        anchors = np.array([0.0, 0.25, 0.5, 0.75, 1.0], dtype=np.float32)
        rgb_anchors = np.array(
            [
                [0.267004, 0.004874, 0.329415],
                [0.229739, 0.322361, 0.545706],
                [0.127568, 0.566949, 0.550556],
                [0.369214, 0.788888, 0.382914],
                [0.993248, 0.906157, 0.143936],
            ],
            dtype=np.float32,
        )
        r = np.interp(v, anchors, rgb_anchors[:, 0])
        g = np.interp(v, anchors, rgb_anchors[:, 1])
        b = np.interp(v, anchors, rgb_anchors[:, 2])
        return np.stack([r, g, b], axis=1).astype(np.float32)

    @staticmethod
    def _build_legend_html(ri_range_text: str) -> str:
        return f"""
<div style="font-family: sans-serif; line-height: 1.35; min-width: 260px;">
    <div style="font-weight: 700; margin-bottom: 6px;">Legend</div>
    <div style="margin-bottom: 6px;">
        Unreachable (N_reach=0):
        <span style="display:inline-block;width:14px;height:14px;border-radius:50%;background: rgba(170,170,170,0.35); border:1px solid rgba(120,120,120,0.6);vertical-align:middle;"></span>
        transparent spheres
    </div>
    <div style="margin-bottom: 4px;">Reachable RI color bar:</div>
    <div style="height: 14px; border-radius: 7px; border: 1px solid #666; background: linear-gradient(90deg, rgb(68,1,84) 0%, rgb(59,82,139) 25%, rgb(33,145,140) 50%, rgb(94,201,98) 75%, rgb(253,231,37) 100%);"></div>
    <div style="display:flex; justify-content:space-between; margin-top: 2px; font-size: 12px;">
        <span>Low RI (viridis dark)</span><span>High RI (viridis yellow)</span>
    </div>
    <div style="margin-top: 6px;">RI range: {ri_range_text}</div>
</div>
"""

    def _ik_with_validation(self, target_transform: np.ndarray, base_seed: np.ndarray):
        target_position = np.asarray(target_transform[:3, 3], dtype=np.float64)
        target_wxyz = self._rotmat_to_wxyz(target_transform[:3, :3])
        pos_tol = 1e-6
        ori_tol = 1e-6

        rng = np.random.default_rng(42)
        candidate_seeds = [np.array(base_seed, dtype=np.float64, copy=True)]
        candidate_seeds.append(np.array(self.cfg, dtype=np.float64, copy=True))
        candidate_seeds.append(np.array(self.cfg, dtype=np.float64, copy=True))
        for _ in range(int(self.args.seed_count)):
            candidate_seeds.append(
                np.array(base_seed, dtype=np.float64, copy=True)
                + rng.normal(0.0, float(self.args.max_seed_jitter), size=base_seed.shape)
            )

        for seed in candidate_seeds:
            try:
                cfg_solution = self.pks.solve_ik(
                    robot=self.robot,
                    target_link_name=EE_LINK,
                    target_position=target_position,
                    target_wxyz=target_wxyz,
                    init_config=np.asarray(seed, dtype=np.float64),
                )
            except TypeError:
                try:
                    cfg_solution = self.pks.solve_ik(
                        robot=self.robot,
                        target_link_name=EE_LINK,
                        target_position=target_position,
                        target_wxyz=target_wxyz,
                    )
                except Exception:
                    continue
            except Exception:
                continue

            ee_xyz, ee_wxyz = self._fk_ee(cfg_solution)
            pos_err = float(np.linalg.norm(ee_xyz - target_position))
            ori_err = self._quat_angle_error_rad(target_wxyz, ee_wxyz)

            # Use tiny epsilon thresholds instead of exact zero to keep the check strict but usable.
            if pos_err <= pos_tol and ori_err <= ori_tol:
                return np.asarray(cfg_solution, dtype=np.float64), pos_err, ori_err

        return None, None, None

    def _clear_workspace_render(self) -> None:
        if self._reachable_handle is not None:
            self._reachable_handle.remove()
            self._reachable_handle = None
        if self._unreachable_handle is not None:
            self._unreachable_handle.remove()
            self._unreachable_handle = None

    def _refresh_workspace_render(self) -> None:
        if self._scan_voxel_arr is None or self._scan_voxel_ri_arr is None:
            return

        voxel_arr = self._scan_voxel_arr
        voxel_ri_arr = self._scan_voxel_ri_arr
        render_mask = np.ones((voxel_arr.shape[0],), dtype=bool)

        if self.enable_z_slice.value:
            z_min = min(float(self.z_slice_min.value), float(self.z_slice_max.value))
            z_max = max(float(self.z_slice_min.value), float(self.z_slice_max.value))
            render_mask &= voxel_arr[:, 2] >= z_min
            render_mask &= voxel_arr[:, 2] <= z_max

        filtered_points = voxel_arr[render_mask]
        filtered_ri = voxel_ri_arr[render_mask]

        self._clear_workspace_render()

        if filtered_points.shape[0] == 0:
            self.legend_html.content = self._build_legend_html("N/A (slice empty)")
            return

        reachable_mask = filtered_ri > 0.0
        unreachable_mask = ~reachable_mask

        reachable_points = filtered_points[reachable_mask]
        reachable_ri = filtered_ri[reachable_mask]
        unreachable_points = filtered_points[unreachable_mask]

        if reachable_points.shape[0] > 0:
            ri_min = float(np.min(reachable_ri))
            ri_max = float(np.max(reachable_ri))
            if ri_max - ri_min > 1e-8:
                normalized_ri = (reachable_ri - ri_min) / (ri_max - ri_min)
            else:
                normalized_ri = np.zeros_like(reachable_ri, dtype=np.float32)

            reachable_colors = self._viridis_colormap(normalized_ri)
            self._reachable_handle = self.server.scene.add_point_cloud(
                "/workspace/ri_voxels_reachable",
                reachable_points,
                reachable_colors,
                point_size=self._scan_point_size,
                point_shape="rounded",
                precision="float32",
            )
            legend_range_text = f"{ri_min:.3f} to {ri_max:.3f}"
        else:
            legend_range_text = "N/A (no reachable voxels in view)"

        if self.show_unreachable.value and unreachable_points.shape[0] > 0:
            unit_vertices, unit_faces = self._build_unit_icosahedron_mesh()
            sphere_scales = np.full((unreachable_points.shape[0],), self._scan_point_size * 0.35, dtype=np.float32)
            sphere_wxyzs = np.tile(np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), (unreachable_points.shape[0], 1))
            self._unreachable_handle = self.server.scene.add_batched_meshes_simple(
                "/workspace/ri_voxels_unreachable",
                unit_vertices,
                unit_faces,
                sphere_wxyzs,
                unreachable_points.astype(np.float32),
                batched_scales=sphere_scales,
                batched_colors=(170, 170, 170),
                batched_opacities=np.full((unreachable_points.shape[0],), 0.22, dtype=np.float32),
                opacity=1.0,
                material="standard",
                flat_shading=False,
                cast_shadow=False,
                receive_shadow=False,
            )

        self.legend_html.content = self._build_legend_html(legend_range_text)

    def _evaluate_position(self, target_position: np.ndarray, base_seed: np.ndarray):
        reachable_cfg = None
        reachable_pose_count = 0
        best_pos_err = None
        best_ori_err = None
        for target_wxyz in self.orientation_candidates:
            target_transform = self._build_transform(target_position, target_wxyz)
            cfg_solution, pos_err, ori_err = self._ik_with_validation(
                target_transform=target_transform,
                base_seed=base_seed,
            )
            if cfg_solution is None:
                continue

            reachable_pose_count += 1
            if reachable_cfg is None:
                reachable_cfg = cfg_solution
                best_pos_err = pos_err
                best_ori_err = ori_err

        return reachable_cfg, reachable_pose_count, best_pos_err, best_ori_err

    def _fk_ee(self, cfg):
        link_names = list(self.robot.links.names)
        if EE_LINK not in link_names:
            self.logger.error("EE link '%s' not in URDF links: %s", EE_LINK, link_names)
            return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
        idx = link_names.index(EE_LINK)
        Ts = np.asarray(self.robot.forward_kinematics(cfg))
        wxyz = Ts[idx, :4]
        xyz = Ts[idx, 4:]
        return xyz, wxyz

    def _save_scan_outputs(
        self,
        output_npz: str,
        voxel_arr: np.ndarray,
        voxel_ri_arr: np.ndarray,
        pose_success_counts_arr: np.ndarray,
        total_pose_per_voxel: int,
        elapsed: float,
        reachable_count: int,
        total_pose_tests: int,
    ) -> None:
        output_path = Path(output_npz)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        xs, ys, zs = self._build_grid_axes()
        grid_shape = np.array([len(xs), len(ys), len(zs)], dtype=np.int32)
        voxel_indices = np.indices(grid_shape, dtype=np.int32).reshape(3, -1).T
        reachable_mask = voxel_ri_arr > 0.0

        np.savez_compressed(
            output_path,
            voxel_points=voxel_arr,
            voxel_indices=voxel_indices,
            ri_values=voxel_ri_arr,
            pose_success_counts=pose_success_counts_arr,
            reachable_mask=reachable_mask,
            total_pose_per_voxel=int(total_pose_per_voxel),
            fixed_wxyz=self.fixed_wxyz,
            bounds=np.asarray(self.args.bounds, dtype=np.float64),
            step=float(self.args.step),
            direction_samples=int(self.args.direction_samples),
            roll_samples=int(self.args.roll_samples),
            x_centers=xs,
            y_centers=ys,
            z_centers=zs,
            grid_shape=grid_shape,
        )

        csv_path = output_path.with_name(f"{output_path.stem}_voxels.csv")
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                "voxel_id",
                "ix",
                "iy",
                "iz",
                "x",
                "y",
                "z",
                "N_reach",
                "N_total",
                "RI",
                "reachable",
            ])
            for voxel_id, (index_triplet, point, n_reach, ri_value, reachable) in enumerate(
                zip(voxel_indices, voxel_arr, pose_success_counts_arr, voxel_ri_arr, reachable_mask)
            ):
                writer.writerow([
                    int(voxel_id),
                    int(index_triplet[0]),
                    int(index_triplet[1]),
                    int(index_triplet[2]),
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                    int(n_reach),
                    int(total_pose_per_voxel),
                    float(ri_value),
                    int(bool(reachable)),
                ])

        self.logger.info(
            "Saved scan result bundle: npz=%s csv=%s",
            output_path,
            csv_path,
        )

    def _run_scan(self):
        try:
            points = self._build_grid()
        except Exception as exc:
            self.logger.exception("Failed to build sampling grid")
            self.status_text.value = f"Grid error: {exc}"
            return

        total = int(points.shape[0])
        voxel_points = []
        voxel_ri_values = []
        pose_success_counts = []
        nominal_seed = np.array(self.cfg, dtype=np.float64, copy=True)
        total_pose_tests = 0
        total_pose_per_voxel = max(1, len(self.orientation_candidates))

        self.status_text.value = f"Scanning {total} samples..."
        t0 = time.time()

        for idx, point in enumerate(points, start=1):
            cfg_solution, pose_success_count, pos_err, ori_err = self._evaluate_position(point, nominal_seed)
            total_pose_tests += len(self.orientation_candidates)

            ri = float(pose_success_count) / float(total_pose_per_voxel)
            voxel_points.append(point)
            voxel_ri_values.append(ri)
            pose_success_counts.append(pose_success_count)

            if idx % max(1, total // 100) == 0 or idx == total:
                ratio = 100.0 * np.count_nonzero(np.asarray(voxel_ri_values, dtype=np.float64) > 0.0) / max(1, idx)
                pose_ratio = 100.0 * sum(pose_success_counts) / max(1, total_pose_tests)
                self.status_text.value = f"Scanned {idx}/{total} points, reachable {ratio:.1f}%, pose-hit {pose_ratio:.1f}%"
                self.logger.info(
                    "Scanned %d/%d points, reachable=%d, unreachable=%d, pose_successes=%d/%d",
                    idx,
                    total,
                    int(np.count_nonzero(np.asarray(voxel_ri_values, dtype=np.float64) > 0.0)),
                    int(idx - np.count_nonzero(np.asarray(voxel_ri_values, dtype=np.float64) > 0.0)),
                    sum(pose_success_counts),
                    total_pose_tests,
                )

        elapsed = time.time() - t0
        voxel_arr = np.asarray(voxel_points, dtype=np.float64) if voxel_points else np.zeros((0, 3), dtype=np.float64)
        voxel_ri_arr = np.asarray(voxel_ri_values, dtype=np.float32) if voxel_ri_values else np.zeros((0,), dtype=np.float32)
        pose_success_counts_arr = np.asarray(pose_success_counts, dtype=np.int32) if pose_success_counts else np.zeros((0,), dtype=np.int32)

        self._scan_voxel_arr = voxel_arr
        self._scan_voxel_ri_arr = voxel_ri_arr
        self._scan_point_size = max(0.003, float(self.args.step) * float(self.args.point_size_scale))
        if voxel_arr.shape[0] > 0:
            self._refresh_workspace_render()

        reachable_count = int(np.count_nonzero(voxel_ri_arr > 0.0))
        success_rate = 100.0 * reachable_count / max(1, total)
        pose_success_total = int(np.sum(pose_success_counts_arr))
        pose_success_rate = 100.0 * pose_success_total / max(1, total_pose_tests)
        summary = (
            f"reachable={reachable_count}/{total} ({success_rate:.1f}%), "
            f"failed={int(total - reachable_count)}, pose-hit={pose_success_total}/{total_pose_tests} ({pose_success_rate:.1f}%), time={elapsed:.1f}s"
        )
        self.summary_text.value = summary
        self.sample_count_text.value = str(total)
        self.status_text.value = "Scan finished"
        self.logger.info("Workspace scan finished: %s", summary)

        if self.args.output_npz:
            self._save_scan_outputs(
                output_npz=self.args.output_npz,
                voxel_arr=voxel_arr,
                voxel_ri_arr=voxel_ri_arr,
                pose_success_counts_arr=pose_success_counts_arr,
                total_pose_per_voxel=int(total_pose_per_voxel),
                elapsed=elapsed,
                reachable_count=reachable_count,
                total_pose_tests=total_pose_tests,
            )


def main() -> None:
    args = _parse_args()
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    visualizer = WorkspaceReachabilityVisualizer(args)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        visualizer.logger.info("Interrupted by user")


if __name__ == "__main__":
    main()