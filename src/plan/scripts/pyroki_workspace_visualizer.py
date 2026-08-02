#!/usr/bin/env python3

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
        "src/arms",
        "src",
        "install",
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
    "src/arms/combine_urdf/urdf/combine_urdf.urdf",
)
EE_LINK = os.environ.get("ROBO_ARM_EE_LINK", "gripper")
DEFAULT_OUTPUT_NPZ = "output/workspace_scan.npz"
DEFAULT_CFG = np.array([0.0, 0.3, 0.0, 0.0, 0.0, 0.0])

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
        default=[-0.1, 1.6, -0.5, 1.2, 0.8, 1.9],
    )
    parser.add_argument(
        "--step",
        type=float,
        default=0.05,
    )
    parser.add_argument(
        "--ik-pos-tol",
        type=float,
        default=0.003,
    )
    parser.add_argument(
        "--ik-ori-tol-deg",
        type=float,
        default=3.0,
    )
    parser.add_argument(
        "--max-seed-jitter",
        type=float,
        default=0.08,
    )
    parser.add_argument(
        "--seed-count",
        type=int,
        default=6,
    )
    parser.add_argument(
        "--ori-samples",
        type=int,
        default=256,
    )
    parser.add_argument(
        "--fixed-wxyz",
        nargs=4,
        type=float,
        metavar=("W", "X", "Y", "Z"),
        default=None,
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
    )
    parser.add_argument(
        "--output-npz",
        type=str,
        default=DEFAULT_OUTPUT_NPZ,
    )
    parser.add_argument(
        "--input-npz",
        type=str,
        default=None,
    )
    parser.add_argument(
        "--point-size-scale",
        type=float,
        default=0.6,
    )
    parser.add_argument(
        "--orientation-demo",
        action="store_true",
    )
    parser.add_argument(
        "--demo-center",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=[0.75, -0.45, 0.55],
    )
    parser.add_argument(
        "--demo-sphere-radius",
        type=float,
        default=0.25,
    )
    parser.add_argument(
        "--demo-frame-length",
        type=float,
        default=0.025,
    )
    parser.add_argument(
        "--demo-sphere-opacity",
        type=float,
        default=0.08,
    )
    return parser.parse_args()


class WorkspaceReachabilityVisualizer:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.logger = LOGGER
        self._pose_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._suppress_target_sync = False

        self.robot = None
        self.pks = None
        self.urdf = None
        self.viser_urdf = None
        self.target_control = None
        self.target_pos_x = None
        self.target_pos_y = None
        self.target_pos_z = None
        self.target_qw = None
        self.target_qx = None
        self.target_qy = None
        self.target_qz = None
        self.solve_target_btn = None
        self.pk_joint_names = []
        self.cfg = np.array(DEFAULT_CFG, dtype=np.float64, copy=True)
        self.last_ik_pos_err_m = None
        self.last_ik_ori_err_rad = None
        self.nominal_ee_pos = np.zeros((3,), dtype=np.float64)
        self.nominal_ee_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.fixed_wxyz = None
        self.orientation_candidates = []
        self._scan_thread = None
        self._loaded_scan_bundle = self._read_scan_bundle(args.input_npz) if args.input_npz else None
        self._scan_step = float(self.args.step)
        self._layer_groups: dict[float, np.ndarray] = {}
        self._layer_handles: dict[str, dict[float, Any]] = {"reachable": {}, "unreachable": {}}
        self._layer_color_min = 0.0
        self._layer_color_max = 1.0
        self._render_refresh_lock = threading.Lock()
        self._render_refresh_timer = None
        self._enable_target_interaction = self._loaded_scan_bundle is not None

        self.server = viser.ViserServer(port=args.port)
        self.server.scene.add_grid("/ground", width=2, height=2)

        self.status_text = self.server.gui.add_text("Scan status", "Starting...", disabled=True)
        self.summary_text = self.server.gui.add_text("Workspace summary", "Waiting...", disabled=True)
        self.sample_count_text = self.server.gui.add_text("Sample count", "0", disabled=True)
        self.ee_pose_text = None
        if self._enable_target_interaction:
            self.ee_pose_text = self.server.gui.add_text("Current EE pose", "Waiting...", disabled=True)
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

        self.enable_z_slice.on_update(lambda _: self._request_workspace_render_refresh())
        self.z_slice_min.on_update(lambda _: self._request_workspace_render_refresh())
        self.z_slice_max.on_update(lambda _: self._request_workspace_render_refresh())
        self.show_unreachable.on_update(lambda _: self._request_workspace_render_refresh())

        self.server.scene.add_frame(
            "/workspace_reference",
            position=(0.0, 0.0, 0.0),
            wxyz=(1.0, 0.0, 0.0, 0.0),
            axes_length=0.12,
            axes_radius=0.004,
        )

        if self._loaded_scan_bundle is not None:
            self._initialize_robot_runtime()
            self._initialize_target_control()
            self._start_ee_pose_printer()
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

    def _initialize_target_control(self) -> None:
        if self.robot is None or not self._enable_target_interaction:
            return

        ee_pos, ee_wxyz = self._fk_ee(self.cfg)
        if self.target_control is None:
            self.target_control = self.server.scene.add_transform_controls(
                "/workspace_target",
                scale=0.15,
                position=tuple(float(x) for x in ee_pos),
                wxyz=tuple(float(x) for x in ee_wxyz),
            )

            @self.target_control.on_update
            def _(_):
                self._sync_inputs_from_target_control()
                self._handle_target_drag()

            self.server.gui.add_markdown("## IK Target Pose")
            self.target_pos_x = self.server.gui.add_text("target_x (m)", f"{float(ee_pos[0]):.6f}")
            self.target_pos_y = self.server.gui.add_text("target_y (m)", f"{float(ee_pos[1]):.6f}")
            self.target_pos_z = self.server.gui.add_text("target_z (m)", f"{float(ee_pos[2]):.6f}")
            self.target_qw = self.server.gui.add_text("target_qw", f"{float(ee_wxyz[0]):.6f}")
            self.target_qx = self.server.gui.add_text("target_qx", f"{float(ee_wxyz[1]):.6f}")
            self.target_qy = self.server.gui.add_text("target_qy", f"{float(ee_wxyz[2]):.6f}")
            self.target_qz = self.server.gui.add_text("target_qz", f"{float(ee_wxyz[3]):.6f}")
            self.solve_target_btn = self.server.gui.add_button("Solve IK From Input Pose")

            def _on_input_update(_):
                if self._suppress_target_sync:
                    return
                self._sync_target_control_from_inputs()

            for handle in (
                self.target_pos_x,
                self.target_pos_y,
                self.target_pos_z,
                self.target_qw,
                self.target_qx,
                self.target_qy,
                self.target_qz,
            ):
                handle.on_update(_on_input_update)

            @self.solve_target_btn.on_click
            def _(_):
                self._solve_from_input_pose()
        else:
            self.target_control.position = tuple(float(x) for x in ee_pos)
            self.target_control.wxyz = tuple(float(x) for x in ee_wxyz)
            self._sync_inputs_from_target_control()

    def _parse_input_pose(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        handles = (
            self.target_pos_x,
            self.target_pos_y,
            self.target_pos_z,
            self.target_qw,
            self.target_qx,
            self.target_qy,
            self.target_qz,
        )
        if any(h is None for h in handles):
            return None, None

        try:
            pos = np.array(
                [
                    float(str(self.target_pos_x.value).strip()),
                    float(str(self.target_pos_y.value).strip()),
                    float(str(self.target_pos_z.value).strip()),
                ],
                dtype=np.float64,
            )
            wxyz = np.array(
                [
                    float(str(self.target_qw.value).strip()),
                    float(str(self.target_qx.value).strip()),
                    float(str(self.target_qy.value).strip()),
                    float(str(self.target_qz.value).strip()),
                ],
                dtype=np.float64,
            )
        except (TypeError, ValueError):
            return None, None

        if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(wxyz)):
            return None, None

        q_norm = float(np.linalg.norm(wxyz))
        if q_norm < 1e-8:
            return None, None
        return pos, (wxyz / q_norm)

    def _sync_inputs_from_target_control(self) -> None:
        if self.target_control is None:
            return
        if any(
            handle is None
            for handle in (
                self.target_pos_x,
                self.target_pos_y,
                self.target_pos_z,
                self.target_qw,
                self.target_qx,
                self.target_qy,
                self.target_qz,
            )
        ):
            return

        pos = np.asarray(self.target_control.position, dtype=np.float64)
        wxyz = np.asarray(self.target_control.wxyz, dtype=np.float64)
        if pos.shape != (3,) or wxyz.shape != (4,):
            return
        q_norm = float(np.linalg.norm(wxyz))
        if q_norm < 1e-8:
            return
        wxyz = wxyz / q_norm

        self._suppress_target_sync = True
        try:
            self.target_pos_x.value = f"{float(pos[0]):.8f}"
            self.target_pos_y.value = f"{float(pos[1]):.8f}"
            self.target_pos_z.value = f"{float(pos[2]):.8f}"
            self.target_qw.value = f"{float(wxyz[0]):.8f}"
            self.target_qx.value = f"{float(wxyz[1]):.8f}"
            self.target_qy.value = f"{float(wxyz[2]):.8f}"
            self.target_qz.value = f"{float(wxyz[3]):.8f}"
        finally:
            self._suppress_target_sync = False

    def _sync_target_control_from_inputs(self) -> bool:
        if self.target_control is None:
            return False
        pos, wxyz = self._parse_input_pose()
        if pos is None or wxyz is None:
            self.status_text.value = "Input pose invalid"
            return False

        self._suppress_target_sync = True
        try:
            self.target_control.position = tuple(float(v) for v in pos)
            self.target_control.wxyz = tuple(float(v) for v in wxyz)
        finally:
            self._suppress_target_sync = False
        return True

    def _solve_from_input_pose(self) -> None:
        if not self._sync_target_control_from_inputs():
            return
        self._handle_target_drag()

    def _handle_target_drag(self) -> None:
        if self.target_control is None:
            return

        target_position = np.asarray(self.target_control.position, dtype=np.float64)
        target_wxyz = np.asarray(self.target_control.wxyz, dtype=np.float64)
        try:
            cfg_solution = self.pks.solve_ik(
                robot=self.robot,
                target_link_name=EE_LINK,
                target_position=target_position,
                target_wxyz=target_wxyz,
                init_config=np.array(self.cfg, dtype=np.float64, copy=True),
            )
        except TypeError:
            try:
                cfg_solution = self.pks.solve_ik(
                    robot=self.robot,
                    target_link_name=EE_LINK,
                    target_position=target_position,
                    target_wxyz=target_wxyz,
                )
            except Exception as exc:
                self.status_text.value = "Target IK failed"
                with self._pose_lock:
                    self.last_ik_pos_err_m = None
                    self.last_ik_ori_err_rad = None
                self.logger.warning(
                    "Target IK failed for pos=%s wxyz=%s: %s",
                    target_position.tolist(),
                    target_wxyz.tolist(),
                    exc,
                )
                return
        except Exception as exc:
            self.status_text.value = "Target IK failed"
            with self._pose_lock:
                self.last_ik_pos_err_m = None
                self.last_ik_ori_err_rad = None
            self.logger.warning(
                "Target IK failed for pos=%s wxyz=%s: %s",
                target_position.tolist(),
                target_wxyz.tolist(),
                exc,
            )
            return

        cfg_solution_arr = np.asarray(cfg_solution, dtype=np.float64)
        ee_pos, ee_wxyz = self._fk_ee(cfg_solution_arr)
        pos_err = float(np.linalg.norm(ee_pos - target_position))
        ori_err_rad = float(self._quat_angle_error_rad(target_wxyz, ee_wxyz))

        with self._pose_lock:
            self.cfg = cfg_solution_arr
            self.last_ik_pos_err_m = pos_err
            self.last_ik_ori_err_rad = ori_err_rad

        self.viser_urdf.update_cfg(self.cfg)
        self.status_text.value = (
            f"Target IK ok: pos_err={pos_err:.4f} m, ori_err={np.rad2deg(ori_err_rad):.2f} deg"
        )
        if self.ee_pose_text is not None:
            self.ee_pose_text.value = f"pos={ee_pos.tolist()} wxyz={ee_wxyz.tolist()}"
        self._sync_inputs_from_target_control()

    def _start_ee_pose_printer(self) -> None:
        if self._stop_event.is_set() or not self._enable_target_interaction or self.ee_pose_text is None:
            return

        thread = threading.Thread(target=self._print_ee_pose_loop, daemon=True)
        thread.start()

    def _print_ee_pose_loop(self) -> None:
        last_print = 0.0
        while not self._stop_event.is_set():
            now = time.time()
            if now - last_print >= 1.0:
                with self._pose_lock:
                    cfg = np.array(self.cfg, dtype=np.float64, copy=True)
                    pos_err = self.last_ik_pos_err_m
                    ori_err_rad = self.last_ik_ori_err_rad
                ee_pos, ee_wxyz = self._fk_ee(cfg)
                if self.ee_pose_text is not None:
                    self.ee_pose_text.value = f"pos={ee_pos.tolist()} wxyz={ee_wxyz.tolist()}"
                if pos_err is not None and ori_err_rad is not None:
                    err_text = f" ik_pos_err_m={pos_err:.6f} ik_ori_err_deg={np.rad2deg(ori_err_rad):.4f}"
                else:
                    err_text = " ik_pos_err_m=N/A ik_ori_err_deg=N/A"
                print(
                    f"[workspace_visualizer] ee_pos={ee_pos.tolist()} ee_wxyz={ee_wxyz.tolist()}{err_text}",
                    flush=True,
                )
                last_print = now
            self._stop_event.wait(0.1)

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
                "ori_samples": int(bundle["ori_samples"]) if "ori_samples" in bundle else None,
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
        self._scan_step = float(bundle.get("step", self.args.step))
        self._scan_point_size = max(0.003, float(bundle["step"]) * float(self.args.point_size_scale))
        self.fixed_wxyz = bundle.get("fixed_wxyz")
        self._prepare_workspace_layer_cache()

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
        if bundle.get("ori_samples") is not None:
            orientation_summary = f", orientations={int(bundle['ori_samples'])}"

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
    def _uniform_so3_quaternions(sample_count: int) -> np.ndarray:
        if sample_count <= 0:
            return np.zeros((0, 4), dtype=np.float64)

        rng = np.random.default_rng(0)
        pool_size = max(128 * sample_count, 1024)
        pool = rng.normal(size=(pool_size, 4)).astype(np.float64)
        pool = WorkspaceReachabilityVisualizer._normalize_unit_quaternions(pool)

        selected = np.empty((sample_count, 4), dtype=np.float64)
        selected[0] = pool[0]
        if sample_count == 1:
            return selected

        min_distances = np.full((pool_size,), np.inf, dtype=np.float64)
        min_distances = np.minimum(min_distances, np.arccos(np.clip(np.abs(pool @ selected[0]), 0.0, 1.0)))

        for selected_idx in range(1, sample_count):
            next_index = int(np.argmax(min_distances))
            selected[selected_idx] = pool[next_index]
            candidate_distances = np.arccos(np.clip(np.abs(pool @ selected[selected_idx]), 0.0, 1.0))
            min_distances = np.minimum(min_distances, candidate_distances)

        return selected

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

    def _build_orientation_candidates(self):
        ori_samples = int(self.args.ori_samples)
        if ori_samples <= 0:
            raise ValueError("--ori-samples must be positive")

        candidates = self._uniform_so3_quaternions(ori_samples)
        uniformity_report = self._test_uniformity(candidates)
        self.logger.info(
            "SO(3) orientation uniformity report: %s",
            {
                "ori_samples": ori_samples,
                "method": "greedy_maximin_haar",
                "pool_size": max(8 * ori_samples, 1024),
                **uniformity_report,
            },
        )
        return [np.asarray(quat, dtype=np.float64) for quat in candidates]

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
            f"orientation demo: poses={pose_count}, ori_samples={int(self.args.ori_samples)}, "
            f"sphere_radius={sphere_radius:.3f} m"
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
        for handle_map in self._layer_handles.values():
            for handle in list(handle_map.values()):
                try:
                    handle.remove()
                except Exception:
                    pass
            handle_map.clear()
        self._reachable_handle = None
        self._unreachable_handle = None
        # debug handles for temporary markers (support multiple per-layer handles)
        if hasattr(self, "_debug_handles"):
            try:
                for h in list(self._debug_handles):
                    if h is None:
                        continue
                    try:
                        h.remove()
                    except Exception:
                        pass
                self._debug_handles = []
            except Exception:
                self._debug_handles = []

    def _prepare_workspace_layer_cache(self) -> None:
        if self._scan_voxel_arr is None or self._scan_voxel_arr.shape[0] == 0:
            self._layer_groups = {}
            self._layer_handles = {"reachable": {}, "unreachable": {}}
            self._layer_color_min = 0.0
            self._layer_color_max = 1.0
            return

        z_values = np.round(np.asarray(self._scan_voxel_arr[:, 2], dtype=np.float64), 8)
        unique_zs = np.unique(z_values)
        self._layer_groups = {
            float(z_key): np.flatnonzero(z_values == z_key)
            for z_key in unique_zs
        }
        self._layer_handles = {"reachable": {}, "unreachable": {}}

        reachable_values = self._scan_voxel_ri_arr[self._scan_voxel_ri_arr > 0.0]
        if reachable_values.size > 0:
            self._layer_color_min = float(np.min(reachable_values))
            self._layer_color_max = float(np.max(reachable_values))
        else:
            self._layer_color_min = 0.0
            self._layer_color_max = 1.0

    def _request_workspace_render_refresh(self) -> None:
        with self._render_refresh_lock:
            if self._render_refresh_timer is not None:
                self._render_refresh_timer.cancel()

            timer = threading.Timer(0.08, self._refresh_workspace_render)
            timer.daemon = True
            self._render_refresh_timer = timer
            timer.start()

    def _get_visible_layer_keys(self) -> list[float]:
        if self._scan_voxel_arr is None or self._scan_voxel_arr.shape[0] == 0:
            return []

        layer_keys = sorted(self._layer_groups.keys())
        if not self.enable_z_slice.value:
            return layer_keys

        z_min = min(float(self.z_slice_min.value), float(self.z_slice_max.value))
        z_max = max(float(self.z_slice_min.value), float(self.z_slice_max.value))
        return [z_key for z_key in layer_keys if z_min <= z_key <= z_max]

    def _sync_layer_handle(self, layer_z: float, kind: str, should_show: bool) -> None:
        handle_map = self._layer_handles[kind]
        layer_indices = self._layer_groups.get(layer_z)
        if layer_indices is None:
            return

        existing_handle = handle_map.get(layer_z)
        if not should_show:
            if existing_handle is not None:
                try:
                    existing_handle.remove()
                except Exception:
                    pass
                del handle_map[layer_z]
            return

        if existing_handle is not None:
            return

        layer_points = self._scan_voxel_arr[layer_indices]
        layer_ri = self._scan_voxel_ri_arr[layer_indices]

        if kind == "reachable":
            visible_mask = layer_ri > 0.0
            if not np.any(visible_mask):
                return

            visible_points = layer_points[visible_mask]
            visible_ri = layer_ri[visible_mask]
            if self._layer_color_max - self._layer_color_min > 1e-8:
                normalized_ri = (visible_ri - self._layer_color_min) / (self._layer_color_max - self._layer_color_min)
            else:
                normalized_ri = np.zeros_like(visible_ri, dtype=np.float32)

            handle_map[layer_z] = self.server.scene.add_point_cloud(
                f"/workspace/ri_voxels_reachable/{layer_z:.8f}",
                visible_points,
                self._viridis_colormap(normalized_ri),
                point_size=self._scan_point_size,
                point_shape="rounded",
                precision="float32",
            )
            return

        if not self.show_unreachable.value:
            return

        visible_mask = layer_ri <= 0.0
        if not np.any(visible_mask):
            return

        visible_points = layer_points[visible_mask]
        unit_vertices, unit_faces = self._build_unit_icosahedron_mesh()
        sphere_scales = np.full((visible_points.shape[0],), self._scan_point_size * 0.35, dtype=np.float32)
        sphere_wxyzs = np.tile(np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), (visible_points.shape[0], 1))
        handle_map[layer_z] = self.server.scene.add_batched_meshes_simple(
            f"/workspace/ri_voxels_unreachable/{layer_z:.8f}",
            unit_vertices,
            unit_faces,
            sphere_wxyzs,
            visible_points.astype(np.float32),
            batched_scales=sphere_scales,
            batched_colors=(170, 170, 170),
            batched_opacities=np.full((visible_points.shape[0],), 0.22, dtype=np.float32),
            opacity=1.0,
            material="standard",
            flat_shading=False,
            cast_shadow=False,
            receive_shadow=False,
        )

    def _refresh_workspace_render(self) -> None:
        if self._scan_voxel_arr is None or self._scan_voxel_ri_arr is None:
            return

        visible_layers = self._get_visible_layer_keys()
        visible_layer_set = set(visible_layers)

        for kind in ("reachable", "unreachable"):
            handle_map = self._layer_handles[kind]
            for layer_z in list(handle_map.keys()):
                should_keep = layer_z in visible_layer_set and not (kind == "unreachable" and not self.show_unreachable.value)
                if should_keep:
                    continue
                try:
                    handle_map[layer_z].remove()
                except Exception:
                    pass
                del handle_map[layer_z]

        for layer_z in visible_layers:
            self._sync_layer_handle(layer_z, "reachable", True)
            self._sync_layer_handle(layer_z, "unreachable", True)

        visible_reachable_count = 0
        visible_total_count = 0
        for layer_z in visible_layers:
            layer_indices = self._layer_groups.get(layer_z)
            if layer_indices is None:
                continue
            layer_ri = self._scan_voxel_ri_arr[layer_indices]
            visible_total_count += int(layer_ri.shape[0])
            visible_reachable_count += int(np.count_nonzero(layer_ri > 0.0))

        if visible_total_count == 0:
            self.legend_html.content = self._build_legend_html("N/A (slice empty)")
        elif visible_reachable_count == 0:
            self.legend_html.content = self._build_legend_html("N/A (no reachable voxels in view)")
        else:
            self.legend_html.content = self._build_legend_html(f"{self._layer_color_min:.3f} to {self._layer_color_max:.3f} (global)")

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
            ori_samples=int(self.args.ori_samples),
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
        self._scan_step = float(self.args.step)
        self._scan_point_size = max(0.003, float(self.args.step) * float(self.args.point_size_scale))
        self._prepare_workspace_layer_cache()
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