#!/usr/bin/env python3
"""Interactive IK unit test tool for pyroki + viser.

Collision loading is enabled in URDF.load as requested.
"""

import importlib
import importlib.util
import logging
import os
import sys
import time

import numpy as np
import pyroki as pk
import viser
import yourdfpy
from viser.extras import ViserUrdf


def _patch_numpy_asarray_copy_kwarg() -> None:
    """Provide NumPy 2.x-style copy= support for older NumPy releases.

    JAX 0.9.x may call numpy.asarray(..., copy=...), but NumPy 1.26 does not
    accept that keyword. Patch only this process before importing JAX users.
    """
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
    """Import pyroki_snippets with optional path fallback.

    Preferred: make pyroki_snippets importable via PYTHONPATH.
    Fallback: try PYROKI_EXAMPLES_PATH, then auto-discover common examples paths.
    """
    try:
        return importlib.import_module("pyroki_snippets")
    except ModuleNotFoundError:
        candidate_paths = []

        env_path = os.environ.get("PYROKI_EXAMPLES_PATH")
        if env_path:
            candidate_paths.append(env_path)

        # Try to infer examples path from installed pyroki package location.
        try:
            spec = importlib.util.find_spec("pyroki")
            if spec and spec.origin:
                pyroki_pkg_dir = os.path.dirname(os.path.abspath(spec.origin))
                candidate_paths.extend(
                    [
                        os.path.join(pyroki_pkg_dir, "examples"),
                        os.path.join(os.path.dirname(pyroki_pkg_dir), "examples"),
                    ]
                )
        except Exception:
            pass

        # Common local workspace/dev locations.
        candidate_paths.extend(
            [
                "/home/hku/robo_arm/src/pyroki/examples",
                "/home/hku/robo_arm/third_party/pyroki/examples",
                "/home/hku/third_party/pyroki/examples",
                os.path.join(os.path.expanduser("~"), "pyroki", "examples"),
            ]
        )

        seen = set()
        for path in candidate_paths:
            if not path:
                continue
            norm = os.path.abspath(path)
            if norm in seen:
                continue
            seen.add(norm)
            has_module_file = os.path.isfile(os.path.join(norm, "pyroki_snippets.py"))
            has_pkg_dir = os.path.isfile(
                os.path.join(norm, "pyroki_snippets", "__init__.py")
            )
            if not (has_module_file or has_pkg_dir):
                continue
            if norm not in sys.path:
                sys.path.insert(0, norm)
            try:
                return importlib.import_module("pyroki_snippets")
            except ModuleNotFoundError:
                continue

        raise ModuleNotFoundError(
            "Cannot import 'pyroki_snippets'. Tried PYTHONPATH and auto-discovery paths. "
            "Set PYROKI_EXAMPLES_PATH to your pyroki examples directory if needed."
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
pks = _import_pyroki_snippets()

URDF_PATH = os.environ.get(
    "ROBO_ARM_URDF",
    "/home/hku/robo_arm/src/arms/robo_arm10/urdf/robo_arm10.urdf",
)
EE_LINK = os.environ.get("ROBO_ARM_EE_LINK", "gripper")
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

LOGGER = logging.getLogger("pyroki_ik_unit_test")


class PyrokiIKUnitTest:
    def __init__(self):
        self.logger = LOGGER
        self._suppress_target_sync = False
        self.logger.info(f"Loading URDF: {URDF_PATH}")

        self.urdf = yourdfpy.URDF.load(
            URDF_PATH,
            filename_handler=_make_package_filename_handler(),
            build_collision_scene_graph=True,
            load_meshes=True,
            load_collision_meshes=True,
        )
        self.robot = pk.Robot.from_urdf(self.urdf)

        self.pk_joint_names = list(self.robot.joints.actuated_names)
        self.logger.info(f"pyroki actuated joints: {self.pk_joint_names}")

        self.cfg = np.array([0.5, -0.5, 0.3, 0.0, 1.0, 0.0], dtype=np.float64)
        self.last_print_time = time.time()

        ee_pos, ee_wxyz = self._fk_ee(self.cfg)
        self.logger.info(
            f"initial EE pose: pos={ee_pos.tolist()}, wxyz={ee_wxyz.tolist()}"
        )

        self.server = viser.ViserServer()
        self.server.scene.add_grid("/ground", width=2, height=2)
        self.viser_urdf = ViserUrdf(self.server, urdf_or_path=self.urdf)
        self.viser_urdf.update_cfg(self.cfg)

        self.server.gui.add_markdown("## IK Target Pose")
        self.server.gui.add_markdown(
            "Use either method below: (1) type precise xyz+xyzw values, "
            "or (2) drag/rotate the 3D target gizmo for quick pose tuning."
        )
        self.target_x = self.server.gui.add_text("target_x (m)", f"{float(ee_pos[0]):.6f}")
        self.target_y = self.server.gui.add_text("target_y (m)", f"{float(ee_pos[1]):.6f}")
        self.target_z = self.server.gui.add_text("target_z (m)", f"{float(ee_pos[2]):.6f}")
        self.target_qx = self.server.gui.add_text("target_qx", f"{float(ee_wxyz[1]):.6f}")
        self.target_qy = self.server.gui.add_text("target_qy", f"{float(ee_wxyz[2]):.6f}")
        self.target_qz = self.server.gui.add_text("target_qz", f"{float(ee_wxyz[3]):.6f}")
        self.target_qw = self.server.gui.add_text("target_qw", f"{float(ee_wxyz[0]):.6f}")
        self.solve_btn = self.server.gui.add_button("Solve IK From Target Pose")
        self.result_text = self.server.gui.add_text(
            "Last IK Result", "Not solved yet", disabled=True
        )
        self.timing = self.server.gui.add_number("IK time (ms)", 0.0, disabled=True)

        self.target_pose_gizmo = self.server.scene.add_transform_controls(
            "/target_pose",
            scale=0.15,
            position=(float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])),
            wxyz=(
                float(ee_wxyz[0]),
                float(ee_wxyz[1]),
                float(ee_wxyz[2]),
                float(ee_wxyz[3]),
            ),
        )

        self.server.scene.add_frame(
            "/target_pose_axes",
            position=(float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])),
            wxyz=(
                float(ee_wxyz[0]),
                float(ee_wxyz[1]),
                float(ee_wxyz[2]),
                float(ee_wxyz[3]),
            ),
            axes_length=0.12,
            axes_radius=0.004,
        )

        def _on_target_input_update(_):
            if self._suppress_target_sync:
                return
            self._update_target_pose_visualization()

        for handle in (
            self.target_x,
            self.target_y,
            self.target_z,
            self.target_qx,
            self.target_qy,
            self.target_qz,
            self.target_qw,
        ):
            handle.on_update(_on_target_input_update)

        @self.solve_btn.on_click
        def _(_):
            self._solve_from_input_pose_and_update_ui()

        @self.target_pose_gizmo.on_update
        def _(_):
            if self._suppress_target_sync:
                return
            self._sync_text_from_gizmo()

        self.logger.info(
            f"viser server running, open the URL above. Input target pose and click button. EE link = {EE_LINK}"
        )

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

    def _ik_with_validation(self, target_position: np.ndarray, target_wxyz: np.ndarray):
        pos_tol = 0.003
        ori_tol = np.deg2rad(3.0)

        base_seed = np.array(self.cfg, dtype=np.float64, copy=True)

        rng = np.random.default_rng(42)
        candidate_seeds = [base_seed]
        candidate_seeds.append(np.array(self.cfg, dtype=np.float64, copy=True))
        for _ in range(6):
            candidate_seeds.append(base_seed + rng.normal(0.0, 0.08, size=base_seed.shape))

        best = None
        best_score = float("inf")

        for idx, seed in enumerate(candidate_seeds):
            try:
                cfg_solution = pks.solve_ik(
                    robot=self.robot,
                    target_link_name=EE_LINK,
                    target_position=target_position,
                    target_wxyz=target_wxyz,
                    init_config=np.asarray(seed, dtype=np.float64),
                )
            except TypeError:
                try:
                    cfg_solution = pks.solve_ik(
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
            score = pos_err + 0.05 * ori_err

            self.logger.debug(
                "IK trial %d: pos_err=%.6f m, ori_err=%.4f rad" % (idx, pos_err, ori_err)
            )

            if score < best_score:
                best_score = score
                best = (cfg_solution, pos_err, ori_err)

            if pos_err <= pos_tol and ori_err <= ori_tol:
                return cfg_solution, pos_err, ori_err

        if best is None:
            return None, None, None
        return best[0], best[1], best[2]

    def _get_target_pose(self):
        try:
            target_position = np.array(
                [
                    float(str(self.target_x.value).strip()),
                    float(str(self.target_y.value).strip()),
                    float(str(self.target_z.value).strip()),
                ],
                dtype=np.float64,
            )
            target_xyzw = np.array(
                [
                    float(str(self.target_qx.value).strip()),
                    float(str(self.target_qy.value).strip()),
                    float(str(self.target_qz.value).strip()),
                    float(str(self.target_qw.value).strip()),
                ],
                dtype=np.float64,
            )
        except ValueError:
            return None, None
        return target_position, target_xyzw

    def _update_target_pose_visualization(self):
        target_position, target_xyzw = self._get_target_pose()
        if target_position is None or target_xyzw is None:
            return
        if not np.all(np.isfinite(target_position)) or not np.all(np.isfinite(target_xyzw)):
            return
        q_norm = np.linalg.norm(target_xyzw)
        if q_norm < 1e-8:
            return
        target_xyzw = target_xyzw / q_norm
        self._suppress_target_sync = True
        try:
            self.target_pose_gizmo.position = tuple(float(x) for x in target_position)
            self.target_pose_gizmo.wxyz = (
                float(target_xyzw[3]),
                float(target_xyzw[0]),
                float(target_xyzw[1]),
                float(target_xyzw[2]),
            )
        finally:
            self._suppress_target_sync = False

    def _sync_text_from_gizmo(self):
        pos = np.asarray(self.target_pose_gizmo.position, dtype=np.float64)
        wxyz = np.asarray(self.target_pose_gizmo.wxyz, dtype=np.float64)
        if pos.shape != (3,) or wxyz.shape != (4,):
            return
        if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(wxyz)):
            return

        q_norm = np.linalg.norm(wxyz)
        if q_norm < 1e-8:
            return
        wxyz = wxyz / q_norm
        xyzw = np.array([wxyz[1], wxyz[2], wxyz[3], wxyz[0]], dtype=np.float64)

        self._suppress_target_sync = True
        try:
            self.target_x.value = f"{float(pos[0]):.8f}"
            self.target_y.value = f"{float(pos[1]):.8f}"
            self.target_z.value = f"{float(pos[2]):.8f}"
            self.target_qx.value = f"{float(xyzw[0]):.8f}"
            self.target_qy.value = f"{float(xyzw[1]):.8f}"
            self.target_qz.value = f"{float(xyzw[2]):.8f}"
            self.target_qw.value = f"{float(xyzw[3]):.8f}"
        finally:
            self._suppress_target_sync = False

    def _solve_from_input_pose_and_update_ui(self):
        t0 = time.time()
        target_position, target_xyzw = self._get_target_pose()
        if target_position is None or target_xyzw is None:
            self.logger.warning("IK input invalid: pose fields are not valid float values")
            self.result_text.value = "Invalid input: please enter valid float numbers"
            return
        self._update_target_pose_visualization()

        if not np.all(np.isfinite(target_position)) or not np.all(np.isfinite(target_xyzw)):
            self.logger.warning("IK input invalid: target pose has non-finite values")
            self.result_text.value = "Invalid input: non-finite pose values"
            return

        if np.linalg.norm(target_xyzw) < 1e-8:
            self.logger.warning("IK input invalid: quaternion norm is zero")
            self.result_text.value = "Invalid input: quaternion norm is zero"
            return

        target_xyzw = target_xyzw / np.linalg.norm(target_xyzw)
        target_wxyz = np.array(
            [target_xyzw[3], target_xyzw[0], target_xyzw[1], target_xyzw[2]],
            dtype=np.float64,
        )

        cfg_solution, pos_err, ori_err = self._ik_with_validation(
            target_position=target_position,
            target_wxyz=target_wxyz,
        )
        if cfg_solution is None:
            self.logger.warning("IK failed: no solution candidate")
            self.result_text.value = "IK failed: no solution candidate"
            return

        pos_tol = 0.3
        ori_tol = np.deg2rad(90.0)
        if pos_err > pos_tol or ori_err > ori_tol:
            self.logger.warning(
                "IK candidate rejected: pos_err=%.6f m (tol %.6f), ori_err=%.4f rad (tol %.4f)"
                % (pos_err, pos_tol, ori_err, ori_tol)
            )
            self.result_text.value = (
                "IK rejected: pos_err=%.4f m, ori_err=%.3f rad" % (pos_err, ori_err)
            )
            return

        self.cfg = cfg_solution
        self.viser_urdf.update_cfg(self.cfg)
        self.timing.value = 0.9 * self.timing.value + 0.1 * (time.time() - t0) * 1000.0
        self.result_text.value = (
            "IK success: pos_err=%.4f m, ori_err=%.3f rad" % (pos_err, ori_err)
        )
        self.logger.info(
            "IK solved in %.1f ms, pos_err=%.6f m, ori_err=%.4f rad"
            % ((time.time() - t0) * 1000.0, pos_err, ori_err)
        )

    def _fk_ee(self, cfg):
        link_names = list(self.robot.links.names)
        if EE_LINK not in link_names:
            self.logger.error(f"EE link '{EE_LINK}' not in URDF links: {link_names}")
            return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
        idx = link_names.index(EE_LINK)
        Ts = np.asarray(self.robot.forward_kinematics(cfg))
        wxyz = Ts[idx, :4]
        xyz = Ts[idx, 4:]
        return xyz, wxyz


def main():
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    PyrokiIKUnitTest()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
