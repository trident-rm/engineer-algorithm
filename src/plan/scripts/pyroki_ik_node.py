#!/usr/bin/env python3
"""
pyroki_ik_node.py
- 启动 viser 网页（默认 http://localhost:8081），可拖拽末端 gizmo
- 用 pyroki 求 IK
- 发布到 /controller_display_robot_state 供 flow_with_pyroki 当作“控制器参考状态”
  (flow_with_pyroki 再决定要不要把它转发到 /joint_states)
"""

import argparse
import os
import sys
import time
import numpy as np


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


_patch_numpy_asarray_copy_kwarg()

# pyroki_snippets 在 pyroki 仓库的 examples/ 下, 不是 pip 包
sys.path.insert(0, "/home/hku/third_party/pyroki/examples")


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayRobotState, MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetPositionIK

import pyroki as pk
import pyroki_snippets as pks
import viser
from viser.extras import ViserUrdf
import yourdfpy

URDF_PATH = os.environ.get(
    "ROBO_ARM_URDF",
    "/home/hku/robo_arm/src/arms/combine_urdf/urdf/combine_urdf.urdf",
)
EE_LINK = os.environ.get("ROBO_ARM_EE_LINK", "gripper")
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def _parse_args():
    parser = argparse.ArgumentParser(description="Run pyroki IK ROS node with a viser control UI.")
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("PYROKI_IK_VISER_PORT", "8081")),
        help="Viser server port. Defaults to 8081 to avoid pyroki_workspace_visualizer's 8080.",
    )
    args, ros_args = parser.parse_known_args()
    return args, [sys.argv[0], *ros_args]


def _make_package_filename_handler():
    """让 yourdfpy 能解析 URDF 里的 package://xxx/... 路径。"""
    search_roots = [
        "/home/hku/robo_arm/src/arms/combine_urdf",
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
        # yourdfpy 用 fname= 作为 keyword argument
        assert fname.startswith("package://"), fname
        rest = fname[len("package://"):]
        pkg, rel = rest.split("/", 1)
        if pkg not in pkg_map:
            raise FileNotFoundError(f"package '{pkg}' not found in {search_roots}")
        return os.path.join(pkg_map[pkg], rel)

    return handler


class PyrokiIKNode(Node):
    def __init__(self, port: int):
        super().__init__("pyroki_ik_node")
        self.pub_js = self.create_publisher(JointState, "/pyroki_joint_states", 10)
        self.pub_drs = self.create_publisher(DisplayRobotState, "/pyroki_controller", 10)
        self.ik_srv = self.create_service(
            GetPositionIK,
            "/pyroki_get_position_ik",
            self._handle_get_position_ik,
        )
        self.get_logger().info(f"Loading URDF: {URDF_PATH}")

        self.urdf = yourdfpy.URDF.load(
            URDF_PATH,
            filename_handler=_make_package_filename_handler(),
            build_collision_scene_graph=False,
            load_meshes=True,
            load_collision_meshes=False,
        )
        self.robot = pk.Robot.from_urdf(self.urdf)

        # pyroki 内部使用的 actuated joint 顺序
        self.pk_joint_names = list(self.robot.joints.actuated_names)
        self.get_logger().info(f"pyroki actuated joints: {self.pk_joint_names}")

        # 初始 cfg：全 0
        #self.cfg = np.zeros(len(self.robot.joints.actuated_names))
        self.cfg = np.array([0.0, -0.3, 0.0, 0.0, 0.0, 0.0])
        # 用于控制打印频率（每5秒打印一次）
        self.last_print_time = time.time()

        # 用 FK 算出 gripper 当前位姿, 后面用来初始化 gizmo
        ee_pos, ee_wxyz = self._fk_ee(self.cfg)
        self.get_logger().info(
            f"initial EE pose: pos={ee_pos.tolist()}, wxyz={ee_wxyz.tolist()}"
        )

        # ---- viser 网页端 ----
        self.server = viser.ViserServer(port=port)
        self.server.scene.add_grid("/ground", width=2, height=2)
        self.viser_urdf = ViserUrdf(self.server, urdf_or_path=self.urdf)
        # 立刻把默认姿态推给网页, 否则会显示成 zero pose 看起来散架
        self.viser_urdf.update_cfg(self.cfg)

        # 一个可拖拽的目标 gizmo, 初始就放在 gripper 当前位置
        self.target = self.server.scene.add_transform_controls(
            "/ik_target",
            scale=0.15,
            position=tuple(float(x) for x in ee_pos),
            wxyz=tuple(float(x) for x in ee_wxyz),
        )
        self.timing = self.server.gui.add_number("IK time (ms)", 0.0, disabled=True)

        @self.target.on_update
        def _(_):
            self._solve_and_publish()

        # 启动后只发布初始值，不立刻求解
        # self._solve_and_publish()  # 注释掉，保持全 0 初始
        self._publish_current()

        # 兜底定时重发, 防止 RViz/MoveIt 丢消息
        self.create_timer(0.1, self._publish_current)

        self.get_logger().info(
            f"viser server running, open the URL above. EE link = {EE_LINK}"
        )

    def _cfg_from_joint_state(self, joint_state: JointState):
        if not joint_state.name or not joint_state.position:
            return None
        name_to_pos = {
            str(name): float(pos)
            for name, pos in zip(joint_state.name, joint_state.position)
        }
        cfg = np.array(self.cfg, dtype=np.float64, copy=True)
        for i, pk_name in enumerate(self.pk_joint_names):
            if pk_name in name_to_pos:
                cfg[i] = name_to_pos[pk_name]
        return cfg

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

    def _ik_with_validation(self, target_position: np.ndarray, target_wxyz: np.ndarray, request_seed: np.ndarray | None):
        pos_tol = 0.003
        ori_tol = np.deg2rad(3.0)

        base_seed = np.array(self.cfg, dtype=np.float64, copy=True)
        if request_seed is not None and request_seed.shape == base_seed.shape:
            base_seed = request_seed

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
                # Backward compatibility: older pyroki_snippets may not support init_config.
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

            self.get_logger().debug(
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

    def _handle_get_position_ik(self, request: GetPositionIK.Request, response: GetPositionIK.Response):
        pose = request.ik_request.pose_stamped.pose
        t0 = time.time()
        self.get_logger().info(
            "IK service request: pos=(%.4f, %.4f, %.4f) frame=%s"
            % (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                request.ik_request.pose_stamped.header.frame_id,
            )
        )
        target_position = np.array(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=np.float64,
        )
        target_xyzw = np.array(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            dtype=np.float64,
        )

        if not np.all(np.isfinite(target_position)) or not np.all(np.isfinite(target_xyzw)):
            response.error_code.val = MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS
            return response

        if np.linalg.norm(target_xyzw) < 1e-8:
            response.error_code.val = MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS
            return response

        target_xyzw = target_xyzw / np.linalg.norm(target_xyzw)
        target_wxyz = np.array(
            [target_xyzw[3], target_xyzw[0], target_xyzw[1], target_xyzw[2]],
            dtype=np.float64,
        )

        request_seed = self._cfg_from_joint_state(request.ik_request.robot_state.joint_state)
        cfg_solution, pos_err, ori_err = self._ik_with_validation(
            target_position=target_position,
            target_wxyz=target_wxyz,
            request_seed=request_seed,
        )
        if cfg_solution is None:
            self.get_logger().warn("IK service failed: no solution candidate")
            response.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION
            return response

        pos_tol = 0.3
        ori_tol = np.deg2rad(90.0)
        if pos_err > pos_tol or ori_err > ori_tol:
            self.get_logger().warn(
                "IK candidate rejected: pos_err=%.6f m (tol %.6f), ori_err=%.4f rad (tol %.4f)"
                % (pos_err, pos_tol, ori_err, ori_tol)
            )
            response.error_code.val = MoveItErrorCodes.NO_IK_SOLUTION
            return response

        positions = []
        for name in JOINT_NAMES:
            if name in self.pk_joint_names:
                positions.append(float(cfg_solution[self.pk_joint_names.index(name)]))
            else:
                positions.append(0.0)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = positions

        response.solution = RobotState()
        response.solution.joint_state = js
        response.error_code.val = MoveItErrorCodes.SUCCESS
        self.get_logger().info(
            "IK service solved in %.1f ms, pos_err=%.6f m, ori_err=%.4f rad"
            % ((time.time() - t0) * 1000.0, pos_err, ori_err)
        )
        return response

    def _solve_and_publish(self):
        try:
            t0 = time.time()
            self.cfg = pks.solve_ik(
                robot=self.robot,
                target_link_name=EE_LINK,
                target_position=np.array(self.target.position, dtype=np.float64),
                target_wxyz=np.array(self.target.wxyz, dtype=np.float64),
            )
            self.timing.value = 0.9 * self.timing.value + 0.1 * (time.time() - t0) * 1000
            self.viser_urdf.update_cfg(self.cfg)
        except Exception as e:
            self.get_logger().warn(f"IK failed: {e}")
            return
        self._publish_current()

    def _publish_current(self):
        # cfg 是按 pyroki 内部 actuated_names 顺序的
        positions = []
        for name in JOINT_NAMES:
            if name in self.pk_joint_names:
                positions.append(float(self.cfg[self.pk_joint_names.index(name)]))
            else:
                positions.append(0.0)

        # 发布 JointState 到 /pyroki_joint_states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = positions
        self.pub_js.publish(msg)

        # 发布 DisplayRobotState 到 /pyroki_controller
        drs = DisplayRobotState()
        drs.state = RobotState()
        drs.state.joint_state = msg
        self.pub_drs.publish(drs)
        
        # 每5秒打印一次末端位置
        current_time = time.time()
        if current_time - self.last_print_time >= 5.0:
            ee_pos, ee_wxyz = self._fk_ee(self.cfg)
            self.get_logger().info(
                f"[pyroki] EE Pose: pos=({ee_pos[0]:.4f}, {ee_pos[1]:.4f}, {ee_pos[2]:.4f}) "
                f"orient=({ee_wxyz[0]:.4f}, {ee_wxyz[1]:.4f}, {ee_wxyz[2]:.4f}, {ee_wxyz[3]:.4f})"
            )
            self.last_print_time = current_time

    def _fk_ee(self, cfg):
        """正运动学: 返回 EE_LINK 在 base frame 下的 (xyz, wxyz)."""
        link_names = list(self.robot.links.names)
        if EE_LINK not in link_names:
            self.get_logger().error(
                f"EE link '{EE_LINK}' not in URDF links: {link_names}"
            )
            return np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0])
        idx = link_names.index(EE_LINK)
        # pyroki: forward_kinematics(cfg) -> (N_links, 7), 每行是 [wxyz, xyz]
        Ts = np.asarray(self.robot.forward_kinematics(cfg))
        wxyz = Ts[idx, :4]
        xyz = Ts[idx, 4:]
        return xyz, wxyz


def main():
    args, ros_args = _parse_args()
    rclpy.init(args=ros_args)
    node = PyrokiIKNode(port=args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
