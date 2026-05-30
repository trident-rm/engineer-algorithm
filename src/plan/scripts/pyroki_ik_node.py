#!/usr/bin/env python3
"""
pyroki_ik_node.py
- 启动 viser 网页（默认 http://localhost:8080），可拖拽末端 gizmo
- 用 pyroki 求 IK
- 发布到 /controller_display_robot_state 供 flow_with_pyroki 当作“控制器参考状态”
  (flow_with_pyroki 再决定要不要把它转发到 /joint_states)
"""

import os
import sys
import time
import numpy as np

# pyroki_snippets 在 pyroki 仓库的 examples/ 下, 不是 pip 包
sys.path.insert(0, "/home/hku/third_party/pyroki/examples")


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayRobotState, RobotState

import pyroki as pk
import pyroki_snippets as pks
import viser
from viser.extras import ViserUrdf
import yourdfpy

URDF_PATH = os.environ.get(
    "ROBO_ARM_URDF",
    "/home/hku/robo_arm/src/arms/robo_arm10/urdf/robo_arm10.urdf",
)
EE_LINK = os.environ.get("ROBO_ARM_EE_LINK", "gripper")
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def _make_package_filename_handler():
    """让 yourdfpy 能解析 URDF 里的 package://xxx/... 路径。"""
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
        # yourdfpy 用 fname= 作为 keyword argument
        assert fname.startswith("package://"), fname
        rest = fname[len("package://"):]
        pkg, rel = rest.split("/", 1)
        if pkg not in pkg_map:
            raise FileNotFoundError(f"package '{pkg}' not found in {search_roots}")
        return os.path.join(pkg_map[pkg], rel)

    return handler


class PyrokiIKNode(Node):
    def __init__(self):
        super().__init__("pyroki_ik_node")
        self.pub_js = self.create_publisher(JointState, "/pyroki_joint_states", 10)
        self.pub_drs = self.create_publisher(DisplayRobotState, "/pyroki_controller", 10)
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
        self.cfg = np.array([0.5, -0.5, 0.3, 0, 1.0, 0])
        # 用于控制打印频率（每5秒打印一次）
        self.last_print_time = time.time()

        # 用 FK 算出 gripper 当前位姿, 后面用来初始化 gizmo
        ee_pos, ee_wxyz = self._fk_ee(self.cfg)
        self.get_logger().info(
            f"initial EE pose: pos={ee_pos.tolist()}, wxyz={ee_wxyz.tolist()}"
        )

        # ---- viser 网页端 ----
        self.server = viser.ViserServer()
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
    rclpy.init()
    node = PyrokiIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
