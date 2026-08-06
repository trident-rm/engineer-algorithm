#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import numpy as np
import viser
from viser.extras import ViserUrdf
import yourdfpy


DEFAULT_INPUT = "output/pk_ik_ri.npz"
URDF_PATH = os.environ.get(
	"ROBO_ARM_URDF",
	"src/arms/combine_urdf/urdf/combine_urdf.urdf",
)


def _make_package_filename_handler():
	search_roots = ["src/arms", "src", "install"]
	pkg_map: dict[str, str] = {}
	for root in search_roots:
		if not os.path.isdir(root):
			continue
		for entry in os.listdir(root):
			full = os.path.join(root, entry)
			if os.path.isdir(full) and entry not in pkg_map:
				pkg_map[entry] = full

	def handler(fname: str) -> str:
		if not fname.startswith("package://"):
			return fname
		rest = fname[len("package://") :]
		pkg, rel = rest.split("/", 1)
		if pkg not in pkg_map:
			raise FileNotFoundError(f"package '{pkg}' not found")
		return os.path.join(pkg_map[pkg], rel)

	return handler


def _parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Visualize pk_ik_ri reachability results with viser.")
	parser.add_argument("--input", type=str, default=DEFAULT_INPUT, help="Path to pk_ik_ri .npz output.")
	parser.add_argument(
		"--port",
		type=int,
		default=int(os.environ.get("PK_IK_RI_VISER_PORT", "8082")),
		help="Viser server port.",
	)
	parser.add_argument("--point-size", type=float, default=0.03, help="Point size for reachable voxels.")
	parser.add_argument(
		"--reachable-alpha",
		type=float,
		default=0.35,
		help="Opacity for reachable voxels when rendered as spheres.",
	)
	parser.add_argument(
		"--unreachable-alpha",
		type=float,
		default=0.18,
		help="Opacity for unreachable voxels.",
	)
	parser.add_argument(
		"--z-slice-tol",
		type=float,
		default=1e-6,
		help="Tolerance used when grouping z layers.",
	)
	return parser.parse_args()


def _viridis_colormap(values: np.ndarray) -> np.ndarray:
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


def _build_unit_icosahedron_mesh() -> tuple[np.ndarray, np.ndarray]:
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


def _legend_html(ri_min: float, ri_max: float) -> str:
	return f"""
<div style=\"font-family: sans-serif; line-height: 1.35; min-width: 260px;\">
    <div style=\"font-weight: 700; margin-bottom: 6px;\">pk_ik_ri legend</div>
	<div style=\"margin-bottom: 6px;\">
		Reachable:
		<span style=\"display:inline-block;width:14px;height:14px;border-radius:50%;background: rgba(94,201,98,0.35); border:1px solid rgba(60,120,60,0.6);vertical-align:middle;\"></span>
		translucent colored spheres
	</div>
    <div style=\"margin-bottom: 6px;\">
        Unreachable:
        <span style=\"display:inline-block;width:14px;height:14px;border-radius:50%;background: rgba(170,170,170,0.35); border:1px solid rgba(120,120,120,0.6);vertical-align:middle;\"></span>
        translucent gray spheres
    </div>
    <div style=\"margin-bottom: 4px;\">Reachable RI color bar:</div>
    <div style=\"height: 14px; border-radius: 7px; border: 1px solid #666; background: linear-gradient(90deg, rgb(68,1,84) 0%, rgb(59,82,139) 25%, rgb(33,145,140) 50%, rgb(94,201,98) 75%, rgb(253,231,37) 100%);\"></div>
    <div style=\"display:flex; justify-content:space-between; margin-top: 2px; font-size: 12px;\">
        <span>{ri_min:.3f}</span><span>{ri_max:.3f}</span>
    </div>
</div>
"""


def main() -> None:
	args = _parse_args()
	input_path = Path(args.input)
	if not input_path.is_file():
		raise FileNotFoundError(f"Input file not found: {input_path}")

	data = np.load(input_path)
	points = np.asarray(data["voxel_points"], dtype=np.float32)
	ri = np.asarray(data["ri_values"], dtype=np.float32)
	bounds = np.asarray(data["bounds"], dtype=np.float32) if "bounds" in data else None
	resolution = float(data["resolution"]) if "resolution" in data else args.point_size

	if points.ndim != 2 or points.shape[1] != 3:
		raise ValueError("voxel_points must have shape (N, 3)")
	if ri.shape[0] != points.shape[0]:
		raise ValueError("ri_values length must match voxel_points")

	server = viser.ViserServer(port=args.port)
	server.scene.add_grid("/ground", width=2.0, height=2.0, cell_size=0.1)
	urdf = yourdfpy.URDF.load(
		URDF_PATH,
		filename_handler=_make_package_filename_handler(),
		build_collision_scene_graph=True,
		load_meshes=True,
		load_collision_meshes=True,
	)
	viser_urdf = ViserUrdf(server, urdf_or_path=urdf)
	joint_limits = viser_urdf.get_actuated_joint_limits()
	zero_cfg = np.zeros(len(joint_limits), dtype=np.float32)
	viser_urdf.update_cfg(zero_cfg)

	gui = server.gui
	gui.add_markdown("## pk_ik_ri viewer")
	stats = gui.add_text("Stats", "", disabled=True)
	legend = gui.add_markdown(_legend_html(float(np.min(ri)), float(np.max(ri))))
	show_robot = gui.add_checkbox("Show robot", initial_value=True)
	show_reachable = gui.add_checkbox("Show reachable", initial_value=True)
	show_unreachable = gui.add_checkbox("Show unreachable", initial_value=True)
	reachable_alpha = gui.add_slider("Reachable alpha", min=0.0, max=1.0, step=0.01, initial_value=float(args.reachable_alpha))
	unreachable_alpha = gui.add_slider("Unreachable alpha", min=0.0, max=1.0, step=0.01, initial_value=float(args.unreachable_alpha))
	show_slice = gui.add_checkbox("Enable z slice", initial_value=False)

	z_values = np.round(points[:, 2] / max(args.z_slice_tol, 1e-9)) * max(args.z_slice_tol, 1e-9)
	unique_z = np.unique(z_values)
	z_min = float(np.min(unique_z)) if unique_z.size else 0.0
	z_max = float(np.max(unique_z)) if unique_z.size else 0.0
	z_low = gui.add_slider("Z min", min=z_min, max=z_max, step=max(resolution, args.z_slice_tol), initial_value=z_min)
	z_high = gui.add_slider("Z max", min=z_min, max=z_max, step=max(resolution, args.z_slice_tol), initial_value=z_max)

	reachable_mask = ri > 0.0
	unreachable_mask = ~reachable_mask
	reachable_values = ri[reachable_mask]
	if reachable_values.size > 0:
		ri_min = float(np.min(reachable_values))
		ri_max = float(np.max(reachable_values))
		normalized = (reachable_values - ri_min) / max(ri_max - ri_min, 1e-8)
		reachable_colors = _viridis_colormap(normalized)
	else:
		ri_min = 0.0
		ri_max = 1.0
		reachable_colors = np.zeros((0, 3), dtype=np.float32)

	unit_vertices, unit_faces = _build_unit_icosahedron_mesh()
	reachable_handle = None
	unreachable_handle = None

	def refresh() -> None:
		nonlocal reachable_handle, unreachable_handle
		if reachable_handle is not None:
			try:
				reachable_handle.remove()
			except Exception:
				pass
			reachable_handle = None
		if unreachable_handle is not None:
			try:
				unreachable_handle.remove()
			except Exception:
				pass
			unreachable_handle = None

		visible_mask = np.ones((points.shape[0],), dtype=bool)
		if show_slice.value:
			z0 = min(float(z_low.value), float(z_high.value))
			z1 = max(float(z_low.value), float(z_high.value))
			visible_mask &= np.logical_and(points[:, 2] >= z0 - args.z_slice_tol, points[:, 2] <= z1 + args.z_slice_tol)

		visible_reachable = np.logical_and(visible_mask, reachable_mask)
		visible_unreachable = np.logical_and(visible_mask, unreachable_mask)

		if show_reachable.value and np.any(visible_reachable):
			idx = np.flatnonzero(visible_reachable)
			color_idx = np.searchsorted(np.flatnonzero(reachable_mask), idx)
			reachable_points = points[idx]
			reachable_handle = server.scene.add_batched_meshes_simple(
				"/ri/reachable",
				unit_vertices,
				unit_faces,
				np.tile(np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), (reachable_points.shape[0], 1)),
				reachable_points,
				batched_scales=np.full((reachable_points.shape[0],), float(args.point_size) * 0.35, dtype=np.float32),
				batched_colors=reachable_colors[color_idx],
					batched_opacities=np.full((reachable_points.shape[0],), float(reachable_alpha.value), dtype=np.float32),
				opacity=1.0,
				material="standard",
				flat_shading=False,
				cast_shadow=False,
				receive_shadow=False,
			)

		if show_unreachable.value and np.any(visible_unreachable):
			unreachable_points = points[visible_unreachable]
			unreachable_handle = server.scene.add_batched_meshes_simple(
				"/ri/unreachable",
				unit_vertices,
				unit_faces,
				np.tile(np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), (unreachable_points.shape[0], 1)),
				unreachable_points,
				batched_scales=np.full((unreachable_points.shape[0],), float(args.point_size) * 0.35, dtype=np.float32),
				batched_colors=(170, 170, 170),
				batched_opacities=np.full((unreachable_points.shape[0],), float(unreachable_alpha.value), dtype=np.float32),
				opacity=1.0,
				material="standard",
				flat_shading=False,
				cast_shadow=False,
				receive_shadow=False,
			)

		visible_count = int(np.sum(visible_mask))
		visible_reach_count = int(np.sum(visible_reachable))
		stats.value = (
			f"voxels={points.shape[0]} | visible={visible_count} | reachable={visible_reach_count} | "
			f"ri_range=[{ri_min:.3f}, {ri_max:.3f}]"
		)

	@show_reachable.on_update
	def _(_event) -> None:
		refresh()

	@show_robot.on_update
	def _(_event) -> None:
		if bool(show_robot.value):
			viser_urdf.update_cfg(zero_cfg)
		else:
			server.scene.reset()
			server.scene.add_grid("/ground", width=2.0, height=2.0, cell_size=0.1)
			refresh()

	@show_unreachable.on_update
	def _(_event) -> None:
		refresh()

	@show_slice.on_update
	def _(_event) -> None:
		refresh()

	@reachable_alpha.on_update
	def _(_event) -> None:
		refresh()

	@unreachable_alpha.on_update
	def _(_event) -> None:
		refresh()

	@z_low.on_update
	def _(_event) -> None:
		refresh()

	@z_high.on_update
	def _(_event) -> None:
		refresh()

	if bounds is not None and bounds.shape[0] == 6:
		center = 0.5 * (bounds[:3] + bounds[3:])
		server.scene.add_frame(
			"/workspace_center",
			position=tuple(center.astype(np.float32).tolist()),
			wxyz=(1.0, 0.0, 0.0, 0.0),
			axes_length=max(0.05, resolution * 1.5),
			axes_radius=max(0.003, resolution * 0.08),
		)

	refresh()
	print(f"viser server running at http://localhost:{args.port}")
	while True:
		time.sleep(1.0)


if __name__ == "__main__":
	main()
