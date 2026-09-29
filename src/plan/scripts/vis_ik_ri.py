#!/usr/bin/env python3

from __future__ import annotations

import argparse
from dataclasses import dataclass
import importlib
import logging
import os
import sys
import time
from pathlib import Path
from typing import Any

os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")

import jax
import jax.numpy as jnp
import jaxlie
import jaxls
import numpy as np
import pyroki as pk
import trimesh
import viser
from viser.extras import ViserUrdf
import yourdfpy


DEFAULT_INPUT = "output/pk_ik_ri.npz"
DEFAULT_BOUNDS = (-1.0, 1.0, -1.0, 1.0, -0.5, 1.0)
DEFAULT_RESOLUTION = 0.05
URDF_PATH = os.environ.get(
	"ROBO_ARM_URDF",
	"src/arms/combine_urdf/urdf/combine_urdf.urdf",
)
GROUND_MESH_PATH = "src/arms/reduced_ground/meshes/ground_link.STL"
PILLAR_MESH_PATH = "src/arms/reduced_pillar/meshes/pillar_link.STL"
WORLD_TO_CHASSIS_POSITION = (-0.1, 0.0, 0.142)
# flow_with_pyroki's positional arguments use yaw, pitch, roll.
WORLD_TO_CHASSIS_WXYZ = (0.0, 0.0, 0.0, 1.0)
WORLD_TO_GROUND_POSITION = (0.0, 0.0, 0.0)
WORLD_TO_PILLAR_POSITION = (-0.8, 0.0, 0.55)
EE_LINK = os.environ.get("ROBO_ARM_EE_LINK", "gripper")
RI_FRAME = os.environ.get("ROBO_ARM_RI_FRAME", "chassis_link")
LOGGER = logging.getLogger(__name__)


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


def _import_pyroki_snippets() -> Any:
	try:
		return importlib.import_module("pyroki_snippets")
	except ModuleNotFoundError:
		candidate_paths: list[str] = []
		env_path = os.environ.get("PYROKI_EXAMPLES_PATH")
		if env_path:
			candidate_paths.append(env_path)
		candidate_paths.append("/home/hku/third_party/pyroki/examples")

		for path in candidate_paths:
			if path and os.path.isdir(path) and path not in sys.path:
				sys.path.insert(0, path)
			try:
				return importlib.import_module("pyroki_snippets")
			except ModuleNotFoundError:
				continue

	raise ModuleNotFoundError(
		"Cannot import 'pyroki_snippets'. Set PYROKI_EXAMPLES_PATH to the PyRoKi examples directory."
	)


@dataclass(frozen=True)
class ScanSpec:
	bounds: tuple[float, float, float, float, float, float]
	resolution: float
	ori_samples: int
	pos_tol: float
	ori_tol_rad: float
	batch_size: int
	seed_count: int
	refine_count: int


class BatchedIKReachability:
	def __init__(self, robot: pk.Robot, ee_link: str, spec: ScanSpec):
		self.robot = robot
		self.spec = spec
		self.target_link_index = jnp.array(robot.links.names.index(ee_link))
		self.num_joints = int(robot.joints.num_actuated_joints)
		self.lower_limits = jnp.asarray(robot.joints.lower_limits)
		self.upper_limits = jnp.asarray(robot.joints.upper_limits)
		self.seed_count = max(1, int(spec.seed_count))
		self.refine_count = max(1, min(int(spec.refine_count), self.seed_count))
		self.orientation_quats = self._build_orientation_quats(spec.ori_samples)
		self.direction_vectors = self._quaternions_z_axes(np.asarray(self.orientation_quats))
		self._batched_solve = jax.jit(jax.vmap(self._solve_pose, in_axes=(0, 0)))
		self._batched_fk = jax.jit(jax.vmap(self._forward_kinematics))

	def _iter_chunks(self, total_count: int):
		batch_size = max(1, int(self.spec.batch_size))
		for start in range(0, total_count, batch_size):
			yield start, min(total_count, start + batch_size)

	def _build_orientation_quats(self, sample_count: int) -> jax.Array:
		rng = np.random.default_rng(0)
		pool = rng.normal(size=(max(1024, sample_count * 64), 4)).astype(np.float32)
		pool /= np.maximum(np.linalg.norm(pool, axis=1, keepdims=True), 1e-8)
		selected = np.empty((sample_count, 4), dtype=np.float32)
		selected[0] = pool[0]
		min_dist = np.arccos(np.clip(np.abs(pool @ selected[0]), 0.0, 1.0))
		for index in range(1, sample_count):
			next_index = int(np.argmax(min_dist))
			selected[index] = pool[next_index]
			min_dist = np.minimum(min_dist, np.arccos(np.clip(np.abs(pool @ selected[index]), 0.0, 1.0)))
		return jnp.asarray(selected)

	@staticmethod
	def _quaternions_z_axes(wxyz: np.ndarray) -> np.ndarray:
		"""Return each orientation's local +Z axis in the RI root frame."""
		w, x, y, z = np.asarray(wxyz, dtype=np.float32).T
		return np.column_stack(
			(
				2.0 * (x * z + y * w),
				2.0 * (y * z - x * w),
				1.0 - 2.0 * (x * x + y * y),
			)
		).astype(np.float32)

	def _initial_seeds(self) -> jax.Array:
		rng = jax.random.PRNGKey(0)
		samples = jax.random.uniform(
			rng,
			shape=(self.seed_count, self.num_joints),
			minval=0.0,
			maxval=1.0,
		)
		return self.lower_limits + samples * (self.upper_limits - self.lower_limits)

	def _solve_once(
		self,
		target_wxyz: jax.Array,
		target_position: jax.Array,
		initial_q: jax.Array,
		lambda_initial: jax.Array,
		max_iters: int,
	) -> tuple[jax.Array, jaxls.SolveSummary]:
		joint_var = self.robot.joint_var_cls(0)
		costs = [
			pk.costs.pose_cost_analytic_jac(
				self.robot,
				joint_var,
				jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(target_wxyz), target_position),
				self.target_link_index,
				pos_weight=50.0,
				ori_weight=10.0,
			),
			pk.costs.limit_cost(self.robot, joint_var, weight=50.0),
		]
		solution, summary = (
			jaxls.LeastSquaresProblem(costs=costs, variables=[joint_var])
			.analyze()
			.solve(
				initial_vals=jaxls.VarValues.make([joint_var.with_value(initial_q)]),
				verbose=False,
				linear_solver="dense_cholesky",
				trust_region=jaxls.TrustRegionConfig(lambda_initial=lambda_initial),
				termination=jaxls.TerminationConfig(max_iterations=max_iters, early_termination=False),
				return_summary=True,
			)
		)
		return solution[joint_var], summary

	def _solve_pose(self, target_wxyz: jax.Array, target_position: jax.Array) -> jax.Array:
		seeds = self._initial_seeds()
		vmapped = jax.vmap(self._solve_once, in_axes=(None, None, 0, 0, None))
		coarse_sol, coarse_summary = vmapped(
			target_wxyz,
			target_position,
			seeds,
			jnp.full((self.seed_count,), 10.0),
			8,
		)
		coarse_final_cost = coarse_summary.cost_history[jnp.arange(self.seed_count), coarse_summary.iterations]
		best_indices = jnp.argsort(coarse_final_cost)[: self.refine_count]
		refined_sol, refined_summary = vmapped(
			target_wxyz,
			target_position,
			coarse_sol[best_indices],
			coarse_summary.lambda_history[jnp.arange(self.seed_count), coarse_summary.iterations][best_indices],
			16,
		)
		final_index = jnp.argmin(refined_summary.cost_history[jnp.arange(self.refine_count), refined_summary.iterations])
		return refined_sol[final_index]

	def _forward_kinematics(self, q: jax.Array) -> jax.Array:
		return self.robot.forward_kinematics(q)[self.target_link_index]

	def build_grid(self) -> np.ndarray:
		xmin, xmax, ymin, ymax, zmin, zmax = self.spec.bounds
		step = self.spec.resolution
		xs = np.arange(xmin + 0.5 * step, xmax - 0.5 * step + 1e-12, step, dtype=np.float32)
		ys = np.arange(ymin + 0.5 * step, ymax - 0.5 * step + 1e-12, step, dtype=np.float32)
		zs = np.arange(zmin + 0.5 * step, zmax - 0.5 * step + 1e-12, step, dtype=np.float32)
		return np.stack(np.meshgrid(xs, ys, zs, indexing="ij"), axis=-1).reshape(-1, 3)

	def evaluate(self) -> dict[str, np.ndarray]:
		points = self.build_grid()
		point_count = points.shape[0]
		ori_count = int(self.orientation_quats.shape[0])
		reach_counts = np.zeros((point_count,), dtype=np.int32)
		direction_success_mask = np.zeros((point_count, ori_count), dtype=bool)
		points_jnp = jnp.asarray(points)

		for start, end in self._iter_chunks(point_count):
			chunk_points = points_jnp[start:end]
			chunk_count = end - start
			tiled_positions = jnp.repeat(chunk_points, ori_count, axis=0)
			tiled_wxyz = jnp.tile(self.orientation_quats, (chunk_count, 1))
			solutions = self._batched_solve(tiled_wxyz, tiled_positions)
			fk = self._batched_fk(solutions)
			pos_err = jnp.linalg.norm(fk[:, 4:7] - tiled_positions, axis=1)
			ori_err = jnp.linalg.norm(
				(jaxlie.SO3(tiled_wxyz).inverse() @ jaxlie.SO3(fk[:, :4])).log(), axis=1
			)
			success = jnp.logical_and(pos_err <= self.spec.pos_tol, ori_err <= self.spec.ori_tol_rad)
			success_matrix = np.asarray(success.reshape(chunk_count, ori_count), dtype=bool)
			direction_success_mask[start:end] = success_matrix
			reach_counts[start:end] = np.sum(success_matrix, axis=1, dtype=np.int32)
			LOGGER.info("Processed points %d to %d / %d", start, end, point_count)

		return {
			"points": points,
			"ri": reach_counts.astype(np.float32) / float(max(1, ori_count)),
			"reach_counts": reach_counts,
			"orientation_quats": np.asarray(self.orientation_quats),
			"direction_vectors": self.direction_vectors,
			"direction_success_mask": direction_success_mask,
			**_fit_capability_shapes(direction_success_mask, self.direction_vectors),
		}


def _fit_capability_shapes(direction_success_mask: np.ndarray, direction_vectors: np.ndarray) -> dict[str, np.ndarray]:
	"""Fit the Zacharias cone, double-cone, and ring-cylinder primitives."""
	voxel_count, direction_count = direction_success_mask.shape
	shape_types = np.full((voxel_count,), "none", dtype="<U14")
	shape_axes = np.tile(np.array([[0.0, 0.0, 1.0]], dtype=np.float32), (voxel_count, 1))
	shape_angles = np.zeros((voxel_count,), dtype=np.float32)
	shape_sfe = np.full((voxel_count,), 100.0, dtype=np.float32)

	for voxel_index, reachable in enumerate(direction_success_mask):
		reachable_count = int(np.count_nonzero(reachable))
		if reachable_count == 0:
			continue
		reachable_directions = direction_vectors[reachable]
		mean_axis = np.sum(reachable_directions, axis=0)
		if np.linalg.norm(mean_axis) < 1e-8:
			covariance = reachable_directions.T @ reachable_directions
			mean_axis = np.linalg.eigh(covariance)[1][:, -1]
		mean_axis /= np.linalg.norm(mean_axis)
		covariance = reachable_directions.T @ reachable_directions
		cylinder_axis = np.linalg.eigh(covariance)[1][:, -1]

		best = (float("inf"), "none", mean_axis, 0.0)
		for shape_type, axis in (("cone", mean_axis), ("double_cone", cylinder_axis), ("ring_cylinder", cylinder_axis)):
			axis = axis / np.linalg.norm(axis)
			dots = direction_vectors @ axis
			if shape_type == "cone":
				angles = np.arccos(np.clip(dots, -1.0, 1.0))
				membership = lambda threshold: angles <= threshold
			elif shape_type == "double_cone":
				angles = np.arccos(np.clip(np.abs(dots), -1.0, 1.0))
				membership = lambda threshold: angles <= threshold
			else:
				angles = np.arcsin(np.clip(np.abs(dots), 0.0, 1.0))
				membership = lambda threshold: angles <= threshold
			for angle in np.unique(angles):
				inside = membership(angle)
				missed_reachable = int(np.count_nonzero(reachable & ~inside))
				false_reachable = int(np.count_nonzero(~reachable & inside))
				error = missed_reachable + false_reachable
				if error < best[0]:
					best = (error, shape_type, axis, float(angle))

		error, shape_type, axis, angle = best
		shape_types[voxel_index] = shape_type
		shape_axes[voxel_index] = axis
		shape_angles[voxel_index] = angle
		shape_sfe[voxel_index] = min(100.0, 100.0 * float(error) / float(reachable_count))

	return {
		"shape_type": shape_types,
		"shape_axis": shape_axes,
		"shape_opening_angle_rad": shape_angles,
		"shape_sfe": shape_sfe,
	}


def _parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Compute and visualize a PyRoKi reachability index.")
	parser.add_argument(
		"--input",
		"--input-npz",
		dest="input",
		type=str,
		default=None,
		help="Load an existing RI .npz instead of calculating one.",
	)
	parser.add_argument("--output", type=str, default=DEFAULT_INPUT, help="Path for a newly calculated RI .npz.")
	parser.add_argument("--no-view", action="store_true", help="Calculate and save without starting Viser.")
	parser.add_argument("--bounds", nargs=6, type=float, default=list(DEFAULT_BOUNDS))
	parser.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION)
	parser.add_argument("--ori-samples", type=int, default=64)
	parser.add_argument("--ik-pos-tol", type=float, default=0.005)
	parser.add_argument("--ik-ori-tol-deg", type=float, default=3.0)
	parser.add_argument("--batch-size", type=int, default=128)
	parser.add_argument("--seed-count", type=int, default=8)
	parser.add_argument("--refine-count", type=int, default=2)
	parser.add_argument(
		"--port",
		type=int,
		default=int(os.environ.get("PK_IK_RI_VISER_PORT", "8082")),
		help="Viser server port.",
	)
	parser.add_argument("--point-size", type=float, default=0.1, help="Point size for reachable voxels.")
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


def _paper_ri_colormap(values: np.ndarray) -> np.ndarray:
	v = np.clip(np.asarray(values, dtype=np.float32), 0.0, 1.0)
	anchors = np.array([0.0, 0.25, 0.5, 0.75, 1.0], dtype=np.float32)
	rgb_anchors = np.array(
		[
			[0.85, 0.05, 0.05],
			[0.95, 0.85, 0.05],
			[0.10, 0.75, 0.10],
			[0.05, 0.75, 0.85],
			[0.05, 0.15, 0.90],
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


def _load_stl_mesh(path: str) -> trimesh.Trimesh:
	mesh = trimesh.load_mesh(path, process=False)
	if not isinstance(mesh, trimesh.Trimesh):
		raise TypeError(f"Expected a triangle mesh at {path}, got {type(mesh).__name__}")
	return mesh


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
	<div style=\"margin-bottom: 4px;\">RI color bar: low reachability to high reachability</div>
	<div style=\"height: 14px; border-radius: 7px; border: 1px solid #666; background: linear-gradient(90deg, rgb(217,13,13) 0%, rgb(242,217,13) 25%, rgb(26,191,26) 50%, rgb(13,191,217) 75%, rgb(13,38,230) 100%);\"></div>
    <div style=\"display:flex; justify-content:space-between; margin-top: 2px; font-size: 12px;\">
        <span>{ri_min:.3f}</span><span>{ri_max:.3f}</span>
    </div>
</div>
"""


def _calculate_scan(args: argparse.Namespace) -> dict[str, np.ndarray]:
	_import_pyroki_snippets()
	urdf = yourdfpy.URDF.load(
		URDF_PATH,
		filename_handler=_make_package_filename_handler(),
		build_collision_scene_graph=True,
		load_meshes=True,
		load_collision_meshes=True,
	)
	robot = pk.Robot.from_urdf(urdf)
	root_frame = robot.links.names[0]
	if RI_FRAME != root_frame:
		raise ValueError(
			f"ROBO_ARM_RI_FRAME must be the PyRoKi URDF root '{root_frame}', got '{RI_FRAME}'"
		)
	if EE_LINK not in robot.links.names:
		raise ValueError(f"End-effector link '{EE_LINK}' not found in URDF links: {list(robot.links.names)}")

	spec = ScanSpec(
		bounds=tuple(float(value) for value in args.bounds),
		resolution=float(args.resolution),
		ori_samples=int(args.ori_samples),
		pos_tol=float(args.ik_pos_tol),
		ori_tol_rad=float(np.deg2rad(args.ik_ori_tol_deg)),
		batch_size=int(args.batch_size),
		seed_count=int(args.seed_count),
		refine_count=int(args.refine_count),
	)
	t0 = time.time()
	result = BatchedIKReachability(robot=robot, ee_link=EE_LINK, spec=spec).evaluate()
	LOGGER.info("Evaluation completed in %.2f seconds.", time.time() - t0)

	output_path = Path(args.output)
	output_path.parent.mkdir(parents=True, exist_ok=True)
	np.savez_compressed(
		output_path,
		voxel_points=result["points"],
		ri_values=result["ri"],
		pose_success_counts=result["reach_counts"],
		orientation_quats=result["orientation_quats"],
		direction_vectors=result["direction_vectors"],
		direction_success_mask=result["direction_success_mask"],
		shape_type=result["shape_type"],
		shape_axis=result["shape_axis"],
		shape_opening_angle_rad=result["shape_opening_angle_rad"],
		shape_sfe=result["shape_sfe"],
		bounds=np.asarray(spec.bounds, dtype=np.float32),
		frame_id=np.asarray(RI_FRAME),
		resolution=np.float32(spec.resolution),
		ori_samples=np.int32(spec.ori_samples),
	)
	LOGGER.info("Saved reachability index to %s", output_path)
	return {
		"voxel_points": result["points"],
		"ri_values": result["ri"],
		"bounds": np.asarray(spec.bounds, dtype=np.float32),
		"resolution": np.float32(spec.resolution),
		"frame_id": np.asarray(RI_FRAME),
		"direction_vectors": result["direction_vectors"],
		"direction_success_mask": result["direction_success_mask"],
		"shape_type": result["shape_type"],
		"shape_axis": result["shape_axis"],
		"shape_opening_angle_rad": result["shape_opening_angle_rad"],
		"shape_sfe": result["shape_sfe"],
	}


def _load_scan(input_path: Path) -> dict[str, np.ndarray]:
	if not input_path.is_file():
		raise FileNotFoundError(f"Input file not found: {input_path}")
	with np.load(input_path) as data:
		if "voxel_points" not in data or "ri_values" not in data:
			raise KeyError(f"RI file {input_path} must contain voxel_points and ri_values")
		return {
			"voxel_points": np.asarray(data["voxel_points"], dtype=np.float32),
			"ri_values": np.asarray(data["ri_values"], dtype=np.float32),
			"bounds": np.asarray(data["bounds"], dtype=np.float32) if "bounds" in data else None,
			"resolution": np.float32(data["resolution"]) if "resolution" in data else None,
			"frame_id": np.asarray(data["frame_id"]) if "frame_id" in data else None,
			"direction_vectors": np.asarray(data["direction_vectors"], dtype=np.float32) if "direction_vectors" in data else None,
			"direction_success_mask": np.asarray(data["direction_success_mask"], dtype=bool) if "direction_success_mask" in data else None,
			"shape_type": np.asarray(data["shape_type"]) if "shape_type" in data else None,
			"shape_axis": np.asarray(data["shape_axis"], dtype=np.float32) if "shape_axis" in data else None,
			"shape_opening_angle_rad": np.asarray(data["shape_opening_angle_rad"], dtype=np.float32) if "shape_opening_angle_rad" in data else None,
			"shape_sfe": np.asarray(data["shape_sfe"], dtype=np.float32) if "shape_sfe" in data else None,
		}


def main() -> None:
	logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")
	args = _parse_args()
	bundle = _load_scan(Path(args.input)) if args.input else _calculate_scan(args)
	if args.no_view:
		return

	frame_id = bundle["frame_id"]
	if frame_id is not None and str(frame_id.item()) != RI_FRAME:
		raise ValueError(f"RI file is in frame '{frame_id.item()}', but this viewer expects '{RI_FRAME}'")
	points = bundle["voxel_points"]
	ri = bundle["ri_values"]
	bounds = bundle["bounds"]
	resolution = float(bundle["resolution"]) if bundle["resolution"] is not None else args.point_size
	direction_vectors = bundle["direction_vectors"]
	direction_success_mask = bundle["direction_success_mask"]
	shape_type = bundle["shape_type"]
	shape_axis = bundle["shape_axis"]
	shape_opening_angle_rad = bundle["shape_opening_angle_rad"]
	shape_sfe = bundle["shape_sfe"]

	if points.ndim != 2 or points.shape[1] != 3:
		raise ValueError("voxel_points must have shape (N, 3)")
	if ri.shape[0] != points.shape[0]:
		raise ValueError("ri_values length must match voxel_points")
	if direction_success_mask is not None and direction_success_mask.shape != (points.shape[0], direction_vectors.shape[0]):
		raise ValueError("direction_success_mask shape must be (voxel_count, direction_count)")

	server = viser.ViserServer(port=args.port)
	server.scene.add_frame("/world", show_axes=False)
	robot_frame = server.scene.add_frame(
		"/world/chassis_link",
		position=WORLD_TO_CHASSIS_POSITION,
		wxyz=WORLD_TO_CHASSIS_WXYZ,
		show_axes=False,
	)
	ground_frame = server.scene.add_frame(
		"/world/reduced_ground_frame",
		position=WORLD_TO_GROUND_POSITION,
		show_axes=False,
	)
	pillar_frame = server.scene.add_frame(
		"/world/reduced_pillar_frame",
		position=WORLD_TO_PILLAR_POSITION,
		show_axes=False,
	)
	server.scene.add_grid("/world/grid", width=2.0, height=2.0, cell_size=0.1)
	server.scene.add_mesh_trimesh("/world/reduced_ground_frame/mesh", _load_stl_mesh(GROUND_MESH_PATH))
	server.scene.add_mesh_trimesh("/world/reduced_pillar_frame/mesh", _load_stl_mesh(PILLAR_MESH_PATH))
	urdf = yourdfpy.URDF.load(
		URDF_PATH,
		filename_handler=_make_package_filename_handler(),
		build_collision_scene_graph=True,
		load_meshes=True,
		load_collision_meshes=True,
	)
	viser_urdf = ViserUrdf(server, urdf_or_path=urdf, root_node_name="/world/chassis_link/robot")
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
	show_plane_cut = gui.add_checkbox("Enable plane cut", initial_value=False)
	plane_normal_x = gui.add_number("Plane normal X", initial_value=0.0, step=0.1)
	plane_normal_y = gui.add_number("Plane normal Y", initial_value=0.0, step=0.1)
	plane_normal_z = gui.add_number("Plane normal Z", initial_value=1.0, step=0.1)
	plane_offset = gui.add_number("Plane offset (m)", initial_value=0.0, step=max(0.001, resolution))
	plane_thickness = gui.add_number("Plane thickness (m)", initial_value=max(resolution, 0.01), step=max(0.001, resolution))
	enable_direction_filter = gui.add_checkbox("Filter by approach direction", initial_value=False)
	direction_x = gui.add_number("Approach direction X", initial_value=0.0, step=0.1)
	direction_y = gui.add_number("Approach direction Y", initial_value=0.0, step=0.1)
	direction_z = gui.add_number("Approach direction Z", initial_value=1.0, step=0.1)
	direction_tolerance_deg = gui.add_slider("Direction tolerance (deg)", min=1.0, max=90.0, step=1.0, initial_value=15.0)
	direction_status = gui.add_text("Direction filter status", "NPZ has direction data" if direction_success_mask is not None else "Recalculate to enable direction filtering", disabled=True)
	selected_voxel = gui.add_text("Selected voxel", "Click a visible voxel to inspect it", disabled=True)
	selected_shape = gui.add_text("Selected capability shape", "No directional capability data" if shape_type is None else "Click a visible voxel", disabled=True)

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
		normalized_reachable_ri = (reachable_values - ri_min) / max(ri_max - ri_min, 1e-8)
		voxel_colors = np.full((points.shape[0], 3), (170, 170, 170), dtype=np.float32)
		voxel_colors[reachable_mask] = _paper_ri_colormap(normalized_reachable_ri)
	else:
		ri_min = 0.0
		ri_max = 1.0
		voxel_colors = np.full((points.shape[0], 3), (170, 170, 170), dtype=np.float32)

	unit_vertices, unit_faces = _build_unit_icosahedron_mesh()
	reachable_handle = None
	unreachable_handle = None
	direction_handle = None
	shape_handle = None
	visible_indices = np.arange(points.shape[0], dtype=np.int32)

	def show_selected_voxel(voxel_index: int) -> None:
		nonlocal direction_handle, shape_handle
		if voxel_index < 0 or voxel_index >= points.shape[0]:
			return
		if direction_handle is not None:
			direction_handle.remove()
			direction_handle = None
		if shape_handle is not None:
			shape_handle.remove()
			shape_handle = None
		position = points[voxel_index]
		selected_voxel.value = f"id={voxel_index}, xyz={position.tolist()}, RI={float(ri[voxel_index]):.3f}"
		if direction_success_mask is None or direction_vectors is None:
			selected_shape.value = "Recalculate this NPZ to inspect direction data"
			return
		radius = max(float(args.point_size) * 3.0, resolution * 0.5)
		reachable_directions = direction_success_mask[voxel_index]
		direction_points = position.reshape(1, 3) + radius * direction_vectors
		direction_colors = np.where(reachable_directions[:, None], (40, 220, 70), (210, 45, 45)).astype(np.uint8)
		direction_handle = server.scene.add_point_cloud(
			"/world/chassis_link/capability/selected_directions",
			direction_points,
			direction_colors,
			point_size=max(0.004, float(args.point_size) * 0.35),
			point_shape="rounded",
			precision="float32",
		)
		if shape_type is None:
			selected_shape.value = "Direction data available; shape data missing"
			return
		axis = shape_axis[voxel_index]
		angle = float(shape_opening_angle_rad[voxel_index])
		dots = direction_vectors @ axis
		shape_name = str(shape_type[voxel_index])
		if shape_name == "cone":
			shape_membership = np.arccos(np.clip(dots, -1.0, 1.0)) <= angle
		elif shape_name == "double_cone":
			shape_membership = np.arccos(np.clip(np.abs(dots), -1.0, 1.0)) <= angle
		else:
			shape_membership = np.arcsin(np.clip(np.abs(dots), 0.0, 1.0)) <= angle
		shape_handle = server.scene.add_point_cloud(
			"/world/chassis_link/capability/selected_shape",
			direction_points[shape_membership],
			(40, 180, 255),
			point_size=max(0.003, float(args.point_size) * 0.22),
			point_shape="diamond",
			precision="float32",
		)
		selected_shape.value = f"{shape_name}; SFE={float(shape_sfe[voxel_index]):.1f}; opening={np.rad2deg(angle):.1f} deg; axis={axis.tolist()}"

	def select_from_ray(ray_origin: tuple[float, float, float], ray_direction: tuple[float, float, float]) -> None:
		if visible_indices.size == 0:
			return
		world_to_chassis_rotation = np.diag([-1.0, -1.0, 1.0]).astype(np.float32)
		origin = world_to_chassis_rotation @ (np.asarray(ray_origin, dtype=np.float32) - np.asarray(WORLD_TO_CHASSIS_POSITION, dtype=np.float32))
		direction = world_to_chassis_rotation @ np.asarray(ray_direction, dtype=np.float32)
		direction /= max(float(np.linalg.norm(direction)), 1e-8)
		candidate_points = points[visible_indices]
		along_ray = (candidate_points - origin) @ direction
		perpendicular = candidate_points - (origin.reshape(1, 3) + along_ray[:, None] * direction.reshape(1, 3))
		distance_sq = np.sum(perpendicular * perpendicular, axis=1)
		distance_sq[along_ray < 0.0] = np.inf
		show_selected_voxel(int(visible_indices[int(np.argmin(distance_sq))]))

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
		if show_plane_cut.value:
			normal = np.array([plane_normal_x.value, plane_normal_y.value, plane_normal_z.value], dtype=np.float32)
			normal_norm = float(np.linalg.norm(normal))
			if normal_norm > 1e-8:
				normal /= normal_norm
				distance = np.abs(points @ normal - float(plane_offset.value))
				visible_mask &= distance <= 0.5 * max(0.0, float(plane_thickness.value))
		if enable_direction_filter.value and direction_success_mask is not None:
			requested_direction = np.array([direction_x.value, direction_y.value, direction_z.value], dtype=np.float32)
			requested_norm = float(np.linalg.norm(requested_direction))
			if requested_norm > 1e-8:
				requested_direction /= requested_norm
				cosine_threshold = float(np.cos(np.deg2rad(float(direction_tolerance_deg.value))))
				matching_directions = (direction_vectors @ requested_direction) >= cosine_threshold
				visible_mask &= np.any(direction_success_mask[:, matching_directions], axis=1)
				direction_status.value = f"Matched {int(np.count_nonzero(matching_directions))} sampled directions"
			else:
				direction_status.value = "Enter a non-zero approach direction"

		visible_indices = np.flatnonzero(visible_mask)
		visible_reachable = np.logical_and(visible_mask, reachable_mask)
		visible_unreachable = np.logical_and(visible_mask, unreachable_mask)

		if show_reachable.value and np.any(visible_reachable):
			idx = np.flatnonzero(visible_reachable)
			reachable_points = points[idx]
			reachable_handle = server.scene.add_batched_meshes_simple(
				"/world/chassis_link/ri/reachable",
				unit_vertices,
				unit_faces,
				np.tile(np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), (reachable_points.shape[0], 1)),
				reachable_points,
				batched_scales=np.full((reachable_points.shape[0],), float(args.point_size) * 0.35, dtype=np.float32),
				batched_colors=voxel_colors[idx],
					batched_opacities=np.full((reachable_points.shape[0],), float(reachable_alpha.value), dtype=np.float32),
				opacity=1.0,
				material="standard",
				flat_shading=False,
				cast_shadow=False,
				receive_shadow=False,
			)
			@reachable_handle.on_click
			def _(event) -> None:
				select_from_ray(event.ray_origin, event.ray_direction)

		if show_unreachable.value and np.any(visible_unreachable):
			unreachable_points = points[visible_unreachable]
			unreachable_handle = server.scene.add_batched_meshes_simple(
				"/world/chassis_link/ri/unreachable",
				unit_vertices,
				unit_faces,
				np.tile(np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), (unreachable_points.shape[0], 1)),
				unreachable_points,
				batched_scales=np.full((unreachable_points.shape[0],), float(args.point_size) * 0.35, dtype=np.float32),
				batched_colors=voxel_colors[visible_unreachable],
				batched_opacities=np.full((unreachable_points.shape[0],), float(unreachable_alpha.value), dtype=np.float32),
				opacity=1.0,
				material="standard",
				flat_shading=False,
				cast_shadow=False,
				receive_shadow=False,
			)
			@unreachable_handle.on_click
			def _(event) -> None:
				select_from_ray(event.ray_origin, event.ray_direction)

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
		robot_frame.visible = bool(show_robot.value)

	@show_unreachable.on_update
	def _(_event) -> None:
		refresh()

	@show_slice.on_update
	def _(_event) -> None:
		refresh()

	for handle in (
		show_plane_cut, plane_normal_x, plane_normal_y, plane_normal_z, plane_offset, plane_thickness,
		enable_direction_filter, direction_x, direction_y, direction_z, direction_tolerance_deg,
	):
		handle.on_update(lambda _: refresh())

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
		center = np.array(
			[
				0.5 * (bounds[0] + bounds[1]),
				0.5 * (bounds[2] + bounds[3]),
				0.5 * (bounds[4] + bounds[5]),
			],
			dtype=np.float32,
		)
		server.scene.add_frame(
			"/world/chassis_link/workspace_center",
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
