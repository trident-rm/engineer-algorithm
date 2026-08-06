#!/usr/bin/env python3

from __future__ import annotations

import argparse
from dataclasses import dataclass
import importlib
import logging
import os
import time
from pathlib import Path
import sys
from typing import Any

os.environ.setdefault("XLA_PYTHON_CLIENT_PREALLOCATE", "false")

import jax
import jax.numpy as jnp
import jaxlie
import jaxls
import numpy as np
import pyroki as pk
import yourdfpy


URDF_PATH = os.environ.get(
	"ROBO_ARM_URDF",
	"src/arms/combine_urdf/urdf/combine_urdf.urdf",
)
EE_LINK = os.environ.get("ROBO_ARM_EE_LINK", "gripper")
BASE_LINK = os.environ.get("ROBO_ARM_BASE_LINK", "base_link")
DEFAULT_BOUNDS = (-1.0, 1.0, -1.0, 1.0, -0.5, 1.0) # [xmin, xmax, ymin, ymax, zmin, zmax]
DEFAULT_RESOLUTION = 0.05


LOGGER = logging.getLogger(__name__)


def _import_pyroki_snippets() -> Any:
	try:
		return importlib.import_module("pyroki_snippets")
	except ModuleNotFoundError:
		candidate_paths: list[str] = []
		env_path = os.environ.get("PYROKI_EXAMPLES_PATH")
		if env_path:
			candidate_paths.append(env_path)

		script_path = Path(__file__).resolve()
		for parent in script_path.parents:
			pyroki_examples = parent / "pyroki" / "examples"
			if pyroki_examples.is_dir():
				candidate_paths.append(str(pyroki_examples))

        # candidate_paths.append("/home/hku/third_party/pyroki/examples")

		for path in candidate_paths:
			if path and os.path.isdir(path) and path not in sys.path:
				sys.path.insert(0, path)
			try:
				return importlib.import_module("pyroki_snippets")
			except ModuleNotFoundError:
				continue

	raise ModuleNotFoundError("Cannot import 'pyroki_snippets'.")


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
		self.ee_link = ee_link
		self.spec = spec
		self.base_link_index = jnp.array(robot.links.names.index(BASE_LINK))
		self.target_link_index = jnp.array(robot.links.names.index(ee_link))
		self.num_joints = int(robot.joints.num_actuated_joints)
		self.lower_limits = jnp.asarray(robot.joints.lower_limits)
		self.upper_limits = jnp.asarray(robot.joints.upper_limits)
		self.seed_count = max(1, int(spec.seed_count))
		self.refine_count = max(1, min(int(spec.refine_count), self.seed_count))

		self.orientation_quats = self._build_orientation_quats(spec.ori_samples)
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
		for idx in range(1, sample_count):
			next_idx = int(np.argmax(min_dist))
			selected[idx] = pool[next_idx]
			min_dist = np.minimum(min_dist, np.arccos(np.clip(np.abs(pool @ selected[idx]), 0.0, 1.0)))
		return jnp.asarray(selected)

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
				jaxlie.SE3.from_rotation_and_translation(
					jaxlie.SO3(target_wxyz), target_position
				),
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
		best_idx = jnp.argsort(coarse_final_cost)[: self.refine_count]
		refined_sol, refined_summary = vmapped(
			target_wxyz,
			target_position,
			coarse_sol[best_idx],
			coarse_summary.lambda_history[jnp.arange(self.seed_count), coarse_summary.iterations][best_idx],
			16,
		)
		final_idx = jnp.argmin(refined_summary.cost_history[jnp.arange(self.refine_count), refined_summary.iterations])
		return refined_sol[final_idx]

	def _forward_kinematics(self, q: jax.Array) -> jax.Array:
		return self.robot.forward_kinematics(q)[self.target_link_index]

	def build_grid(self) -> np.ndarray:
		xmin, xmax, ymin, ymax, zmin, zmax = self.spec.bounds
		step = self.spec.resolution
		xs = np.arange(xmin + 0.5 * step, xmax - 0.5 * step + 1e-12, step, dtype=np.float32)
		ys = np.arange(ymin + 0.5 * step, ymax - 0.5 * step + 1e-12, step, dtype=np.float32)
		zs = np.arange(zmin + 0.5 * step, zmax - 0.5 * step + 1e-12, step, dtype=np.float32)
		points_in_base = np.stack(np.meshgrid(xs, ys, zs, indexing="ij"), axis=-1).reshape(-1, 3)
		zero_cfg = jnp.zeros((self.num_joints,), dtype=self.lower_limits.dtype)
		base_pose = np.asarray(self.robot.forward_kinematics(zero_cfg)[self.base_link_index])
		base_translation = base_pose[4:7].astype(np.float32)
		return points_in_base + base_translation.reshape(1, 3)

	def evaluate(self) -> dict[str, np.ndarray]:
		points = self.build_grid()
		point_count = points.shape[0]
		ori_count = int(self.orientation_quats.shape[0])

		reach_counts_np = np.zeros((point_count,), dtype=np.int32)
		points_jnp = jnp.asarray(points)
		for start, end in self._iter_chunks(point_count):
			chunk_points = points_jnp[start:end]
			chunk_count = end - start
			tiled_positions = jnp.repeat(chunk_points, ori_count, axis=0)
			tiled_wxyz = jnp.tile(self.orientation_quats, (chunk_count, 1))

			solutions = self._batched_solve(tiled_wxyz, tiled_positions)
			fk = self._batched_fk(solutions)

			actual_wxyz = fk[:, :4]
			actual_xyz = fk[:, 4:7]
			pos_err = jnp.linalg.norm(actual_xyz - tiled_positions, axis=1)
			ori_err = jnp.linalg.norm(
				(jaxlie.SO3(tiled_wxyz).inverse() @ jaxlie.SO3(actual_wxyz)).log(),
				axis=1,
			)
			success = jnp.logical_and(pos_err <= self.spec.pos_tol, ori_err <= self.spec.ori_tol_rad)
			success_matrix = success.reshape(chunk_count, ori_count)
			reach_counts_np[start:end] = np.asarray(jnp.sum(success_matrix, axis=1), dtype=np.int32)
			LOGGER.info("Processed points %d to %d / %d", start, end, point_count)
   
		ri = reach_counts_np.astype(np.float32) / float(max(1, ori_count))

		return {
			"points": np.asarray(points),
			"ri": np.asarray(ri, dtype=np.float32),
			"reach_counts": reach_counts_np,
			"orientation_quats": np.asarray(self.orientation_quats),
		}


def _parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="GPU IK reachability index with JAX/JAXLS.")
	parser.add_argument("--bounds", nargs=6, type=float, default=list(DEFAULT_BOUNDS))
	parser.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION)
	parser.add_argument("--ori-samples", type=int, default=64)
	parser.add_argument("--ik-pos-tol", type=float, default=0.005)
	parser.add_argument("--ik-ori-tol-deg", type=float, default=3.0)
	parser.add_argument("--batch-size", type=int, default=128)
	parser.add_argument("--seed-count", type=int, default=8)
	parser.add_argument("--refine-count", type=int, default=2)
	parser.add_argument("--output", type=str, default="output/pk_ik_ri.npz")
	return parser.parse_args()


def main() -> None:
	logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")
	args = _parse_args()
	snippets = _import_pyroki_snippets()
	_ = snippets  # keep import path behavior aligned with visualizer setup

	urdf = yourdfpy.URDF.load(
		URDF_PATH,
		filename_handler=_make_package_filename_handler(),
		build_collision_scene_graph=True,
		load_meshes=True,
		load_collision_meshes=True,
	)
	robot = pk.Robot.from_urdf(urdf)
	if BASE_LINK not in robot.links.names:
		raise ValueError(f"Base link '{BASE_LINK}' not found in robot links: {list(robot.links.names)}")

	spec = ScanSpec(
		bounds=(
			float(args.bounds[0]),
			float(args.bounds[1]),
			float(args.bounds[2]),
			float(args.bounds[3]),
			float(args.bounds[4]),
			float(args.bounds[5]),
		),
		resolution=float(args.resolution),
		ori_samples=int(args.ori_samples),
		pos_tol=float(args.ik_pos_tol),
		ori_tol_rad=float(np.deg2rad(args.ik_ori_tol_deg)),
		batch_size=int(args.batch_size),
		seed_count=int(args.seed_count),
		refine_count=int(args.refine_count),
	)
	evaluator = BatchedIKReachability(robot=robot, ee_link=EE_LINK, spec=spec)
	t0 = time.time()
	result = evaluator.evaluate()
	t1 = time.time()
	LOGGER.info("Evaluation completed in %.2f seconds.", t1 - t0)

	output_path = Path(args.output)
	output_path.parent.mkdir(parents=True, exist_ok=True)
	np.savez_compressed(
		output_path,
		voxel_points=result["points"],
		ri_values=result["ri"],
		pose_success_counts=result["reach_counts"],
		orientation_quats=result["orientation_quats"],
		bounds=np.asarray(spec.bounds, dtype=np.float32),
		resolution=np.float32(spec.resolution),
		ori_samples=np.int32(spec.ori_samples),
	)
	LOGGER.info("Saved reachability index to %s", output_path)


if __name__ == "__main__":
	main()
