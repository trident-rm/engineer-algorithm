"""Sequential optimizer for Pyroki Type-III task trajectories."""

from dataclasses import dataclass

import jax
import jax.numpy as jnp
import jaxlie
import numpy as np
from scipy.optimize import minimize


@dataclass(frozen=True)
class CostWeights:
    position: float = 10000.0
    axis_alignment: float = 5000.0
    smoothness: float = 10.0
    joint_limit: float = 5.0
    manipulability: float = 0.001


@dataclass
class TrajectoryResult:
    joint_positions: np.ndarray
    target_positions: np.ndarray
    target_axes: np.ndarray
    position_errors: np.ndarray
    axis_errors_rad: np.ndarray
    optimizer_iterations: int
    optimizer_success: bool
    objective_value: float


def position_cost(robot, joint_positions, target_position, link_index):
    """Squared TCP position error in meters."""
    tcp_position = robot.forward_kinematics(joint_positions)[link_index, 4:]
    return jnp.sum(jnp.square(tcp_position - target_position))


def axis_alignment_cost(robot, joint_positions, target_axis, link_index):
    """One minus the dot product of the current and target TCP X axes."""
    pose = robot.forward_kinematics(joint_positions)[link_index]
    tcp_x_axis = jaxlie.SO3(pose[:4]).apply(jnp.array([1.0, 0.0, 0.0]))
    return 1.0 - jnp.clip(jnp.dot(tcp_x_axis, target_axis), -1.0, 1.0)


def smoothness_cost(joint_positions, previous_joint_positions):
    """Squared joint displacement from the previous sequential solution."""
    return jnp.sum(jnp.square(joint_positions - previous_joint_positions))


def joint_limit_cost(joint_positions, lower_limits, upper_limits, margin=0.15, temperature=0.03):
    """Softly penalize configurations approaching either joint limit."""
    lower_distance = temperature * jax.nn.softplus(
        (lower_limits + margin - joint_positions) / temperature
    )
    upper_distance = temperature * jax.nn.softplus(
        (joint_positions - upper_limits + margin) / temperature
    )
    return jnp.sum(jnp.square(lower_distance) + jnp.square(upper_distance))


def manipulability_cost(robot, joint_positions, link_index):
    """Negative Yoshikawa translational manipulability of the TCP."""
    position_jacobian = jax.jacfwd(
        lambda cfg: robot.forward_kinematics(cfg)[link_index, 4:]
    )(joint_positions)
    gram = position_jacobian @ position_jacobian.T
    manipulability = jnp.sqrt(jnp.maximum(jnp.linalg.det(gram), 0.0) + 1e-12)
    return -manipulability


def total_cost(
    robot,
    joint_positions,
    target_position,
    target_axis,
    previous_joint_positions,
    link_index,
    lower_limits,
    upper_limits,
    weights,
):
    """Weighted Type-III objective without collision terms."""
    return (
        weights.position
        * position_cost(robot, joint_positions, target_position, link_index)
        + weights.axis_alignment
        * axis_alignment_cost(robot, joint_positions, target_axis, link_index)
        + weights.smoothness
        * smoothness_cost(joint_positions, previous_joint_positions)
        + weights.joint_limit
        * joint_limit_cost(joint_positions, lower_limits, upper_limits)
        + weights.manipulability
        * manipulability_cost(robot, joint_positions, link_index)
    )


def trajectory_total_cost(
    robot,
    joint_trajectory,
    target_positions,
    target_axes,
    link_index,
    lower_limits,
    upper_limits,
    weights,
):
    """Sum task, smoothness, limit, and manipulability costs over the full path."""
    link_poses = robot.forward_kinematics(joint_trajectory)[:, link_index]
    tcp_positions = link_poses[:, 4:]
    position_terms = jnp.sum(jnp.square(tcp_positions - target_positions), axis=-1)

    tcp_x_axes = jax.vmap(
        lambda pose: jaxlie.SO3(pose[:4]).apply(jnp.array([1.0, 0.0, 0.0]))
    )(link_poses)
    axis_dots = jnp.sum(tcp_x_axes * target_axes, axis=-1)
    axis_terms = 1.0 - jnp.clip(axis_dots, -1.0, 1.0)

    smoothness = jnp.sum(jnp.square(joint_trajectory[1:] - joint_trajectory[:-1]))
    limits = joint_limit_cost(joint_trajectory, lower_limits, upper_limits)

    position_jacobians = jax.vmap(
        jax.jacfwd(
            lambda cfg: robot.forward_kinematics(cfg)[link_index, 4:]
        )
    )(joint_trajectory)
    position_grams = position_jacobians @ jnp.swapaxes(position_jacobians, -1, -2)
    manipulabilities = jnp.sqrt(
        jnp.maximum(jnp.linalg.det(position_grams), 0.0) + 1e-12
    )

    return (
        weights.position * jnp.sum(position_terms)
        + weights.axis_alignment * jnp.sum(axis_terms)
        + weights.smoothness * smoothness
        + weights.joint_limit * limits
        - weights.manipulability * jnp.sum(manipulabilities)
    )


def make_circular_task_path(
    start_position,
    start_axis,
    rotation_axis,
    start_radial,
    sweep_rad,
    samples,
    orientation_axis=None,
    orientation_sweep_rad=None,
):
    """Generate circular positions and independently rotated TCP-X targets."""
    start_position = np.asarray(start_position, dtype=np.float64)
    start_axis = np.asarray(start_axis, dtype=np.float64)
    rotation_axis = np.asarray(rotation_axis, dtype=np.float64)
    orientation_axis = np.asarray(
        rotation_axis if orientation_axis is None else orientation_axis,
        dtype=np.float64,
    )
    start_radial = np.asarray(start_radial, dtype=np.float64)
    if samples < 1 or sweep_rad <= 0.0:
        raise ValueError("samples and sweep_rad must be positive")
    orientation_sweep_rad = (
        sweep_rad if orientation_sweep_rad is None else float(orientation_sweep_rad)
    )
    if orientation_sweep_rad <= 0.0:
        raise ValueError("orientation_sweep_rad must be positive")
    for name, vector in (
        ("start_axis", start_axis),
        ("rotation_axis", rotation_axis),
        ("orientation_axis", orientation_axis),
        ("start_radial", start_radial),
    ):
        norm = np.linalg.norm(vector)
        if not np.isfinite(norm) or norm < 1e-8:
            raise ValueError(f"{name} must be a finite non-zero vector")
        if name != "start_radial":
            if name == "start_axis":
                start_axis = vector / norm
            elif name == "rotation_axis":
                rotation_axis = vector / norm
            elif name == "orientation_axis":
                orientation_axis = vector / norm
    radius = np.linalg.norm(start_radial)
    if radius < 1e-8:
        raise ValueError("start_radial must have a non-zero radius")
    if abs(float(np.dot(start_radial / radius, rotation_axis))) > 1e-5:
        raise ValueError("start_radial must be perpendicular to rotation_axis")

    center = start_position - start_radial
    position_axis_cross = np.array(
        [
            [0.0, -rotation_axis[2], rotation_axis[1]],
            [rotation_axis[2], 0.0, -rotation_axis[0]],
            [-rotation_axis[1], rotation_axis[0], 0.0],
        ]
    )
    orientation_axis_cross = np.array(
        [
            [0.0, -orientation_axis[2], orientation_axis[1]],
            [orientation_axis[2], 0.0, -orientation_axis[0]],
            [-orientation_axis[1], orientation_axis[0], 0.0],
        ]
    )
    target_positions = []
    target_axes = []
    for progress in np.linspace(0.0, 1.0, samples + 1):
        theta = progress * sweep_rad
        orientation_theta = progress * orientation_sweep_rad
        position_rotation = (
            np.eye(3)
            + np.sin(theta) * position_axis_cross
            + (1.0 - np.cos(theta)) * (position_axis_cross @ position_axis_cross)
        )
        orientation_rotation = (
            np.eye(3)
            + np.sin(orientation_theta) * orientation_axis_cross
            + (1.0 - np.cos(orientation_theta))
            * (orientation_axis_cross @ orientation_axis_cross)
        )
        target_positions.append(center + position_rotation @ start_radial)
        target_axis = orientation_rotation @ start_axis
        target_axes.append(target_axis / np.linalg.norm(target_axis))
    return center, np.asarray(target_positions), np.asarray(target_axes)


class TypeIIITaskPlanner:
    """Warm-start per sample, then jointly optimize the full joint trajectory."""

    def __init__(
        self,
        robot,
        link_index,
        weights=None,
    ):
        self.robot = robot
        self.link_index = int(link_index)
        self.weights = weights or CostWeights()
        self.lower_limits = np.asarray(robot.joints.lower_limits, dtype=np.float64)
        self.upper_limits = np.asarray(robot.joints.upper_limits, dtype=np.float64)
        self._value_and_grad = jax.jit(
            jax.value_and_grad(
                lambda q, p, x, q_prev: total_cost(
                    self.robot,
                    q,
                    p,
                    x,
                    q_prev,
                    self.link_index,
                    jnp.asarray(self.lower_limits),
                    jnp.asarray(self.upper_limits),
                    self.weights,
                )
            )
        )
        self._trajectory_value_and_grad = jax.jit(
            jax.value_and_grad(
                lambda free_trajectory, start, positions, axes: trajectory_total_cost(
                    self.robot,
                    jnp.concatenate([start[None, :], free_trajectory], axis=0),
                    positions,
                    axes,
                    self.link_index,
                    jnp.asarray(self.lower_limits),
                    jnp.asarray(self.upper_limits),
                    self.weights,
                )
            )
        )

    def _measure_errors(self, joint_positions, target_position, target_axis):
        pose = np.asarray(self.robot.forward_kinematics(joint_positions))[self.link_index]
        actual_position = pose[4:]
        actual_axis = np.asarray(
            jaxlie.SO3(jnp.asarray(pose[:4])).apply(jnp.array([1.0, 0.0, 0.0]))
        )
        position_error = float(np.linalg.norm(actual_position - target_position))
        dot = float(np.clip(np.dot(actual_axis, target_axis), -1.0, 1.0))
        axis_error = float(np.arccos(dot))
        return position_error, axis_error

    def plan(self, start_joint_positions, target_positions, target_axes):
        start_joint_positions = np.asarray(start_joint_positions, dtype=np.float64)
        target_positions = np.asarray(target_positions, dtype=np.float64)
        target_axes = np.asarray(target_axes, dtype=np.float64)
        if target_positions.ndim != 2 or target_positions.shape[1] != 3:
            raise ValueError("target_positions must have shape (samples, 3)")
        if target_axes.shape != target_positions.shape:
            raise ValueError("target_axes must have the same shape as target_positions")
        if start_joint_positions.shape != self.lower_limits.shape:
            raise ValueError("start_joint_positions does not match the robot joint count")
        if target_positions.shape[0] < 2:
            raise ValueError("At least two task samples are required")

        warm_start_path = [start_joint_positions]
        warm_start_iterations = 0
        previous = start_joint_positions

        for sample_index in range(1, target_positions.shape[0]):
            target_position = target_positions[sample_index]
            target_axis = target_axes[sample_index]

            def objective(q):
                value, gradient = self._value_and_grad(
                    jnp.asarray(q),
                    jnp.asarray(target_position),
                    jnp.asarray(target_axis),
                    jnp.asarray(previous),
                )
                return float(value), np.asarray(gradient, dtype=np.float64)

            result = minimize(
                objective,
                previous,
                jac=True,
                method="L-BFGS-B",
                bounds=list(zip(self.lower_limits, self.upper_limits)),
                options={"maxiter": 150, "ftol": 1e-10, "maxls": 40},
            )
            if np.all(np.isfinite(result.x)):
                previous = result.x
                warm_start_iterations += int(result.nit)
            warm_start_path.append(previous)

        initial_free_trajectory = np.asarray(warm_start_path[1:], dtype=np.float64)
        free_point_count = initial_free_trajectory.shape[0]

        def trajectory_objective(flat_trajectory):
            free_trajectory = jnp.asarray(flat_trajectory).reshape(
                (free_point_count, self.lower_limits.size)
            )
            value, gradient = self._trajectory_value_and_grad(
                free_trajectory,
                jnp.asarray(start_joint_positions),
                jnp.asarray(target_positions),
                jnp.asarray(target_axes),
            )
            return float(value), np.asarray(gradient, dtype=np.float64).reshape(-1)

        global_result = minimize(
            trajectory_objective,
            initial_free_trajectory.reshape(-1),
            jac=True,
            method="L-BFGS-B",
            bounds=list(zip(
                np.tile(self.lower_limits, free_point_count),
                np.tile(self.upper_limits, free_point_count),
            )),
            options={"maxiter": 500, "ftol": 1e-10, "maxls": 50},
        )
        if not np.all(np.isfinite(global_result.x)):
            raise RuntimeError("Global trajectory optimizer returned non-finite joints")

        optimized_free_trajectory = global_result.x.reshape(
            (free_point_count, self.lower_limits.size)
        )
        joint_trajectory = np.concatenate(
            [start_joint_positions[None, :], optimized_free_trajectory], axis=0
        )
        errors = [
            self._measure_errors(joint_positions, target_position, target_axis)
            for joint_positions, target_position, target_axis in zip(
                joint_trajectory, target_positions, target_axes
            )
        ]

        return TrajectoryResult(
            joint_positions=joint_trajectory,
            target_positions=target_positions,
            target_axes=target_axes,
            position_errors=np.asarray([error[0] for error in errors]),
            axis_errors_rad=np.asarray([error[1] for error in errors]),
            optimizer_iterations=warm_start_iterations + int(global_result.nit),
            optimizer_success=bool(global_result.success),
            objective_value=float(global_result.fun),
        )