"""Sequential optimizer for Pyroki Type-III task trajectories."""

from dataclasses import dataclass

import jax
import jax.numpy as jnp
import jaxlie
import numpy as np
from scipy.optimize import minimize

POSITION_ERROR_SCALE_M = 0.01
AXIS_ERROR_SCALE = 1.0 - np.cos(np.deg2rad(5.0))
JOINT_STEP_SCALE_RAD = 0.1
JOINT_LIMIT_MARGIN_RAD = 0.15
MANIPULABILITY_SCALE_M3 = 0.05
WARM_START_KEYFRAME_INTERVAL = 5


@dataclass(frozen=True)
class CostWeights:
    position: float = 1.0
    axis_alignment: float = 1.55
    smoothness: float = 0.0
    joint_limit: float = 0.0
    manipulability: float = 0.0


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
    weighted_cost_components: dict[str, float]


def position_cost(tcp_position, target_position):
    """Squared TCP position error normalized to a 1 cm reference."""
    return jnp.sum(jnp.square(tcp_position - target_position)) / POSITION_ERROR_SCALE_M**2


def axis_alignment_cost(tcp_x_axis, target_axis):
    """Axis error normalized so a 5 degree error has unit cost."""
    error = 1.0 - jnp.clip(jnp.dot(tcp_x_axis, target_axis), -1.0, 1.0)
    return error / AXIS_ERROR_SCALE


def smoothness_cost(joint_positions, previous_joint_positions):
    """Mean squared joint displacement normalized to a 0.1 rad RMS reference."""
    return jnp.mean(
        jnp.square(joint_positions - previous_joint_positions)
    ) / JOINT_STEP_SCALE_RAD**2


def joint_limit_cost(
    joint_positions,
    lower_limits,
    upper_limits,
    margin=JOINT_LIMIT_MARGIN_RAD,
    temperature=0.03,
):
    """Softly penalize configurations approaching either joint limit."""
    lower_distance = temperature * jax.nn.softplus(
        (lower_limits + margin - joint_positions) / temperature
    )
    upper_distance = temperature * jax.nn.softplus(
        (joint_positions - upper_limits + margin) / temperature
    )
    return jnp.mean(jnp.square(lower_distance) + jnp.square(upper_distance)) / margin**2


def manipulability_cost(position_jacobian):
    """Negative Yoshikawa translational manipulability of the TCP."""
    gram = position_jacobian @ position_jacobian.T
    manipulability = jnp.sqrt(jnp.maximum(jnp.linalg.det(gram), 0.0) + 1e-12)
    return -manipulability / MANIPULABILITY_SCALE_M3


def _position_with_pose(robot, joint_positions, link_index):
    pose = robot.forward_kinematics(joint_positions)[link_index]
    return pose[4:], pose


def _tcp_x_axis(pose):
    return jaxlie.SO3(pose[:4]).apply(jnp.array([1.0, 0.0, 0.0]))


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
    position_weight=1.0,
    axis_weight=1.0,
):
    """Weighted Type-III objective without collision terms."""
    position_jacobian = None
    if weights.manipulability != 0.0:
        position_jacobian, pose = jax.jacfwd(
            lambda cfg: _position_with_pose(robot, cfg, link_index),
            has_aux=True,
        )(joint_positions)
    elif weights.position != 0.0 or weights.axis_alignment != 0.0:
        _, pose = _position_with_pose(robot, joint_positions, link_index)

    cost = jnp.asarray(0.0, dtype=joint_positions.dtype)
    if weights.position != 0.0:
        cost = cost + (
            weights.position
            * position_weight
            * position_cost(pose[4:], target_position)
        )
    if weights.axis_alignment != 0.0:
        cost = cost + (
            weights.axis_alignment
            * axis_weight
            * axis_alignment_cost(_tcp_x_axis(pose), target_axis)
        )
    if weights.smoothness != 0.0:
        cost = cost + weights.smoothness * smoothness_cost(
            joint_positions, previous_joint_positions
        )
    if weights.joint_limit != 0.0:
        cost = cost + weights.joint_limit * joint_limit_cost(
            joint_positions, lower_limits, upper_limits
        )
    if weights.manipulability != 0.0:
        cost = cost + weights.manipulability * manipulability_cost(position_jacobian)
    return cost


def trajectory_total_cost(
    robot,
    joint_trajectory,
    target_positions,
    target_axes,
    link_index,
    lower_limits,
    upper_limits,
    weights,
    position_weights=None,
    axis_weights=None,
):
    """Mean dimensionless objective over the full path."""
    return jnp.sum(
        trajectory_cost_components(
            robot,
            joint_trajectory,
            target_positions,
            target_axes,
            link_index,
            lower_limits,
            upper_limits,
            weights,
            position_weights,
            axis_weights,
        )
    )


def trajectory_cost_components(
    robot,
    joint_trajectory,
    target_positions,
    target_axes,
    link_index,
    lower_limits,
    upper_limits,
    weights,
    position_weights=None,
    axis_weights=None,
):
    """Return the five weighted, normalized objective contributions."""
    position_jacobians = None
    if weights.manipulability != 0.0:
        position_jacobians, link_poses = jax.vmap(
            jax.jacfwd(
                lambda cfg: _position_with_pose(robot, cfg, link_index),
                has_aux=True,
            )
        )(joint_trajectory)
    elif weights.position != 0.0 or weights.axis_alignment != 0.0:
        link_poses = robot.forward_kinematics(joint_trajectory)[:, link_index]

    zero = jnp.asarray(0.0, dtype=joint_trajectory.dtype)
    position_term = zero
    if weights.position != 0.0:
        tcp_positions = link_poses[:, 4:]
        position_terms = (
            jnp.sum(jnp.square(tcp_positions - target_positions), axis=-1)
            / POSITION_ERROR_SCALE_M**2
        )
        if position_weights is not None:
            position_terms = position_terms * jnp.asarray(position_weights)
        position_normalizer = (
            jnp.asarray(joint_trajectory.shape[0], dtype=joint_trajectory.dtype)
            if position_weights is None
            else jnp.maximum(jnp.sum(position_weights), 1.0)
        )
        position_term = (
            weights.position * jnp.sum(position_terms) / position_normalizer
        )

    axis_term = zero
    if weights.axis_alignment != 0.0:
        tcp_x_axes = jax.vmap(_tcp_x_axis)(link_poses)
        axis_dots = jnp.sum(tcp_x_axes * target_axes, axis=-1)
        axis_terms = (
            1.0 - jnp.clip(axis_dots, -1.0, 1.0)
        ) / AXIS_ERROR_SCALE
        if axis_weights is not None:
            axis_terms = axis_terms * jnp.asarray(axis_weights)
        axis_normalizer = (
            jnp.asarray(joint_trajectory.shape[0], dtype=joint_trajectory.dtype)
            if axis_weights is None
            else jnp.maximum(jnp.sum(axis_weights), 1.0)
        )
        axis_term = (
            weights.axis_alignment * jnp.sum(axis_terms) / axis_normalizer
        )

    smoothness = zero
    if weights.smoothness != 0.0:
        smoothness = weights.smoothness * jnp.mean(
            jnp.square(joint_trajectory[1:] - joint_trajectory[:-1])
        ) / JOINT_STEP_SCALE_RAD**2

    limits = zero
    if weights.joint_limit != 0.0:
        limits = weights.joint_limit * joint_limit_cost(
            joint_trajectory, lower_limits, upper_limits
        )

    manipulability_term = zero
    if weights.manipulability != 0.0:
        position_grams = position_jacobians @ jnp.swapaxes(
            position_jacobians, -1, -2
        )
        manipulabilities = jnp.sqrt(
            jnp.maximum(jnp.linalg.det(position_grams), 0.0) + 1e-12
        )
        manipulability_term = (
            -weights.manipulability
            * jnp.mean(manipulabilities)
            / MANIPULABILITY_SCALE_M3
        )

    return jnp.stack(
        [
            position_term,
            axis_term,
            smoothness,
            limits,
            manipulability_term,
        ]
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


class TypeIIIPlanner:
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
                lambda q, p, x, q_prev, position_weight, axis_weight: total_cost(
                    self.robot,
                    q,
                    p,
                    x,
                    q_prev,
                    self.link_index,
                    jnp.asarray(self.lower_limits),
                    jnp.asarray(self.upper_limits),
                    self.weights,
                    position_weight,
                    axis_weight,
                )
            )
        )
        self._trajectory_value_and_grad = jax.jit(
            jax.value_and_grad(
                lambda free_trajectory, start, positions, axes, position_weights, axis_weights: trajectory_total_cost(
                    self.robot,
                    jnp.concatenate([start[None, :], free_trajectory], axis=0),
                    positions,
                    axes,
                    self.link_index,
                    jnp.asarray(self.lower_limits),
                    jnp.asarray(self.upper_limits),
                    self.weights,
                    position_weights,
                    axis_weights,
                )
            )
        )

    def warm_up(self, start_joint_positions, sample_count):
        """Compile the single-sample and full-trajectory objectives for a path shape."""
        if sample_count < 2:
            raise ValueError("sample_count must be at least 2")
        start = jnp.asarray(start_joint_positions)
        target_positions = jnp.zeros((sample_count, 3), dtype=start.dtype)
        target_axes = jnp.broadcast_to(
            jnp.array([1.0, 0.0, 0.0], dtype=start.dtype),
            (sample_count, 3),
        )
        sample_weights = jnp.ones((sample_count,), dtype=start.dtype)

        single_outputs = self._value_and_grad(
            start,
            target_positions[0],
            target_axes[0],
            start,
            jnp.asarray(1.0, dtype=start.dtype),
            jnp.asarray(1.0, dtype=start.dtype),
        )
        for output in single_outputs:
            output.block_until_ready()

        trajectory_outputs = self._trajectory_value_and_grad(
            jnp.broadcast_to(start, (sample_count - 1, start.shape[0])),
            start,
            target_positions,
            target_axes,
            sample_weights,
            sample_weights,
        )
        for output in trajectory_outputs:
            output.block_until_ready()

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

    def plan(
        self,
        start_joint_positions,
        target_positions,
        target_axes,
        position_weights=None,
        axis_weights=None,
    ):
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
        if position_weights is None:
            position_weights = np.ones(target_positions.shape[0], dtype=np.float64)
        else:
            position_weights = np.asarray(position_weights, dtype=np.float64)
            if position_weights.shape != (target_positions.shape[0],):
                raise ValueError("position_weights must have shape (samples,)")
            if not np.all(np.isfinite(position_weights)) or np.any(position_weights < 0.0):
                raise ValueError("position_weights must be finite and non-negative")
        if axis_weights is None:
            axis_weights = np.ones(target_positions.shape[0], dtype=np.float64)
        else:
            axis_weights = np.asarray(axis_weights, dtype=np.float64)
            if axis_weights.shape != (target_positions.shape[0],):
                raise ValueError("axis_weights must have shape (samples,)")
            if not np.all(np.isfinite(axis_weights)) or np.any(axis_weights < 0.0):
                raise ValueError("axis_weights must be finite and non-negative")

        keyframe_indices = np.arange(
            WARM_START_KEYFRAME_INTERVAL,
            target_positions.shape[0],
            WARM_START_KEYFRAME_INTERVAL,
        )
        final_sample_index = target_positions.shape[0] - 1
        if keyframe_indices.size == 0 or keyframe_indices[-1] != final_sample_index:
            keyframe_indices = np.append(keyframe_indices, final_sample_index)

        keyframe_configurations = [start_joint_positions]
        optimized_keyframe_indices = [0]
        warm_start_iterations = 0
        previous = start_joint_positions

        for sample_index in keyframe_indices:
            target_position = target_positions[sample_index]
            target_axis = target_axes[sample_index]

            def objective(q):
                value, gradient = self._value_and_grad(
                    jnp.asarray(q),
                    jnp.asarray(target_position),
                    jnp.asarray(target_axis),
                    jnp.asarray(previous),
                    jnp.asarray(position_weights[sample_index]),
                    jnp.asarray(axis_weights[sample_index]),
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
            keyframe_configurations.append(previous)
            optimized_keyframe_indices.append(sample_index)

        keyframe_configurations = np.asarray(keyframe_configurations, dtype=np.float64)
        optimized_keyframe_indices = np.asarray(optimized_keyframe_indices)
        sample_indices = np.arange(target_positions.shape[0])
        initial_trajectory = np.column_stack(
            [
                np.interp(
                    sample_indices,
                    optimized_keyframe_indices,
                    keyframe_configurations[:, joint_index],
                )
                for joint_index in range(self.lower_limits.size)
            ]
        )
        initial_free_trajectory = initial_trajectory[1:]
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
                jnp.asarray(position_weights),
                jnp.asarray(axis_weights),
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
            weighted_cost_components=dict(
                zip(
                    ("position", "axis", "smoothness", "joint_limit", "manipulability"),
                    np.asarray(
                        trajectory_cost_components(
                            self.robot,
                            jnp.asarray(joint_trajectory),
                            jnp.asarray(target_positions),
                            jnp.asarray(target_axes),
                            self.link_index,
                            jnp.asarray(self.lower_limits),
                            jnp.asarray(self.upper_limits),
                            self.weights,
                            jnp.asarray(position_weights),
                            jnp.asarray(axis_weights),
                        ),
                        dtype=np.float64,
                    ).tolist(),
                )
            ),
        )