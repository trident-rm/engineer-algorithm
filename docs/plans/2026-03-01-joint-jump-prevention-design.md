# Joint Jump Prevention in test_constraint.cpp

## Problem
During simulation (test_constraint), joint angles can suddenly jump when the
controller drives the end-effector near singularities or workspace boundaries.
The damped Jacobian pseudo-inverse produces large delta_q values, and post-hoc
joint clamping breaks motion consistency, causing cascading jumps.

## Solution: Adaptive Damping + Joint Delta Clamping

All changes are in `constrained_step()` in `src/z_plan/src/test_constraint.cpp`.

### 1. Adaptive Damping
Replace fixed `damping = 0.005` with condition-number-based scaling:
- cond < 50: damping = 0.005 (normal)
- 50 < cond < 200: linear interpolation 0.005 -> 0.1
- cond > 200: damping = 0.1 (max suppression)

### 2. Joint Delta Clamping (hard safety limit)
After computing delta_q, if any component exceeds 0.1 rad:
- Scale ALL deltas uniformly by `(0.1 / max_delta)`
- Preserves Jacobian-consistent motion direction

### 3. Joint Limit Margin
Replace post-hoc per-joint clamping with preventive scaling:
- If any joint would come within 0.05 rad of its limit, scale back entire delta_q
- Maintains motion direction consistency

## Files Modified
- `src/z_plan/src/test_constraint.cpp` — `constrained_step()` method only

## Verification
- Run `ros2 launch z_plan test_constraint.launch.py`
- Observe no "Large joint delta detected!" warnings
- Verify arm moves smoothly without sudden configuration changes
- Verify e_hat constraint still holds (drift < 0.5mm)
