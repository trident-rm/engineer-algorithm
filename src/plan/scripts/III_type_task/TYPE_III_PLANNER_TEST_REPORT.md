# Type-III Planner Parameter Tuning and Test Report

## Executive summary

The planner was evaluated using both trajectory-average and trajectory-peak task-space errors. A finite scan of the normalized position and TCP-X objective weights selected:

| Global objective weight | Value |
|---|---:|
| Position | 1.0 |
| TCP-X alignment | 1.55 |
| Joint smoothness | 0.0 |
| Joint limits | 0.0 |
| Manipulability | 0.0 |

The planner applies additional per-waypoint multipliers of 10 at the Approach/Hover endpoint for position and TCP-X; all other waypoints use 1.

On 2,000 randomized cases with seed `20260930`, radius `0.1 m`, three intervals per stage, and thresholds of `50 mm` and `30°`, the tuned configuration achieved:

| Criterion | Passed | Rate |
|---|---:|---:|
| Average error | 2,000 / 2,000 | 100.0% |
| Peak error | 1,020 / 2,000 | 51.0% |

On the same test distribution with TCP-X weight `0.1`, peak pass was 600/2,000 (30.0%). A separate 300-case seed (`20261019`) gave 27.3% peak pass at `0.1` and 49.7% at `1.55`, supporting improvement beyond the tuning seed.

The tuning favors axis alignment and increases position error. It is the best tested compromise for peak pass in the finite scan, not a proven global optimum and not a guarantee of real robot tracking accuracy.

## Objective and pass definitions

The optimizer's normalized per-waypoint terms use reference scales of `0.01 m` for position and `5°` for TCP-X:

```text
position_term = ||p_actual - p_target||^2 / (0.01 m)^2
axis_term = (1 - dot(x_actual, x_target)) / (1 - cos(5°))

objective = position_weight * weighted_mean(position_term)
          + axis_weight * weighted_mean(axis_term)
          + optional smoothness, joint-limit and manipulability terms
```

`position_weight` and `axis_weight` are global across the whole trajectory. The optional arrays `position_weights` and `axis_weights` multiply individual waypoint contributions; they are not per-segment scalar settings. The demo and test set both arrays to `10` at the Approach/Hover endpoint and `1` elsewhere. Array-weighted terms are normalized by the sum of the corresponding waypoint weights.

Every case is classified with the configured `50 mm` position threshold, `30°` TCP-X threshold, and the joint-limit check:

- **Average-error pass:** mean of per-waypoint Euclidean position errors is at most `50 mm`, mean of per-waypoint TCP-X angular errors is at most `30°`, and no joint limit is violated.
- **Peak-error pass:** maximum position error is at most `50 mm`, maximum TCP-X angular error is at most `30°`, and no joint limit is violated.

The legacy `quality_pass` field continues to mean peak-error pass. New output explicitly records `average_quality_pass` and `peak_quality_pass`. Optimizer convergence is reported separately; convergence is not itself a quality pass.

## Test environment and fixed inputs

The comparisons used the repository's `end2end_robustness_test.py` and the same code/worktree for baseline and tuned runs. These historical runs used both the pre-change Euler-angle orientation generator and the old center box (`X=[0.65,0.75] m`, `Y=[-0.10,0.10] m`, `Z=[0.30,0.50] m`). They remain valid comparisons for that older sampled task distribution, but are not measurements of the current RoboMaster core position and `(theta, phi, alpha)` orientation ranges. Re-run the commands below with a fresh CSV path for updated-rule measurements.

| Parameter | Value |
|---|---:|
| Cases in main comparison | 2,000 |
| Main seed | `20260930` |
| Holdout cases | 300 |
| Holdout seed | `20261019` |
| Path radius | `0.1 m` |
| Core center X range in latest demo retest | `0.40–0.50 m` |
| Core center Y range | `-0.10–0.10 m` |
| Core center Z range used by the current demo | `0.30–0.50 m` |
| Hover/insert distance | `0.20 m` |
| Slide distance | `0.10 m` |
| Turn P sweep | `90°` |
| Turn Q sweep | `60°` |
| Main comparison intervals per stage | 3 |
| Samples per path, at 3 intervals/stage | 16 |
| Position threshold | `50 mm` |
| TCP-X threshold | `30°` |
| Smoothness/joint-limit/manipulability weights | `0 / 0 / 0` |
| Start configuration | `joint2=1.9`, `joint3=0.5`, `joint5=-1.6`, other joints zero, resolved through the test's default-pose IK |

The historical result tables below used the old center box, `X=[0.65,0.75] m`, `Y=[-0.10,0.10] m`, `Z=[0.30,0.50] m`, along with the old Euler orientation generator. Those measurements do not represent later demo ranges or orientation conventions. The demo's default is 8 intervals per stage (41 points). The pass-rate measurements below use 3 intervals per stage to match the previously established 2,000-case comparison; they should not be represented as a measured 8-interval demo pass rate.

## Retest using the current demo core range and orientation

The robustness test was rerun after the demo's core center range and orientation parameterization were updated. It samples the same values as the current demo: `X=[0.30,0.40] m`, `Y=[-0.10,0.10] m`, `Z=[0.30,0.50] m`, with `robomaster_theta_phi_alpha_axis_sequence_ccw_v4`. The run used seed `20260930`, 2,000 cases, 3 intervals per stage (16 target samples), radius `0.1 m`, and the default weights `position=1.0`, `axis=1.55`, other terms `0`. Pass thresholds remain `50 mm` and `30°`, with joint limits included in each pass decision.

| Criterion | Passed | Rate |
|---|---:|---:|
| Average-error pass | 1,653 / 2,000 | 82.65% |
| Peak-error pass (`quality_pass`) | 690 / 2,000 | 34.50% |

The optimizer converged for all 2,000 cases with no planning errors. Mean per-case peak position error was `81.11 mm`; mean per-case peak TCP-X error was `33.96°`. Average runtime was `0.158 s` per case. These are simulated kinematic results for this specific range, seed, path resolution, and weighting; they do not predict physical tracking accuracy. The case data and summary are saved as [test_result_demo_range_center035_z040_2000_seed20260930.csv](./test_result_demo_range_center035_z040_2000_seed20260930.csv) and [test_result_demo_range_center035_z040_2000_seed20260930_summary.csv](./test_result_demo_range_center035_z040_2000_seed20260930_summary.csv).

### Demo-resolution retest (8 intervals per stage)

The same 2,000-case seed and configuration were rerun at the demo's default resolution of 8 intervals per stage (41 target samples per trajectory). The core position ranges and orientation parameterization are unchanged from the preceding retest.

| Criterion | Passed | Rate |
|---|---:|---:|
| Average-error pass | 1,990 / 2,000 | 99.50% |
| Peak-error pass (`quality_pass`) | 1,241 / 2,000 | 62.05% |

All 2,000 cases converged with no planning errors. Mean runtime was `0.271 s` per case (p95 `0.373 s`). The case data and summary are saved as [test_result_demo_range_8interval_2000_seed20260930.csv](./test_result_demo_range_8interval_2000_seed20260930.csv) and [test_result_demo_range_8interval_2000_seed20260930_summary.csv](./test_result_demo_range_8interval_2000_seed20260930_summary.csv).

### Latest demo-range retest

After the demo's X center was changed, the 2,000-case, 8-interval test was rerun with the same seed, thresholds, and weights. The current sampled core bounds are `X=[0.40,0.50] m`, `Y=[-0.10,0.10] m`, and `Z=[0.30,0.50] m`.

| Criterion | Passed | Rate |
|---|---:|---:|
| Average-error pass | 1,993 / 2,000 | 99.65% |
| Peak-error pass (`quality_pass`) | 1,347 / 2,000 | 67.35% |

All cases converged with no planning errors. Mean runtime was `0.266 s` per case (p95 `0.372 s`). Results are saved as [test_result_demo_range_center045_z040_8interval_2000_seed20260930.csv](./test_result_demo_range_center045_z040_8interval_2000_seed20260930.csv) and [test_result_demo_range_center045_z040_8interval_2000_seed20260930_summary.csv](./test_result_demo_range_center045_z040_8interval_2000_seed20260930_summary.csv).

## Weight search

Parameters were screened on the first 200 cases from seed `20260930`. Position weight was initially fixed at `1.0`; smoothness, joint-limit, and manipulability weights remained zero while scanning axis weight.

### TCP-X weight coarse and fine scan

| Position weight | TCP-X weight | Smoothness | Average pass / 200 | Peak pass / 200 |
|---:|---:|---:|---:|---:|
| 1.0 | 0.10 | 0 | 200 | 57 |
| 1.0 | 0.20 | 0 | 200 | 60 |
| 1.0 | 0.35 | 0 | 200 | 63 |
| 1.0 | 0.50 | 0 | 200 | 66 |
| 1.0 | 0.75 | 0 | 200 | 74 |
| 1.0 | 1.00 | 0 | 200 | 81 |
| 1.0 | 1.30 | 0 | 200 | 92 |
| 1.0 | 1.35 | 0 | 200 | 94 |
| 1.0 | 1.40 | 0 | 200 | 94 |
| 1.0 | 1.45 | 0 | 200 | 96 |
| 1.0 | 1.50 | 0 | 200 | 97 |
| 1.0 | 1.55 | 0 | 200 | 97 |
| 1.0 | 1.60 | 0 | 200 | 96 |
| 1.0 | 2.00 | 0 | 200 | 78 |
| 1.0 | 3.00 | 0 | 200 | 54 |

### Smoothness check

At position weight `1.0` and TCP-X weight `1.5`, smoothness values `0`, `0.01`, `0.03`, and `0.05` produced peak pass counts `97`, `96`, `97`, and `97` out of 200. Small smoothness penalties did not provide a consistent improvement, so the selected setting leaves smoothness at zero.

### Position-weight check

With TCP-X weight fixed at `1.55`, tested position weights were `0.5`, `0.75`, `1.0`, `1.25`, `1.5`, `2.0`, and `2.5`. Peak pass counts out of 200 were respectively `50`, `73`, `97`, `90`, `81`, `74`, and `69`. Thus, increasing position weight to reduce position residuals lowered the joint peak pass rate because direction error became more limiting.

Average-error pass was 200/200 for every tested configuration. It was therefore not discriminative for this case distribution and threshold pair; parameter selection was based on peak pass while tracking average errors as a secondary guard.

## Full fixed-seed results

The baseline and tuned runs used the same 2,000 deterministic cases, geometric path configuration, thresholds, code, and non-task weights. Only TCP-X global weight changed.

| Metric | Baseline (`axis=0.1`) | Tuned (`axis=1.55`) |
|---|---:|---:|
| Completed cases | 2,000 | 2,000 |
| Optimizer convergence | 1,981 / 2,000 (99.0%) | 2,000 / 2,000 (100.0%) |
| Planning errors | 0 | 0 |
| Average-error pass | 2,000 / 2,000 (100.0%) | 2,000 / 2,000 (100.0%) |
| Peak-error pass | 600 / 2,000 (30.0%) | 1,020 / 2,000 (51.0%) |
| Mean case planning time | 0.2961 s | 0.1617 s |
| P95 case planning time | 0.4082 s | 0.2104 s |
| Mean of case-average position errors | 0.874 mm | 8.429 mm |
| P95 of case-average position errors | 2.076 mm | 18.381 mm |
| Mean of case-average TCP-X errors | 7.721° | 5.298° |
| P95 of case-average TCP-X errors | 16.798° | 11.463° |
| Mean per-case peak position error | 4.456 mm | 44.764 mm |
| P95 per-case peak position error | 8.701 mm | 76.25 mm |
| Mean per-case peak TCP-X error | 40.15° | 28.41° |
| P95 per-case peak TCP-X error | 71.43° | 50.22° |

The tuned planner gains 420 peak-pass cases (+21 percentage points). Its mean and P95 peak position errors rise substantially, and the P95 peak position error exceeds the `50 mm` threshold. The overall peak pass improves because the baseline's TCP-X errors frequently exceed `30°`, while the tuned weights shift the trade-off toward alignment. This is not a component-wise improvement.

## Independent-seed validation

The selected `1.0/1.55` weights and baseline `1.0/0.1` weights were compared on 300 cases generated with seed `20261019`, not used in the initial parameter scan:

| Criterion | Baseline (`axis=0.1`) | Tuned (`axis=1.55`) |
|---|---:|---:|
| Average-error pass | 300 / 300 (100.0%) | 300 / 300 (100.0%) |
| Peak-error pass | 82 / 300 (27.3%) | 149 / 300 (49.7%) |
| Mean case-average position error | 0.85 mm | 8.39 mm |
| Mean case-average TCP-X error | 7.76° | 5.33° |
| Mean per-case peak position error | 4.37 mm | 45.28 mm |
| Mean per-case peak TCP-X error | 40.75° | 28.95° |

The direction of the peak-pass improvement persists on this holdout, with the same position-versus-axis trade-off. It does not establish performance over all reachable task poses.

## Reproduction

Run from the repository root. Use distinct CSV paths because the test rejects reuse of case files with a different weight configuration:

```bash
# Tuned setting, full fixed-seed test
/usr/bin/python3 src/plan/scripts/III_type_task/end2end_robustness_test.py \
  --csv /tmp/typeiii_tuned_20260930.csv \
  --cases 2000 --seed 20260930 --steps 3 \
  --radius 0.1 --hover-distance 0.20 --slide-distance 0.10 \
  --turn-p-sweep-deg 90 --turn-q-sweep-deg 60 \
  --position-weight 1 --axis-weight 1.55 \
  --smoothness-weight 0 --joint-limit-weight 0 --manipulability-weight 0

# Same cases as baseline comparison
/usr/bin/python3 src/plan/scripts/III_type_task/end2end_robustness_test.py \
  --csv /tmp/typeiii_baseline_20260930.csv \
  --cases 2000 --seed 20260930 --steps 3 \
  --radius 0.1 --hover-distance 0.20 --slide-distance 0.10 \
  --turn-p-sweep-deg 90 --turn-q-sweep-deg 60 \
  --position-weight 1 --axis-weight 0.1 \
  --smoothness-weight 0 --joint-limit-weight 0 --manipulability-weight 0

# Independent-seed validation for the selected setting
/usr/bin/python3 src/plan/scripts/III_type_task/end2end_robustness_test.py \
  --csv /tmp/typeiii_tuned_holdout_20261019.csv \
  --cases 300 --seed 20261019 --steps 3 \
  --radius 0.1 --hover-distance 0.20 --slide-distance 0.10 \
  --turn-p-sweep-deg 90 --turn-q-sweep-deg 60 \
  --position-weight 1 --axis-weight 1.55 \
  --smoothness-weight 0 --joint-limit-weight 0 --manipulability-weight 0
```

Each command writes a case CSV and a companion `*_summary.csv`. The current test also prints separate average-error and peak-error pass rates. The default `--steps` value is 8; `--steps 3` above intentionally reproduces the tested 16-point trajectories.

## Limitations and interpretation

- Results are kinematic forward-kinematics predictions, not measurements of physical execution or controller tracking.
- The planner has no collision checking and does not time-parameterize trajectories.
- Mean-error pass can hide local deviations: it was 100% even when only 30–51% of cases satisfied the peak criterion.
- Peak-error pass is strict and sensitive to a single waypoint; the tuned weight improves total peak pass but worsens position peak statistics.
- Pass rates depend on path resolution: the historical weight-scan tables and the first current-range retest use 3 intervals per stage; the demo-resolution retest above uses 8 intervals per stage. Do not compare or report these as if they used the same trajectory sampling.
- The weight scan tested a finite set of values and seeds. The selected value is an empirical trade-off, not a globally optimal setting.
- Hardware safety limits and acceptance criteria should be checked on measured robot state before execution.
