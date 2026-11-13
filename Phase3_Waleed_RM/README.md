# Lecture 11A/11B wheel-camera experiment

This package contains the reproducible analysis, numerical results, figures, and relevant robot code for ten completed RoboMaster T3 approaches on 13 November 2026. The illustrated PDF report is provided separately.

## Main findings

| Quantity | Result |
|---|---:|
| Completed full trials | 10 |
| Moving rows with both camera and wheel values | 1,621 / 1,621 |
| Mean camera-based travel | 1.474 m |
| Mean wheel-integrated travel | 1.445 m |
| Signed endpoint difference, wheel minus camera | −2.95 cm (between-run SD 2.84 cm) |
| Endpoint RMSE | 3.99 cm |
| Velocity difference RMSE (smoothed camera derivative) | 2.01 cm/s, 1,374 valid pairs |
| Within-run centered velocity-position error correlation | −0.060 |
| Lecture 11B corrected endpoint MAE | 3.51 cm (wheel only: 3.50 cm) |
| Corrected velocity RMSE | 1.92 cm/s (wheel only: 2.01 cm/s) |

Camera depth change is an **external relative position reference**, not an independently calibrated absolute position truth. ToF is used for stopping and quality control, not to compute the position errors. Four later runs included short, recovered tag pauses; stale camera intervals are excluded from the paired velocity analysis.

**Lecture 11B update performed:** the completed runs were replayed offline through a two-state [forward position, forward speed] covariance filter. At each distinct wheel timestamp, the logged command drives prediction, and the wheel-speed reading corrects speed and position using K = P⁻Hᵀ(HP⁻Hᵀ+R)⁻¹. For every held-out run, wheel noise R and process noise Q are estimated from the other nine runs without using camera depth. The camera is used only for scoring. Mean speed gain was 0.925, and mean direct position gain 0.00789 s. Endpoint MAE was 3.495 cm without correction and 3.506 cm after correction; 5/10 runs improved. This is no demonstrated endpoint benefit. Velocity RMSE fell modestly from 2.006 to 1.919 cm/s against smoothed camera-derived speed. A matched-timestamp wheel-only replay also has lower endpoint MAE (3.483 cm) than the correction, so different time grids cannot explain a hidden improvement. The physical pilot was not modified or rerun.

## Package contents

- `lecture11_analysis.py`: Reads an extracted copy of the original input ZIP and reproduces the metrics and figures.
- `lecture11_correction.py`: Applies the offline Lecture 11B weighted speed update and scores it against the original ten runs.
- `build_report.py`: Generates the illustrated PDF from `results/` (requires ReportLab and Pillow).
- `results/run_metrics.csv`: One row per completed full trial.
- `results/aligned_samples.csv`: Uniform 0.1-second analysis grid, including position predictions and validity flags.
- `results/valid_error_pairs.csv`: The 1,374 paired points used for the velocity error scatter and covariance.
- `results/phase3_paired_comparison.csv`: Separate retrospective comparison of ten older filtered Phase 3 trials.
- `results/summary.json`: Aggregates, confidence intervals, speed bands, and smoothing sensitivity.
- `results/correction_run_metrics.csv`, `correction_state_traces.csv`, and `correction_summary.json`: Per-run comparison, every weighted update, calibration, gain, and aggregate comparison.
- `results/error_trajectory_comparison.csv`: Wheel-only and corrected position errors against camera at the same valid times in all ten runs.
- `results/figures/09_error_trajectories_all_runs.png` and `10_error_trajectory_full03.png`: Paired error paths; orange dashed is wheel only and blue solid is corrected. Negative values mean estimated travel is less than camera travel. Camera gaps are masked. The last plotted values can differ from endpoint metrics because the latter use stationary before/after camera medians.
- `results/figures/*.png`: Full-resolution plots in report order.
- `experiment_code/` and `phase3_code/`: Relevant source files copied unchanged from the supplied archive.

## Reproduce

Extract the user's `lecture11_analysis_input.zip` so its root contains `lecture11_wheel_error/` and `logs/`. Install `numpy`, `pandas`, `scipy`, and `matplotlib`, then run:

```bash
python3 lecture11_analysis.py --input-dir /path/to/extracted/root --output-dir results
python3 lecture11_correction.py --input-dir /path/to/extracted/root --baseline-results results --output-dir results
```

To rebuild the PDF after generating results:

```bash
python3 build_report.py
```

The analysis uses distinct camera/wheel message timestamps, a per-run stationary start and end median, explicit Euler integration of four-wheel forward speed, and an approximately 1.1-second quadratic Savitzky–Golay derivative for camera velocity. It masks camera gaps above 0.35 s, including the smoothing half-window, and uses run-level endpoints for the confidence interval.

The earlier Phase 3 filter comparison is **separate** from the new wheel-speed correction. It compares logged filtered position against raw odometry within ten older filtered trials, both relative to camera depth change. The old 8.0 cm marker assumption is retrospectively scaled by the measured 7.3/8.0 width ratio. The earlier filter used odometry and ToF position updates. Its improvement does not describe the new ten-run wheel-speed update.

## Source snapshot

All results derive from the user-supplied archive `lecture11_analysis_input.zip`, SHA-256:

`2db7aeab6367d75f73698ef4751696bb176930a8bd9713b902324c5a716e4b80`

No robot motion was commanded during offline analysis. Aborted 07/08 attempts had been removed before the archive was made.
