#!/usr/bin/env python3
"""Reproduce the Lecture 11 wheel/camera study from the exported CSV archive.

Camera depth changes are the external *relative* position reference. Camera
velocity is derived from a smoothed depth trace; ToF is only a stopping and
quality-control channel. This script never sends robot commands.
"""

import argparse
import json
import math
import re
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.ndimage import median_filter
from scipy.signal import savgol_filter
from scipy.stats import t as student_t

TAG_SIZE_RATIO_OLD = 0.073 / 0.080
GRID_STEP_S = 0.1
CAMERA_WINDOW = 11  # 1.1 seconds on the 10 Hz analysis grid
CAMERA_POLY = 2
COLORS = ["#176B9A", "#DB6B38", "#4A9D84", "#7C5AA6"]


def csv_data(path):
    return pd.read_csv(path)


def column_pairs(df, stamp, value, age=None, max_age=None):
    cols = [stamp, value] + ([age] if age else [])
    data = df[cols].dropna()
    if age:
        data = data[(data[age] >= 0) & (data[age] < max_age)]
    data = data.drop_duplicates(stamp, keep="last").sort_values(stamp)
    return data[stamp].to_numpy(float), data[value].to_numpy(float)


def camera_gap_mask(grid, stamps, half_window=0.0):
    """Do not invent camera measurements across substantial missing intervals."""
    valid = (grid >= stamps[0]) & (grid <= stamps[-1])
    for a, b in zip(stamps[:-1], stamps[1:]):
        if b - a > 0.35:
            valid &= ~((grid >= a - half_window) &
                       (grid <= b + half_window))
    return valid


def pilot_events(path):
    text = path.read_text()
    lines = text.splitlines()
    stop = next((x for x in lines if "PILOT_STOP_TOF" in x or
                 "PILOT_STOP_CAMERA" in x), "")
    if not stop or "ABORT_" in text:
        raise ValueError(f"Trial did not have a normal stop: {path}")
    pause = [x for x in lines if "PAUSE_TAG_LOST" in x]
    durations = [float(m.group(1)) for x in lines
                 if "RESUME_TAG" in x
                 if (m := re.search(r"paused ([0-9.]+) s", x))]
    first_tracking = next((float(m.group(1)) for x in lines
                           if "TRACKING" in x
                           if (m := re.search(r"\[([0-9]+\.[0-9]+)\]", x))),
                          None)
    return {"stop": "ToF" if "PILOT_STOP_TOF" in stop else "camera",
            "pause_count": len(pause), "pause_s": sum(durations),
            "first_tracking_stamp_s": first_tracking}


def analyse_run(csv_path, pilot_path, camera_window=CAMERA_WINDOW):
    d = csv_data(csv_path)
    if len(d) < 150:
        raise ValueError(f"Insufficient odometry rows: {csv_path}")
    moving = d[d.cmd_v_mps.fillna(0) > 0.005]
    if len(moving) < 100:
        raise ValueError(f"Insufficient moving rows: {csv_path}")
    on, off = moving.odom_stamp_s.iloc[[0, -1]].to_numpy(float)
    start, finish = on - 0.3, off + 1.7
    pre = d[(d.odom_stamp_s >= on - 1.5) &
            (d.odom_stamp_s < on - 0.2)]
    post = d[(d.odom_stamp_s > off + 2.0) &
             (d.odom_stamp_s < off + 6.0)]
    if pre.t3_camera_z_m.count() < 6 or post.t3_camera_z_m.count() < 15:
        raise ValueError(f"Missing stationary camera windows: {csv_path}")
    z0 = float(pre.t3_camera_z_m.median())
    zf = float(post.t3_camera_z_m.median())
    camera_end = z0 - zf
    odom_end = float(post.x_m.median() - pre.x_m.median())
    tof_end = float(pre.tof_m.median() - post.tof_m.median())

    tc, zc = column_pairs(d, "tag_stamp_s", "t3_camera_z_m",
                          "tag_age_s", 0.5)
    tw, vw = column_pairs(d, "wheel_stamp_s", "wheel_forward_mps",
                          "wheel_age_s", 0.3)
    if len(tc) < 100 or len(tw) < 100:
        raise ValueError(f"Missing distinct sensor stamps: {csv_path}")
    grid = np.arange(start, finish + 0.0001, GRID_STEP_S)
    wv = np.interp(grid, tw, vw)
    yaw = np.interp(grid, d.odom_stamp_s, d.yaw_rad)
    vforward = wv * np.cos(yaw)
    dt = np.diff(grid)
    # Explicit Euler prediction, as in Lecture 11A: x[k+1] = x[k]+v[k]cos(theta[k])dt.
    wheel_x = np.r_[0.0, np.cumsum(vforward[:-1] * dt)]

    zgrid = np.interp(grid, tc, zc)
    zmed = median_filter(zgrid, size=3, mode="nearest")
    zsmooth = savgol_filter(zmed, camera_window, CAMERA_POLY,
                           mode="interp")
    cam_v = -savgol_filter(zmed, camera_window, CAMERA_POLY,
                           deriv=1, delta=GRID_STEP_S, mode="interp")
    cam_x = z0 - zsmooth
    e_x = wheel_x - cam_x
    e_v = vforward - cam_v
    camera_valid = camera_gap_mask(grid, tc)
    velocity_valid = camera_gap_mask(
        grid, tc, half_window=(camera_window // 2) * GRID_STEP_S)
    idx = np.searchsorted(d.odom_stamp_s.to_numpy(float), grid,
                          side="right") - 1
    held_command = d.cmd_v_mps.fillna(0).to_numpy(float)[np.clip(idx, 0, len(d)-1)]
    active = ((grid >= on + 0.45) & (grid <= off - 0.35) &
              (held_command > 0.005) & (vforward > 0.01))
    velocity_valid &= active

    frame = pd.DataFrame({
        "run": csv_path.name[:6], "elapsed_s": grid - on,
        "camera_position_m": cam_x,
        "wheel_prediction_m": wheel_x,
        "camera_velocity_mps": cam_v,
        "wheel_velocity_mps": vforward,
        "position_error_m": e_x, "velocity_error_mps": e_v,
        "camera_position_valid": camera_valid,
        "velocity_pair_valid": velocity_valid,
        "command_v_mps": held_command,
    })
    pairs = frame[velocity_valid].copy()
    if len(pairs) < 60:
        raise ValueError(f"Too few valid velocity pairs: {csv_path}")
    valid_pos = frame[camera_valid & (grid >= on) & (grid <= off + 1.0)]
    evt = pilot_events(pilot_path)
    metric = {
        "run": csv_path.name[:6], "csv_file": csv_path.name,
        "pilot_file": pilot_path.name, "stop": evt["stop"],
        "command_on_stamp_s":on,
        "first_tracking_stamp_s":evt["first_tracking_stamp_s"],
        "pause_count": evt["pause_count"], "pause_s": evt["pause_s"],
        "moving_rows": len(moving), "unique_camera_samples": len(tc),
        "unique_wheel_samples": len(tw), "velocity_pairs": len(pairs),
        "motion_duration_s": float(off - on),
        "start_camera_z_m": z0, "final_camera_z_m": zf,
        "camera_travel_m": camera_end,
        "wheel_travel_m": float(wheel_x[-1]),
        "odom_travel_m": odom_end, "tof_change_m_qc_only": tof_end,
        "endpoint_error_m": float(wheel_x[-1] - camera_end),
        "position_rmse_m": float(np.sqrt(np.mean(valid_pos.position_error_m**2))),
        "velocity_bias_mps": float(pairs.velocity_error_mps.mean()),
        "velocity_rmse_mps": float(np.sqrt(np.mean(pairs.velocity_error_mps**2))),
        "error_cov_m2ps": float(np.cov(pairs.velocity_error_mps,
                                      pairs.position_error_m, ddof=1)[0, 1]),
        "error_corr": float(pairs[["velocity_error_mps", "position_error_m"]]
                            .corr().iloc[0, 1]),
        "max_abs_yaw_rad": float(moving.yaw_rad.abs().max()),
        "max_camera_gap_s": float(np.diff(tc).max()),
    }
    return metric, frame, pairs


def old_camera_endpoint(d):
    active = d[d.cmd_v_mps.fillna(0) > 0.005]
    if len(active) < 100:
        raise ValueError("Old run lacks a full active interval")
    on, off = active.t_s.iloc[[0, -1]].to_numpy(float)
    pre = d[(d.t_s >= on - 1.5) & (d.t_s < on - 0.2)]
    post = d[(d.t_s > off + 2) & (d.t_s < off + 7)]
    if pre.t3_camera_z_m.count() < 6 or post.t3_camera_z_m.count() < 15:
        raise ValueError("Old run lacks stationary camera windows")
    camera = float((pre.t3_camera_z_m.median() -
                    post.t3_camera_z_m.median()) * TAG_SIZE_RATIO_OLD)
    odom = float(post.x_m.median() - pre.x_m.median())
    return camera, odom


def prior_phase3(root):
    folder = root / "logs"
    states = list(folder.glob("filtered_state_*.csv"))
    output = []
    for csv_path in sorted(folder.glob("filtered_[0-9][0-9]_*.csv")):
        rec_date = datetime.strptime("_".join(csv_path.stem.split("_")[-2:]),
                                     "%Y%m%d_%H%M%S")
        nearest = min(states, key=lambda f: abs((datetime.strptime(
            "_".join(f.stem.split("_")[-2:]), "%Y%m%d_%H%M%S") - rec_date)
            .total_seconds()))
        separation = abs((datetime.strptime("_".join(nearest.stem.split("_")[-2:]),
                                            "%Y%m%d_%H%M%S")-rec_date).total_seconds())
        if separation > 5:
            raise ValueError(f"Cannot match filter state to {csv_path.name}")
        camera, odom = old_camera_endpoint(csv_data(csv_path))
        state = csv_data(nearest)
        stopped = state[state.controller_state == "STOPPED"]
        if len(stopped) < 20:
            raise ValueError(f"Missing stable filter endpoint: {nearest}")
        filtered = float(stopped.filtered_x_m.tail(20).median())
        output.append({
            "run": csv_path.name.split("_")[1],
            "camera_travel_size_corrected_m": camera,
            "raw_odom_m": odom, "filtered_estimate_m": filtered,
            "raw_error_m": odom - camera,
            "filtered_error_m": filtered - camera,
            "raw_abs_error_m": abs(odom-camera),
            "filtered_abs_error_m": abs(filtered-camera),
            "recorder_csv": csv_path.name, "filter_csv": nearest.name,
        })
    return pd.DataFrame(output)


def aggregate(metrics, paired, prior):
    endpoint = metrics.endpoint_error_m.to_numpy(float)
    mean = float(endpoint.mean()); sd = float(endpoint.std(ddof=1))
    ci = student_t.ppf(.975, len(endpoint)-1)*sd/math.sqrt(len(endpoint))
    center = paired.copy()
    center["ex_centered"] = center.position_error_m - center.groupby("run").position_error_m.transform("mean")
    center["ev_centered"] = center.velocity_error_mps - center.groupby("run").velocity_error_mps.transform("mean")
    cov = float(np.cov(center.ev_centered, center.ex_centered, ddof=1)[0, 1])
    corr = float(np.corrcoef(center.ev_centered, center.ex_centered)[0, 1])
    phase3_gain = prior.raw_abs_error_m-prior.filtered_abs_error_m
    phase3_ci = (student_t.ppf(.975, len(prior)-1) *
                 phase3_gain.std(ddof=1)/math.sqrt(len(prior)))
    speed_bands = {}
    for name, group in (("low_command_at_most_0.07_mps",
                         paired[paired.command_v_mps <= 0.07]),
                        ("normal_command_above_0.07_mps",
                         paired[paired.command_v_mps > 0.07])):
        speed_bands[name] = {
            "pairs": len(group),
            "velocity_rmse_mps": float(np.sqrt(np.mean(group.velocity_error_mps**2))),
            "position_rmse_m": float(np.sqrt(np.mean(group.position_error_m**2))),
        }
    return {
        "new_runs": len(metrics), "velocity_pairs": len(paired),
        "camera_travel_mean_m": float(metrics.camera_travel_m.mean()),
        "wheel_travel_mean_m": float(metrics.wheel_travel_m.mean()),
        "endpoint_error_mean_m": mean, "endpoint_error_sd_m": sd,
        "endpoint_error_95ci_mean_m": [mean-ci, mean+ci],
        "endpoint_error_rmse_m": float(np.sqrt(np.mean(endpoint**2))),
        "endpoint_error_mae_m": float(np.mean(abs(endpoint))),
        "endpoint_error_percent_of_camera_mean": float(np.mean(
            metrics.endpoint_error_m/metrics.camera_travel_m)*100),
        "odom_endpoint_error_mean_m":float((metrics.odom_travel_m-
                                             metrics.camera_travel_m).mean()),
        "wheel_minus_odom_mean_m":float((metrics.wheel_travel_m-
                                         metrics.odom_travel_m).mean()),
        "velocity_error_mean_mps": float(paired.velocity_error_mps.mean()),
        "velocity_error_rmse_mps": float(np.sqrt(np.mean(paired.velocity_error_mps**2))),
        "within_run_covariance_m2ps": cov,
        "within_run_correlation": corr,
        "prior_phase3_raw_mae_m": float(prior.raw_abs_error_m.mean()),
        "prior_phase3_filtered_mae_m": float(prior.filtered_abs_error_m.mean()),
        "prior_phase3_paired_mae_change_m": float(
            phase3_gain.mean()),
        "prior_phase3_gain_95ci_mean_m": [float(phase3_gain.mean()-phase3_ci),
                                          float(phase3_gain.mean()+phase3_ci)],
        "prior_phase3_runs_improved":int((phase3_gain>0).sum()),
        "speed_bands":speed_bands,
    }


def make_figures(metrics, samples, paired, prior, out):
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False, "figure.dpi": 140,
                         "savefig.dpi": 200})
    figdir = out / "figures"; figdir.mkdir(parents=True, exist_ok=True)
    representative = samples[samples.run == "full03"]
    fig, ax = plt.subplots(figsize=(8.3, 4.2))
    ax.plot(representative.elapsed_s, representative.camera_position_m,
            color=COLORS[0], lw=2.4, label="Camera displacement (smoothed)")
    ax.plot(representative.elapsed_s, representative.wheel_prediction_m,
            color=COLORS[1], lw=2.2, label="Integrated wheel prediction")
    r3=metrics[metrics.run=="full03"].iloc[0]
    ref_tau=np.clip((r3.command_on_stamp_s+representative.elapsed_s-
                     r3.first_tracking_stamp_s)/20.0,0,1)
    nominal_ref=2.0*(3*ref_tau**2-2*ref_tau**3)
    ax.plot(representative.elapsed_s,nominal_ref,color="#79818B",
            ls="--",lw=1.5,label="Nominal cubic plan (before speed cap)")
    ax.set(xlabel="Time since first forward command (s)",
           ylabel="Forward displacement (m)",
           title="One run: camera, wheel prediction, and nominal plan")
    ax.legend(frameon=False); ax.grid(alpha=.18)
    fig.tight_layout();fig.savefig(figdir/"01_position_trace.png");plt.close(fig)

    fig, axes = plt.subplots(1,2,figsize=(10.5,4.0))
    for run, g in samples.groupby("run"):
        sel = g[g.camera_position_valid & (g.elapsed_s >= 0)]
        axes[0].plot(sel.camera_position_m, sel.position_error_m*100,
                     lw=1.25,alpha=.65,label=run)
    axes[0].axhline(0,color="#606770",lw=.8)
    axes[0].set(xlabel="Camera displacement (m)",ylabel="Wheel minus camera (cm)",
                title="Position error grows along the path")
    axes[0].grid(alpha=.18)
    axes[1].bar(metrics.run, metrics.endpoint_error_m*100,color=COLORS[1])
    axes[1].axhline(0,color="#606770",lw=.8)
    axes[1].tick_params(axis="x",rotation=50)
    axes[1].set(ylabel="Endpoint error (cm)",title="Ten independent endpoints")
    fig.tight_layout();fig.savefig(figdir/"02_position_errors.png");plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.2,5.0))
    for run,g in paired.groupby("run"):
        ax.scatter(g.velocity_error_mps*100,g.position_error_m*100,
                   s=10,alpha=.28,label=run)
    ax.axhline(0,color="#777",lw=.8);ax.axvline(0,color="#777",lw=.8)
    ax.set(xlabel="Wheel minus smoothed camera velocity (cm/s)",
           ylabel="Wheel minus camera position (cm)",
           title="Paired velocity and position errors")
    ax.grid(alpha=.18)
    ax.legend(ncol=5,loc="upper center",bbox_to_anchor=(.5,-.13),
              frameon=False,fontsize=8)
    fig.tight_layout();fig.savefig(figdir/"03_error_scatter.png",bbox_inches="tight");plt.close(fig)

    fig,axes=plt.subplots(1,2,figsize=(10.5,4.0))
    p=paired[paired.run=="full03"]
    axes[0].plot(p.elapsed_s,p.wheel_velocity_mps,color=COLORS[1],
                 label="Wheel velocity",lw=1.8)
    axes[0].plot(p.elapsed_s,p.camera_velocity_mps,color=COLORS[0],
                 label="Camera-derived velocity",lw=1.8)
    axes[0].set(xlabel="Time since first forward command (s)",
                ylabel="Forward velocity (m/s)",title="Velocity from two sensors")
    axes[0].legend(frameon=False,fontsize=8);axes[0].grid(alpha=.18)
    axes[1].scatter(metrics.run,metrics.velocity_rmse_mps*100,
                    color=COLORS[2],s=42)
    axes[1].plot(metrics.run,metrics.velocity_rmse_mps*100,
                 color=COLORS[2],alpha=.5)
    axes[1].tick_params(axis="x",rotation=50)
    axes[1].set(ylabel="Velocity error RMSE (cm/s)",
                title="Velocity discrepancy by run")
    fig.tight_layout();fig.savefig(figdir/"04_velocity.png");plt.close(fig)

    fig,ax=plt.subplots(figsize=(7.7,4.1))
    x=np.arange(len(prior));width=.36
    ax.bar(x-width/2,prior.raw_abs_error_m*100,width,
           label="Raw odometry",color=COLORS[1])
    ax.bar(x+width/2,prior.filtered_abs_error_m*100,width,
           label="Earlier Phase 3 filter",color=COLORS[0])
    ax.set_xticks(x,[f"{int(x):02d}" for x in prior.run])
    ax.set(xlabel="Earlier filtered trial",ylabel="Absolute error vs camera (cm)",
           title="Retrospective Phase 3 position correction comparison")
    ax.legend(frameon=False);ax.grid(axis="y",alpha=.18)
    fig.tight_layout();fig.savefig(figdir/"05_phase3_comparison.png");plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir",type=Path,required=True,
                        help="Extracted archive root containing lecture11_wheel_error/ and logs/")
    parser.add_argument("--output-dir",type=Path,required=True)
    args=parser.parse_args()
    out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
    folder=args.input_dir/"lecture11_wheel_error"/"logs"
    metrics=[];frames=[];pairs=[]
    for i in range(1,11):
        label=f"full{i:02d}"
        candidates=list(folder.glob(f"{label}_*.csv"))
        logs=list(folder.glob(f"{label}_*_pilot.txt"))
        if len(candidates)!=1 or len(logs)!=1:
            raise ValueError(f"Expected one completed CSV and pilot log for {label}")
        metric,frame,pair=analyse_run(candidates[0],logs[0])
        metrics.append(metric);frames.append(frame);pairs.append(pair)
    metric_df=pd.DataFrame(metrics);sample_df=pd.concat(frames,ignore_index=True)
    pair_df=pd.concat(pairs,ignore_index=True)
    prior=prior_phase3(args.input_dir)
    if len(prior)!=10: raise ValueError("Expected ten paired older filtered trials")
    summary=aggregate(metric_df,pair_df,prior)
    sensitivity={}
    for w in (7,11,15):
        results=[analyse_run(list(folder.glob(f"full{i:02d}_*.csv"))[0],
                             list(folder.glob(f"full{i:02d}_*_pilot.txt"))[0],
                             camera_window=w)[2]
                 for i in range(1,11)]
        both=pd.concat(results)
        sensitivity[str(w)]={
            "velocity_rmse_mps":float(np.sqrt(np.mean(both.velocity_error_mps**2))),
            "within_run_cov_m2ps":float(np.cov(
                both.velocity_error_mps-both.groupby("run").velocity_error_mps.transform("mean"),
                both.position_error_m-both.groupby("run").position_error_m.transform("mean"),
                ddof=1)[0,1]),
            "pairs":len(both)}
    summary["smoothing_sensitivity"]=sensitivity
    metric_df.to_csv(out/"run_metrics.csv",index=False)
    sample_df.to_csv(out/"aligned_samples.csv",index=False)
    pair_df.to_csv(out/"valid_error_pairs.csv",index=False)
    prior.to_csv(out/"phase3_paired_comparison.csv",index=False)
    (out/"summary.json").write_text(json.dumps(summary,indent=2))
    make_figures(metric_df,sample_df,pair_df,prior,out)
    print(json.dumps(summary,indent=2))
    print(metric_df[["run","camera_travel_m","wheel_travel_m",
                     "endpoint_error_m","velocity_rmse_mps","pause_count"]]
          .round(4).to_string(index=False))


if __name__=="__main__":
    main()
