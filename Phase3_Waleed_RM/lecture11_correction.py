#!/usr/bin/env python3
"""Offline Lecture 11B wheel-speed Kalman correction, leave-one-run-out.

The camera is used for evaluation only. For each held-out run, R and Q are
estimated from the other nine runs' wheel and command channels. No camera
values from the held-out run enter its filter or parameter calibration.
"""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import t as student_t

from lecture11_analysis import column_pairs


def robust_std(values):
    a=np.asarray(values,float)
    a=a[np.isfinite(a)]
    if len(a)<20:
        raise ValueError("Too few calibration samples")
    return 1.4826*np.median(np.abs(a-np.median(a)))


def prepare(csv_path):
    d=pd.read_csv(csv_path)
    active=d[d.cmd_v_mps.fillna(0)>.005]
    if len(active)<100:raise ValueError(csv_path)
    on,off=active.odom_stamp_s.iloc[[0,-1]].to_numpy(float)
    tw,wheel=column_pairs(d,"wheel_stamp_s","wheel_forward_mps",
                          "wheel_age_s",.3)
    return {"run":csv_path.name[:6],"path":csv_path,"d":d,"on":on,
            "off":off,"tw":tw,"wheel":wheel}


def held_command(d, times):
    stamps=d.odom_stamp_s.to_numpy(float)
    idx=np.searchsorted(stamps,times,side="right")-1
    return d.cmd_v_mps.fillna(0).to_numpy(float)[np.clip(idx,0,len(d)-1)]


def calibrate(train):
    stationary=[];innovations=[];steps=[]
    for run in train:
        tw=run["tw"];v=run["wheel"]
        u=held_command(run["d"],tw)
        stable=(tw<run["on"]-.3)|(tw>run["off"]+2.0)
        stationary.extend(v[stable])
        dt=np.diff(tw)
        residual=np.diff(v)-np.diff(u)
        chosen=((dt>.055)&(dt<.25)&
                (tw[:-1]>run["on"]+.3)&(tw[1:]<run["off"]-.3))
        innovations.extend(residual[chosen])
        steps.extend(dt[chosen])
    # R is wheel-speed variance at rest. The Q proxy measures unmodeled
    # velocity increments relative to the command changes, converted to
    # acceleration units. Both are robust estimates using training runs only.
    r_std=max(.003,robust_std(stationary))
    delta_std=max(.001,robust_std(innovations))
    a_std=delta_std/float(np.median(steps))
    return {"wheel_std_mps":r_std,"accel_std_mps2":a_std,
            "R_vv_m2ps2":r_std**2,"Q_accel_m2ps4":a_std**2,
            "stationary_samples":len(stationary),
            "increment_samples":len(innovations)}


def replay(run,parameters):
    d=run["d"];on,off=run["on"],run["off"]
    start,finish=on-.3,off+1.7
    stamps=run["tw"]
    used=(stamps>start)&(stamps<=finish)
    events=np.r_[stamps[used],finish]
    x=np.array([0.,0.]) # forward position, forward speed
    matched_wheel_x=0.
    wheel_previous=float(np.interp(start,stamps,run["wheel"]))
    P=np.diag([.02**2,.05**2])
    R=parameters["R_vv_m2ps2"]
    qa2=parameters["Q_accel_m2ps4"]
    time=start
    u_previous=float(held_command(d,np.array([start]))[0])
    output=[]
    for j,new_time in enumerate(events):
        dt=float(new_time-time)
        if dt<=0:continue
        yaw=float(np.interp(time,d.odom_stamp_s,d.yaw_rad))
        u=float(held_command(d,np.array([new_time]))[0])
        matched_wheel_x+=wheel_previous*np.cos(yaw)*dt
        # Lecture 11A prediction. Command change is the input acceleration
        # integrated across this interval, not a position observation.
        x[0]+=x[1]*np.cos(yaw)*dt
        x[1]+=u-u_previous
        F=np.array([[1.,np.cos(yaw)*dt],[0.,1.]])
        G=np.array([[0.],[dt]])
        P=F@P@F.T+G@G.T*qa2
        P=(P+P.T)/2
        if j<len(events)-1: # final boundary is prediction-only
            z=float(np.interp(new_time,stamps,run["wheel"]))
            wheel_previous=z
            innovation=z-x[1]
            S=P[1,1]+R
            K=P[:,1]/S
            x+=K*innovation
            I_KH=np.eye(2)-np.outer(K,np.array([0.,1.]))
            P=I_KH@P@I_KH.T+np.outer(K,K)*R # Joseph form
            P=(P+P.T)/2
            output.append({"run":run["run"],"wall_stamp_s":new_time,
                           "elapsed_s":new_time-on,"corrected_x_m":x[0],
                           "corrected_v_mps":x[1],"innovation_mps":innovation,
                           "gain_x_s":K[0],"gain_v":K[1],
                           "P_xv_m2ps":P[0,1],"P_xx_m2":P[0,0],
                           "wheel_v_mps":z,"command_v_mps":u})
        time=new_time;u_previous=u
    if not np.all(np.linalg.eigvalsh(P)>-1e-10):
        raise ValueError("Non-PSD covariance")
    return float(x[0]),float(matched_wheel_x),pd.DataFrame(output)


def make_figures(metrics,traces,prior_samples,error_trajectories,output):
    figdir=output/"figures";figdir.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({"font.size":10,"axes.spines.top":False,
                         "axes.spines.right":False,"savefig.dpi":210})
    fig,ax=plt.subplots(figsize=(8.2,4.6))
    x=np.arange(len(metrics));w=.37
    ax.bar(x-w/2,abs(metrics.uncorrected_error_m)*100,w,
           color="#DB6B38",label="Wheel-only prediction")
    ax.bar(x+w/2,abs(metrics.corrected_error_m)*100,w,
           color="#176B9A",label="Weighted speed correction")
    ax.set_xticks(x,metrics.run)
    ax.set(ylabel="Absolute endpoint difference vs camera (cm)",
           title="Held-out run comparison: does the weighted correction help?")
    ax.legend(frameon=False);ax.grid(axis="y",alpha=.18)
    fig.tight_layout();fig.savefig(figdir/"06_weighted_correction_endpoints.png")
    plt.close(fig)
    # Representative run with trajectory and gain diagnostics.
    g=traces[traces.run=="full03"]
    b=prior_samples[prior_samples.run=="full03"]
    fig,ax=plt.subplots(figsize=(8.2,4.4))
    ax.plot(b.elapsed_s,b.camera_position_m,color="#176B9A",lw=2.1,
            label="Camera reference")
    ax.plot(b.elapsed_s,b.wheel_prediction_m,color="#DB6B38",lw=1.8,
            label="Wheel-only prediction")
    ax.plot(g.elapsed_s,g.corrected_x_m,color="#2B8A78",lw=1.9,
            label="Corrected position")
    ax.set(xlabel="Time since first forward command (s)",
           ylabel="Forward displacement (m)",
           title="Full03: wheel measurement correction during replay")
    ax.legend(frameon=False);ax.grid(alpha=.18)
    fig.tight_layout();fig.savefig(figdir/"07_corrected_trajectory.png")
    plt.close(fig)
    fig,ax=plt.subplots(figsize=(8.2,3.8))
    other=ax.twinx()
    ax.plot(g.elapsed_s,g.gain_v,color="#176B9A",label="Speed gain K_v")
    other.plot(g.elapsed_s,g.gain_x_s,color="#DB6B38",
               label="Position gain K_x")
    ax.set(xlabel="Time since first forward command (s)",
           ylabel="Speed gain K_v (fraction)",
           title="Kalman weights change as covariance propagates")
    other.set_ylabel("Position gain K_x (s)",color="#DB6B38")
    other.tick_params(axis="y",colors="#DB6B38")
    ax.set_ylim(0,1.05);other.set_ylim(0,.027)
    ax.grid(alpha=.18)
    fig.tight_layout();fig.savefig(figdir/"08_kalman_weights.png")
    plt.close(fig)
    # A positive error means the estimated travel exceeds camera travel.
    # Each line is formed on the same 0.1 s grid and masked at tag gaps.
    fig,axes=plt.subplots(5,2,figsize=(10,11.8),sharex=True,sharey=True)
    for ax,(run,g) in zip(axes.flat,error_trajectories.groupby("run",sort=True)):
        for i,(_,piece) in enumerate(g.groupby(
                (g.elapsed_s.diff().fillna(.1)>.15).cumsum())):
            ax.plot(piece.elapsed_s,piece.corrected_error_cm,
                    color="#176B9A",lw=1.7,
                    label="Weighted correction" if i==0 else None)
            ax.plot(piece.elapsed_s,piece.wheel_only_error_cm,
                    color="#DB6B38",lw=1.6,ls=(0,(3,2)),
                    label="Wheel only" if i==0 else None)
        ax.axhline(0,color="#536779",lw=.85,ls="--",alpha=.6)
        ax.set_title(run,fontweight="bold",loc="left")
        ax.grid(alpha=.15)
    axes[0,0].set_ylim(-10,10)
    for ax in axes[-1,:]:ax.set_xlabel("Time since first forward command (s)")
    for ax in axes[:,0]:ax.set_ylabel("Error vs camera (cm)")
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc="upper center",ncol=2,frameon=False,
               bbox_to_anchor=(.5,.995))
    fig.suptitle("Position error through time: wheel only and corrected",
                 y=1.018,fontsize=14)
    fig.tight_layout(rect=(0,0,1,.975),h_pad=1.15,w_pad=1.4)
    fig.savefig(figdir/"09_error_trajectories_all_runs.png",bbox_inches="tight")
    plt.close(fig)
    g=error_trajectories[error_trajectories.run=="full03"]
    fig,ax=plt.subplots(figsize=(8.2,4.5))
    ax.plot(g.elapsed_s,g.corrected_error_cm,color="#176B9A",lw=2,
            label="Weighted correction")
    ax.plot(g.elapsed_s,g.wheel_only_error_cm,color="#DB6B38",lw=2,
            ls=(0,(3,2)),label="Wheel only")
    ax.axhline(0,color="#536779",lw=1,ls="--")
    ax.set(xlabel="Time since first forward command (s)",
           ylabel="Position error vs camera (cm)",
           title="Full03: the correction barely changes the error path")
    ax.legend(frameon=False);ax.grid(alpha=.18)
    fig.tight_layout();fig.savefig(figdir/"10_error_trajectory_full03.png")
    plt.close(fig)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--input-dir",type=Path,required=True)
    ap.add_argument("--baseline-results",type=Path,required=True)
    ap.add_argument("--output-dir",type=Path,required=True)
    a=ap.parse_args()
    folder=a.input_dir/"lecture11_wheel_error"/"logs"
    runs=[]
    for n in range(1,11):
        files=list(folder.glob(f"full{n:02d}_*.csv"))
        if len(files)!=1:raise ValueError(f"Expected one CSV for full{n:02d}")
        runs.append(prepare(files[0]))
    base=pd.read_csv(a.baseline_results/"run_metrics.csv")
    old_samples=pd.read_csv(a.baseline_results/"aligned_samples.csv")
    metrics=[];traces=[];error_trajectories=[]
    for held in runs:
        train=[r for r in runs if r["run"]!=held["run"]]
        params=calibrate(train)
        end,matched_wheel_end,trace=replay(held,params)
        b=base[base.run==held["run"]].iloc[0]
        camera=b.camera_travel_m
        corrected_error=end-camera
        grid=old_samples[old_samples.run==held["run"]].copy()
        valid_v=grid[grid.velocity_pair_valid.astype(bool)].copy()
        corrected_body_v=np.interp(valid_v.elapsed_s,
                                    trace.elapsed_s,trace.corrected_v_mps)
        relative_yaw=np.interp(held["on"]+valid_v.elapsed_s,
                               held["d"].odom_stamp_s,held["d"].yaw_rad)
        corrected_forward_v=corrected_body_v*np.cos(relative_yaw)
        corrected_v_rmse=float(np.sqrt(np.mean(
            (corrected_forward_v-valid_v.camera_velocity_mps)**2)))
        original_v_rmse=float(np.sqrt(np.mean(
            (valid_v.wheel_velocity_mps-valid_v.camera_velocity_mps)**2)))
        valid_p=grid[(grid.camera_position_valid.astype(bool)) &
                     (grid.elapsed_s>=0)&
                     (grid.elapsed_s<=held["off"]-held["on"]+1.)].copy()
        corrected_x=np.interp(valid_p.elapsed_s,trace.elapsed_s,
                              trace.corrected_x_m)
        valid_p["corrected_position_m"]=corrected_x
        valid_p["wheel_only_error_cm"]=(
            valid_p.wheel_prediction_m-valid_p.camera_position_m)*100
        valid_p["corrected_error_cm"]=(
            valid_p.corrected_position_m-valid_p.camera_position_m)*100
        error_trajectories.append(valid_p[[
            "run","elapsed_s","camera_position_m","wheel_prediction_m",
            "corrected_position_m","wheel_only_error_cm","corrected_error_cm"
        ]])
        metrics.append({
            "run":held["run"],"camera_travel_m":camera,
            "wheel_only_m":b.wheel_travel_m,"corrected_m":end,
            "matched_wheel_only_m":matched_wheel_end,
            "uncorrected_error_m":b.endpoint_error_m,
            "matched_uncorrected_error_m":matched_wheel_end-camera,
            "corrected_error_m":corrected_error,
            "abs_error_change_m":abs(b.endpoint_error_m)-abs(corrected_error),
            "velocity_pairs":len(valid_v),
            "original_velocity_rmse_mps":original_v_rmse,
            "corrected_velocity_rmse_mps":corrected_v_rmse,
            "original_trajectory_rmse_m":float(np.sqrt(np.mean(
                (valid_p.wheel_prediction_m-valid_p.camera_position_m)**2))),
            "corrected_trajectory_rmse_m":float(np.sqrt(np.mean(
                (corrected_x-valid_p.camera_position_m)**2))),
            "calibration_wheel_std_mps":params["wheel_std_mps"],
            "calibration_accel_std_mps2":params["accel_std_mps2"],
            "mean_gain_v":trace.gain_v.mean(),
            "mean_gain_x_s":trace.gain_x_s.mean(),
            "min_P_xx_m2":trace.P_xx_m2.min(),
        })
        traces.append(trace)
    m=pd.DataFrame(metrics);t=pd.concat(traces,ignore_index=True)
    errors=pd.concat(error_trajectories,ignore_index=True)
    un=m.uncorrected_error_m.to_numpy(float);cor=m.corrected_error_m.to_numpy(float)
    matched=m.matched_uncorrected_error_m.to_numpy(float)
    gain=m.abs_error_change_m.to_numpy(float)
    ci=student_t.ppf(.975,9)*gain.std(ddof=1)/np.sqrt(10)
    summary={
        "method":"Offline 2-state forward slice of Lecture 11A/11B; leave-one-run-out command/wheel-only Q,R calibration; camera evaluation only",
        "n_runs":10,"uncorrected_mae_m":float(np.mean(abs(un))),
        "corrected_mae_m":float(np.mean(abs(cor))),
        "uncorrected_rmse_m":float(np.sqrt(np.mean(un**2))),
        "corrected_rmse_m":float(np.sqrt(np.mean(cor**2))),
        "matched_wheel_mae_m":float(np.mean(abs(matched))),
        "matched_wheel_rmse_m":float(np.sqrt(np.mean(matched**2))),
        "original_velocity_rmse_pooled_mps":float(np.sqrt(np.average(
            m.original_velocity_rmse_mps**2,weights=m.velocity_pairs))),
        "corrected_velocity_rmse_pooled_mps":float(np.sqrt(np.average(
            m.corrected_velocity_rmse_mps**2,weights=m.velocity_pairs))),
        "original_trajectory_rmse_run_mean_m":float(
            m.original_trajectory_rmse_m.mean()),
        "corrected_trajectory_rmse_run_mean_m":float(
            m.corrected_trajectory_rmse_m.mean()),
        "mean_corrected_minus_matched_wheel_m":float(
            (m.corrected_m-m.matched_wheel_only_m).mean()),
        "mean_original_minus_matched_wheel_m":float(
            (m.wheel_only_m-m.matched_wheel_only_m).mean()),
        "uncorrected_signed_mean_m":float(un.mean()),
        "corrected_signed_mean_m":float(cor.mean()),
        "paired_mae_improvement_m":float(gain.mean()),
        "paired_improvement_95ci_m":[float(gain.mean()-ci),float(gain.mean()+ci)],
        "runs_improved":int((gain>0).sum()),
        "mean_velocity_gain":float(m.mean_gain_v.mean()),
        "mean_position_gain_s":float(m.mean_gain_x_s.mean()),
        "wheel_std_range_mps":[float(m.calibration_wheel_std_mps.min()),
                               float(m.calibration_wheel_std_mps.max())],
        "accel_std_range_mps2":[float(m.calibration_accel_std_mps2.min()),
                                 float(m.calibration_accel_std_mps2.max())],
    }
    a.output_dir.mkdir(parents=True,exist_ok=True)
    m.to_csv(a.output_dir/"correction_run_metrics.csv",index=False)
    t.to_csv(a.output_dir/"correction_state_traces.csv",index=False)
    errors.to_csv(a.output_dir/"error_trajectory_comparison.csv",index=False)
    (a.output_dir/"correction_summary.json").write_text(json.dumps(summary,indent=2))
    make_figures(m,t,old_samples,errors,a.output_dir)
    print(json.dumps(summary,indent=2))
    print(m[["run","uncorrected_error_m","corrected_error_m",
             "matched_uncorrected_error_m","abs_error_change_m",
             "mean_gain_v","mean_gain_x_s"]]
          .round(4).to_string(index=False))


if __name__=="__main__":main()
