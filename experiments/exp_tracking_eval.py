"""
Experiment: MPC trajectory tracking evaluation.

Runs four reference trajectories through the full MuJoCo pusher-slider simulation
and evaluates tracking performance with quantitative metrics and plots.

Trajectories:
  a. Straight line:   y=0.235->0.78, x=0, theta=0
  b. Diagonal line:   y=0.235->0.78, x shifts 0->+0.05m
  c. S-curve:         y=0.235->0.78, x = 0.03*sin(2*pi*(y-y0)/(y1-y0))
  d. With rotation:   y=0.235->0.78, theta ramps 0->10 deg

Output: /workspace/results/exp_tracking_eval/
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

sys.path.insert(0, "/workspace")
from pusher_slider_mpc import PusherSliderMPC
from run_stage1 import (
    Config,
    damped_pinv,
    get_jacobian,
    move_tip_to,
    pusher_in_slider_body,
    slider_pose_from_data,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
Y_START = 0.235
Y_GOAL = 0.78
Y_RANGE = Y_GOAL - Y_START
OUTPUT_DIR = Path("/workspace/results/exp_tracking_eval")


# ---------------------------------------------------------------------------
# Reference trajectory definitions
# ---------------------------------------------------------------------------
def ref_straight(y: float) -> tuple[float, float, float]:
    """Straight line: x=0, theta=0."""
    return 0.0, y, 0.0


def ref_diagonal(y: float) -> tuple[float, float, float]:
    """Diagonal: x shifts 0 -> +0.05 m linearly with y progress."""
    progress = (y - Y_START) / Y_RANGE
    progress = np.clip(progress, 0.0, 1.0)
    x = 0.05 * progress
    return float(x), y, 0.0


def ref_scurve(y: float) -> tuple[float, float, float]:
    """S-curve: x = 0.03*sin(2*pi*(y-y0)/(y1-y0))."""
    progress = (y - Y_START) / Y_RANGE
    x = 0.03 * np.sin(2.0 * np.pi * progress)
    return float(x), y, 0.0


def ref_rotation(y: float) -> tuple[float, float, float]:
    """With rotation: theta ramps 0 -> 10 deg linearly."""
    progress = (y - Y_START) / Y_RANGE
    progress = np.clip(progress, 0.0, 1.0)
    theta = np.deg2rad(10.0) * progress
    return 0.0, y, float(theta)


TRAJECTORIES = {
    "straight": ref_straight,
    "diagonal": ref_diagonal,
    "s-curve": ref_scurve,
    "rotation": ref_rotation,
}


# ---------------------------------------------------------------------------
# Per-run data logger
# ---------------------------------------------------------------------------
@dataclass
class TrackingLog:
    time: list[float] = field(default_factory=list)
    slider_x: list[float] = field(default_factory=list)
    slider_y: list[float] = field(default_factory=list)
    slider_theta: list[float] = field(default_factory=list)
    ref_x: list[float] = field(default_factory=list)
    ref_y: list[float] = field(default_factory=list)
    ref_theta: list[float] = field(default_factory=list)
    vn: list[float] = field(default_factory=list)
    vt: list[float] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Metrics computation
# ---------------------------------------------------------------------------
@dataclass
class Metrics:
    name: str
    rms_x: float
    rms_y: float
    rms_theta_deg: float
    max_x: float
    max_y: float
    max_theta_deg: float
    control_effort: float
    smoothness: float
    completion_time: float
    final_err_x: float
    final_err_y: float
    final_err_theta_deg: float


def compute_metrics(name: str, log: TrackingLog, dt: float) -> Metrics:
    sx = np.array(log.slider_x)
    sy = np.array(log.slider_y)
    st = np.array(log.slider_theta)
    rx = np.array(log.ref_x)
    ry = np.array(log.ref_y)
    rt = np.array(log.ref_theta)
    vn = np.array(log.vn)
    vt_arr = np.array(log.vt)
    t = np.array(log.time)

    ex = sx - rx
    ey = sy - ry
    et = st - rt

    rms_x = float(np.sqrt(np.mean(ex**2)))
    rms_y = float(np.sqrt(np.mean(ey**2)))
    rms_theta = float(np.sqrt(np.mean(et**2)))

    max_x = float(np.max(np.abs(ex)))
    max_y = float(np.max(np.abs(ey)))
    max_theta = float(np.max(np.abs(et)))

    effort = float(np.sum(vn**2 + vt_arr**2) * dt)

    if len(vn) > 1:
        dvn = np.diff(vn)
        dvt = np.diff(vt_arr)
        smoothness = float(np.sum(dvn**2 + dvt**2))
    else:
        smoothness = 0.0

    completion_time = float(t[-1] - t[0]) if len(t) > 1 else 0.0

    final_ex = float(abs(sx[-1] - rx[-1]))
    final_ey = float(abs(sy[-1] - ry[-1]))
    final_et = float(abs(st[-1] - rt[-1]))

    return Metrics(
        name=name,
        rms_x=rms_x,
        rms_y=rms_y,
        rms_theta_deg=np.degrees(rms_theta),
        max_x=max_x,
        max_y=max_y,
        max_theta_deg=np.degrees(max_theta),
        control_effort=effort,
        smoothness=smoothness,
        completion_time=completion_time,
        final_err_x=final_ex,
        final_err_y=final_ey,
        final_err_theta_deg=np.degrees(final_et),
    )


# ---------------------------------------------------------------------------
# Single trajectory simulation
# ---------------------------------------------------------------------------
def run_trajectory(
    traj_name: str,
    ref_fn: callable,
    cfg: Config,
) -> TrackingLog:
    """Run a full MuJoCo simulation with MPC tracking a given reference trajectory."""
    m = mujoco.MjModel.from_xml_path(cfg.scene_path)
    d = mujoco.MjData(m)

    tip_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pusher_tip_site")
    slider_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "slider")
    key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "ready")
    pusher_geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "pusher_tip")
    slider_geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "slider_geom")

    mujoco.mj_resetDataKeyframe(m, d, key_id)
    mujoco.mj_forward(m, d)

    ctrl = d.ctrl[:6].copy()
    substeps = int(cfg.mpc_dt / m.opt.timestep)

    # --- Phase 0: Approach ---
    slider_pos_init, _ = slider_pose_from_data(d, slider_body_id)
    tip_init = d.site_xpos[tip_site_id].copy()
    slider_face_y = slider_pos_init[1] - 0.03
    approach_target = np.array(
        [slider_pos_init[0], slider_face_y - 0.0005, tip_init[2]]
    )
    ctrl = move_tip_to(
        m, d, tip_site_id, approach_target, ctrl, cfg, None, max_steps=200
    )
    mujoco.mj_forward(m, d)

    # --- Phase 1: MPC push ---
    mpc = PusherSliderMPC(
        slider_dims=(0.08, 0.06),
        mass=1.05,
        mu_pusher=0.3,
        mu_ground=0.35,
        dt=cfg.mpc_dt,
        horizon_N=cfg.mpc_horizon,
        Q_weights=np.array([30.0, 10.0, 15.0, 0.1]),
        R_weights=np.array([0.1, 0.1]),
        Q_terminal_scale=10.0,
        v_max=cfg.mpc_v_max,
        contact_face="-y",
    )

    log = TrackingLog()
    t_start = d.time
    step_count = 0
    tip_z_ref = d.site_xpos[tip_site_id][2]

    while True:
        mujoco.mj_forward(m, d)

        tip_pos = d.site_xpos[tip_site_id].copy()
        slider_pos, slider_theta = slider_pose_from_data(d, slider_body_id)
        pusher_body = pusher_in_slider_body(tip_pos, slider_pos, slider_theta)

        # Compute reference at current slider y
        ref_x, ref_y, ref_theta = ref_fn(slider_pos[1])

        log.time.append(d.time)
        log.slider_x.append(slider_pos[0])
        log.slider_y.append(slider_pos[1])
        log.slider_theta.append(slider_theta)
        log.ref_x.append(ref_x)
        log.ref_y.append(ref_y)
        log.ref_theta.append(ref_theta)

        if slider_pos[1] >= cfg.y_goal:
            log.vn.append(0.0)
            log.vt.append(0.0)
            break
        if d.time - t_start > cfg.max_sim_time:
            log.vn.append(0.0)
            log.vt.append(0.0)
            break

        pusher_body_clamped = np.array([np.clip(pusher_body[0], -0.038, 0.038), -0.03])

        # MPC target: current reference pose (look ahead by horizon for y)
        target_y_now = min(
            slider_pos[1] + cfg.push_speed * cfg.mpc_dt * cfg.mpc_horizon, cfg.y_goal
        )
        # Get reference at the look-ahead y for x and theta
        ref_x_ahead, _, ref_theta_ahead = ref_fn(target_y_now)
        current_target = np.array([ref_x_ahead, target_y_now, ref_theta_ahead])

        try:
            vn, vt = mpc.compute_control(
                slider_pose=np.array([slider_pos[0], slider_pos[1], slider_theta]),
                pusher_pos_body=pusher_body_clamped,
                target_pose=current_target,
            )
        except Exception as e:
            print(f"  MPC failed at step {step_count} ({traj_name}): {e}")
            vn, vt = cfg.push_speed, 0.0

        vn = max(vn, 0.005)
        log.vn.append(vn)
        log.vt.append(vt)

        v_world_xy = mpc.contact_to_world(vn, vt, slider_theta)
        z_error = tip_z_ref - tip_pos[2]
        v_des_3d = np.array([v_world_xy[0], v_world_xy[1], 5.0 * z_error])

        J = get_jacobian(m, d, tip_site_id)
        dq = damped_pinv(J, cfg.damping) @ (v_des_3d * cfg.mpc_dt)
        ctrl = ctrl + dq
        d.ctrl[:6] = ctrl

        for _ in range(substeps):
            mujoco.mj_step(m, d)

        step_count += 1
        if step_count % 200 == 0:
            print(
                f"  [{traj_name}] step={step_count}  t={d.time:.2f}s"
                f"  slider=({slider_pos[0]:.4f}, {slider_pos[1]:.4f})"
                f"  ref=({ref_x:.4f}, {ref_y:.4f})"
            )

    print(f"  [{traj_name}] done: {step_count} steps, {d.time - t_start:.2f}s sim time")
    return log


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_trajectories(
    results: dict[str, tuple[TrackingLog, Metrics]],
    output_dir: Path,
) -> None:
    """2x2 subplot: actual vs reference trajectory with orientation arrows."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    traj_names = list(results.keys())

    for idx, name in enumerate(traj_names):
        ax = axes[idx // 2, idx % 2]
        log, metrics = results[name]

        sx = np.array(log.slider_x)
        sy = np.array(log.slider_y)
        st = np.array(log.slider_theta)
        rx = np.array(log.ref_x)
        ry = np.array(log.ref_y)
        rt = np.array(log.ref_theta)

        ax.plot(rx, ry, "r--", linewidth=1.5, label="Reference", zorder=2)
        ax.plot(sx, sy, "b-", linewidth=1.2, label="Actual", zorder=3)

        # Orientation arrows at intervals
        n_arrows = min(15, len(sx))
        if n_arrows > 1:
            arrow_idx = np.linspace(0, len(sx) - 1, n_arrows, dtype=int)
            arrow_len = 0.012
            for i in arrow_idx:
                # Actual
                dx_a = arrow_len * np.cos(st[i])
                dy_a = arrow_len * np.sin(st[i])
                ax.annotate(
                    "",
                    xy=(sx[i] + dx_a, sy[i] + dy_a),
                    xytext=(sx[i], sy[i]),
                    arrowprops=dict(arrowstyle="->", color="blue", lw=0.8),
                )
                # Reference
                dx_r = arrow_len * np.cos(rt[i])
                dy_r = arrow_len * np.sin(rt[i])
                ax.annotate(
                    "",
                    xy=(rx[i] + dx_r, ry[i] + dy_r),
                    xytext=(rx[i], ry[i]),
                    arrowprops=dict(arrowstyle="->", color="red", lw=0.8),
                )

        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title(
            f"{name}\n"
            f"RMS: x={metrics.rms_x * 1000:.1f}mm, y={metrics.rms_y * 1000:.1f}mm, "
            f"theta={metrics.rms_theta_deg:.2f} deg",
            fontsize=10,
        )
        ax.legend(loc="upper left", fontsize=8)
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(output_dir / "trajectories.png", dpi=150)
    plt.close(fig)
    print(f"Saved: {output_dir / 'trajectories.png'}")


def plot_metrics_bars(
    results: dict[str, tuple[TrackingLog, Metrics]],
    output_dir: Path,
) -> None:
    """Bar chart comparing metrics across trajectories."""
    names = list(results.keys())
    metrics_list = [results[n][1] for n in names]

    metric_groups = {
        "RMS tracking error (mm)": {
            "x": [m.rms_x * 1000 for m in metrics_list],
            "y": [m.rms_y * 1000 for m in metrics_list],
        },
        "RMS theta error (deg)": {
            "theta": [m.rms_theta_deg for m in metrics_list],
        },
        "Max tracking error (mm)": {
            "x": [m.max_x * 1000 for m in metrics_list],
            "y": [m.max_y * 1000 for m in metrics_list],
        },
        "Max theta error (deg)": {
            "theta": [m.max_theta_deg for m in metrics_list],
        },
        "Control effort": {
            "effort": [m.control_effort for m in metrics_list],
        },
        "Smoothness (jerk proxy)": {
            "smoothness": [m.smoothness for m in metrics_list],
        },
        "Completion time (s)": {
            "time": [m.completion_time for m in metrics_list],
        },
        "Final pose error (mm / deg)": {
            "x (mm)": [m.final_err_x * 1000 for m in metrics_list],
            "y (mm)": [m.final_err_y * 1000 for m in metrics_list],
            "theta (deg)": [m.final_err_theta_deg for m in metrics_list],
        },
    }

    n_groups = len(metric_groups)
    fig, axes = plt.subplots(2, 4, figsize=(20, 8))
    axes = axes.flatten()

    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    x_pos = np.arange(len(names))

    for ax_idx, (group_name, sub_metrics) in enumerate(metric_groups.items()):
        ax = axes[ax_idx]
        n_sub = len(sub_metrics)
        width = 0.8 / n_sub

        for s_idx, (sub_name, values) in enumerate(sub_metrics.items()):
            offset = (s_idx - (n_sub - 1) / 2) * width
            bars = ax.bar(
                x_pos + offset, values, width, label=sub_name, color=colors[s_idx]
            )
            for bar, val in zip(bars, values):
                ax.text(
                    bar.get_x() + bar.get_width() / 2,
                    bar.get_height(),
                    f"{val:.2f}",
                    ha="center",
                    va="bottom",
                    fontsize=6,
                )

        ax.set_xticks(x_pos)
        ax.set_xticklabels(names, rotation=30, ha="right", fontsize=8)
        ax.set_title(group_name, fontsize=9)
        if n_sub > 1:
            ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    fig.savefig(output_dir / "metrics_comparison.png", dpi=150)
    plt.close(fig)
    print(f"Saved: {output_dir / 'metrics_comparison.png'}")


# ---------------------------------------------------------------------------
# Summary table
# ---------------------------------------------------------------------------
def print_summary(results: dict[str, tuple[TrackingLog, Metrics]]) -> None:
    """Print a formatted summary table of all metrics."""
    names = list(results.keys())
    metrics_list = [results[n][1] for n in names]

    header = (
        f"{'Trajectory':<12} | "
        f"{'RMS_x':>7} {'RMS_y':>7} {'RMS_th':>7} | "
        f"{'Max_x':>7} {'Max_y':>7} {'Max_th':>7} | "
        f"{'Effort':>8} {'Smooth':>8} | "
        f"{'Time':>6} | "
        f"{'Fin_x':>7} {'Fin_y':>7} {'Fin_th':>7}"
    )
    units = (
        f"{'':.<12} | "
        f"{'(mm)':>7} {'(mm)':>7} {'(deg)':>7} | "
        f"{'(mm)':>7} {'(mm)':>7} {'(deg)':>7} | "
        f"{'':>8} {'':>8} | "
        f"{'(s)':>6} | "
        f"{'(mm)':>7} {'(mm)':>7} {'(deg)':>7}"
    )

    sep = "-" * len(header)
    print(f"\n{'=' * len(header)}")
    print("TRACKING EVALUATION RESULTS")
    print(f"{'=' * len(header)}")
    print(header)
    print(units)
    print(sep)

    for name, m in zip(names, metrics_list):
        row = (
            f"{name:<12} | "
            f"{m.rms_x * 1000:7.2f} {m.rms_y * 1000:7.2f} {m.rms_theta_deg:7.2f} | "
            f"{m.max_x * 1000:7.2f} {m.max_y * 1000:7.2f} {m.max_theta_deg:7.2f} | "
            f"{m.control_effort:8.4f} {m.smoothness:8.4f} | "
            f"{m.completion_time:6.2f} | "
            f"{m.final_err_x * 1000:7.2f} {m.final_err_y * 1000:7.2f} "
            f"{m.final_err_theta_deg:7.2f}"
        )
        print(row)

    print(sep)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    cfg = Config()

    results: dict[str, tuple[TrackingLog, Metrics]] = {}

    for traj_name, ref_fn in TRAJECTORIES.items():
        print(f"\n{'=' * 60}")
        print(f"Running trajectory: {traj_name}")
        print(f"{'=' * 60}")

        t0 = time.monotonic()
        log = run_trajectory(traj_name, ref_fn, cfg)
        wall_time = time.monotonic() - t0
        print(f"  Wall time: {wall_time:.1f}s")

        metrics = compute_metrics(traj_name, log, cfg.mpc_dt)
        results[traj_name] = (log, metrics)

    print_summary(results)

    print("\nGenerating plots...")
    plot_trajectories(results, OUTPUT_DIR)
    plot_metrics_bars(results, OUTPUT_DIR)
    print(f"\nAll outputs saved to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
