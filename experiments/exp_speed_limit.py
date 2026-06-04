"""
Experiment: quasi-static speed limit investigation.

Tests the MPC controller at increasing push speeds to find where the
quasi-static assumption (A=0, no inertial terms) breaks down.

Criterion: inertial force vs friction force.
  Quasi-static valid when m*a << mu*m*g, i.e. a << mu*g ~ 3.4 m/s^2.
  Ratio = max_accel / (mu_ground * g).  >0.1 questionable, >0.5 violated.

Output:
  results/exp_speed_limit/
    speed_limit_summary.png   multi-panel diagnostic plot
    speed_limit_data.npz      raw data for all speeds
"""

from __future__ import annotations

import sys
import time
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
    Log,
    damped_pinv,
    get_jacobian,
    get_pusher_slider_contact_force,
    move_tip_to,
    pusher_in_slider_body,
    slider_pose_from_data,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PUSH_SPEEDS = [0.02, 0.05, 0.08, 0.12, 0.20, 0.35, 0.50]
MPC_V_MAX_FACTOR = 2.5
Y_START = 0.235
Y_GOAL = 0.55
MAX_SIM_TIME = 30.0
MU_GROUND = 0.35
G = 9.81
MU_G = MU_GROUND * G  # ~3.43 m/s^2

OUT_DIR = Path("/workspace/results/exp_speed_limit")
OUT_DIR.mkdir(parents=True, exist_ok=True)


# ---------------------------------------------------------------------------
# Single-speed trial
# ---------------------------------------------------------------------------
def run_single_speed(push_speed: float) -> dict:
    """Run a full MPC push at the given speed, return metrics dict."""
    mpc_v_max = MPC_V_MAX_FACTOR * push_speed

    cfg = Config(
        push_speed=push_speed,
        y_start=Y_START,
        y_goal=Y_GOAL,
        max_sim_time=MAX_SIM_TIME,
        mpc_dt=0.05,
        mpc_horizon=5,
        mpc_v_max=mpc_v_max,
    )

    m = mujoco.MjModel.from_xml_path(cfg.scene_path)
    d = mujoco.MjData(m)

    tip_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pusher_tip_site")
    slider_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "slider")
    key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "ready")
    pusher_geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "pusher_tip")
    slider_geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "slider_geom")

    mujoco.mj_resetDataKeyframe(m, d, key_id)
    mujoco.mj_forward(m, d)

    tip_init = d.site_xpos[tip_site_id].copy()
    slider_pos_init, _ = slider_pose_from_data(d, slider_body_id)

    ctrl = d.ctrl[:6].copy()
    substeps = int(cfg.mpc_dt / m.opt.timestep)

    # --- Phase 0: Approach ---
    slider_face_y = slider_pos_init[1] - 0.03
    approach_target = np.array(
        [slider_pos_init[0], slider_face_y - 0.0005, tip_init[2]]
    )
    ctrl = move_tip_to(
        m, d, tip_site_id, approach_target, ctrl, cfg, renderer=None, max_steps=200
    )
    mujoco.mj_forward(m, d)

    # --- Phase 1: MPC push ---
    mpc = PusherSliderMPC(
        slider_dims=(0.08, 0.06),
        mass=1.05,
        mu_pusher=0.3,
        mu_ground=MU_GROUND,
        dt=cfg.mpc_dt,
        horizon_N=cfg.mpc_horizon,
        Q_weights=np.array([30.0, 10.0, 15.0, 0.1]),
        R_weights=np.array([0.1, 0.1]),
        Q_terminal_scale=10.0,
        v_max=mpc_v_max,
        contact_face="-y",
    )

    # Data collection
    times = []
    slider_xs = []
    slider_ys = []
    slider_thetas = []
    contact_force_norms = []

    t_start = d.time
    tip_z_ref = d.site_xpos[tip_site_id][2]
    step_count = 0

    while True:
        mujoco.mj_forward(m, d)

        tip_pos = d.site_xpos[tip_site_id].copy()
        slider_pos, slider_theta = slider_pose_from_data(d, slider_body_id)
        pusher_body = pusher_in_slider_body(tip_pos, slider_pos, slider_theta)
        cf = get_pusher_slider_contact_force(m, d, pusher_geom_id, slider_geom_id)

        times.append(d.time)
        slider_xs.append(slider_pos[0])
        slider_ys.append(slider_pos[1])
        slider_thetas.append(slider_theta)
        contact_force_norms.append(np.linalg.norm(cf))

        if slider_pos[1] >= cfg.y_goal:
            break
        if d.time - t_start > MAX_SIM_TIME:
            break

        pusher_body_clamped = np.array([np.clip(pusher_body[0], -0.038, 0.038), -0.03])
        target_y_now = min(
            slider_pos[1] + cfg.push_speed * cfg.mpc_dt * cfg.mpc_horizon, cfg.y_goal
        )
        current_target = np.array([0.0, target_y_now, 0.0])

        try:
            vn, vt = mpc.compute_control(
                slider_pose=np.array([slider_pos[0], slider_pos[1], slider_theta]),
                pusher_pos_body=pusher_body_clamped,
                target_pose=current_target,
            )
        except Exception:
            vn, vt = push_speed, 0.0

        vn = max(vn, 0.005)

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

    # --- Compute metrics ---
    times_arr = np.array(times)
    sx = np.array(slider_xs)
    sy = np.array(slider_ys)
    stheta = np.array(slider_thetas)
    cf_norms = np.array(contact_force_norms)

    success = sy[-1] >= (Y_GOAL - 0.03)
    x_drift = abs(sx[-1])
    theta_final_deg = abs(np.degrees(stheta[-1]))
    theta_rms_deg = np.degrees(np.sqrt(np.mean(stheta**2)))
    sim_time = times_arr[-1] - times_arr[0]

    # Contact loss events: force < 0.1 N
    contact_loss_count = int(np.sum(cf_norms < 0.1))

    # Slider velocity and acceleration via finite differences
    dt_arr = np.diff(times_arr)
    dt_arr = np.where(dt_arr < 1e-12, 1e-12, dt_arr)  # avoid div by 0
    vx = np.diff(sx) / dt_arr
    vy = np.diff(sy) / dt_arr

    if len(vx) > 1:
        dt_arr2 = dt_arr[:-1]
        dt_arr2 = np.where(dt_arr2 < 1e-12, 1e-12, dt_arr2)
        ax = np.diff(vx) / dt_arr2
        ay = np.diff(vy) / dt_arr2
        accel_mag = np.sqrt(ax**2 + ay**2)
        max_accel = float(np.max(accel_mag))
    else:
        max_accel = 0.0

    # Quasi-static validity ratio
    qs_ratio = max_accel / MU_G

    # Tracking error: RMS of (slider_y - expected_y)
    # Expected: constant speed from initial y
    expected_y = sy[0] + push_speed * (times_arr - times_arr[0])
    expected_y = np.minimum(expected_y, Y_GOAL)
    tracking_err_rms = float(np.sqrt(np.mean((sy - expected_y) ** 2)))

    return {
        "push_speed": push_speed,
        "mpc_v_max": mpc_v_max,
        "success": success,
        "x_drift": x_drift,
        "theta_final_deg": theta_final_deg,
        "theta_rms_deg": theta_rms_deg,
        "contact_loss_count": contact_loss_count,
        "max_accel": max_accel,
        "qs_ratio": qs_ratio,
        "tracking_err_rms": tracking_err_rms,
        "sim_time": sim_time,
        "step_count": step_count,
        # Trajectories for plotting
        "times": times_arr,
        "slider_x": sx,
        "slider_y": sy,
        "slider_theta": stheta,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    print("=" * 72)
    print("Experiment: Quasi-Static Speed Limit")
    print(f"Speeds: {PUSH_SPEEDS} m/s")
    print(f"Validity threshold: a/(mu*g) > 0.1 questionable, > 0.5 violated")
    print("=" * 72)

    results = []
    for i, speed in enumerate(PUSH_SPEEDS):
        print(f"\n--- [{i + 1}/{len(PUSH_SPEEDS)}] push_speed = {speed:.3f} m/s ---")
        t0 = time.monotonic()
        r = run_single_speed(speed)
        elapsed = time.monotonic() - t0
        tag = "OK" if r["success"] else "FAIL"
        print(
            f"  {tag}  x_drift={r['x_drift']:.4f}m  "
            f"theta_final={r['theta_final_deg']:.2f}deg  "
            f"qs_ratio={r['qs_ratio']:.3f}  "
            f"contact_loss={r['contact_loss_count']}  "
            f"wall_time={elapsed:.1f}s"
        )
        results.append(r)

    # --- Summary table ---
    print("\n" + "=" * 120)
    print(
        f"{'speed':>7s}  {'v_max':>6s}  {'ok?':>4s}  {'x_drift':>8s}  "
        f"{'th_final':>9s}  {'th_rms':>7s}  {'c_loss':>6s}  "
        f"{'max_a':>7s}  {'qs_rat':>7s}  {'trk_rms':>8s}  "
        f"{'sim_t':>6s}  {'steps':>6s}"
    )
    print("-" * 120)
    critical_speed = None
    for r in results:
        flag = " " if r["success"] else "X"
        qs_mark = " "
        if r["qs_ratio"] > 0.5:
            qs_mark = "!!"
        elif r["qs_ratio"] > 0.1:
            qs_mark = " ?"
            if critical_speed is None:
                critical_speed = r["push_speed"]
        print(
            f"{r['push_speed']:7.3f}  {r['mpc_v_max']:6.3f}  "
            f"  {flag}   {r['x_drift']:8.5f}  "
            f"{r['theta_final_deg']:9.3f}  {r['theta_rms_deg']:7.3f}  "
            f"{r['contact_loss_count']:6d}  "
            f"{r['max_accel']:7.3f}  {r['qs_ratio']:6.3f}{qs_mark}  "
            f"{r['tracking_err_rms']:8.5f}  "
            f"{r['sim_time']:6.1f}  {r['step_count']:6d}"
        )

    if critical_speed is None:
        # Find first speed where ratio > 0.1, or report last tested speed
        for r in results:
            if r["qs_ratio"] > 0.1:
                critical_speed = r["push_speed"]
                break
    if critical_speed is None:
        critical_speed = PUSH_SPEEDS[-1]
        print(f"\nNo quasi-static breakdown detected up to {critical_speed} m/s.")
    else:
        print(f"\nCritical speed (qs_ratio > 0.1): ~{critical_speed:.3f} m/s")

    # --- Save data ---
    save_dict = {}
    for i, r in enumerate(results):
        prefix = f"s{i}_"
        save_dict[prefix + "push_speed"] = r["push_speed"]
        save_dict[prefix + "success"] = r["success"]
        save_dict[prefix + "x_drift"] = r["x_drift"]
        save_dict[prefix + "theta_final_deg"] = r["theta_final_deg"]
        save_dict[prefix + "theta_rms_deg"] = r["theta_rms_deg"]
        save_dict[prefix + "contact_loss_count"] = r["contact_loss_count"]
        save_dict[prefix + "max_accel"] = r["max_accel"]
        save_dict[prefix + "qs_ratio"] = r["qs_ratio"]
        save_dict[prefix + "tracking_err_rms"] = r["tracking_err_rms"]
        save_dict[prefix + "sim_time"] = r["sim_time"]
        save_dict[prefix + "slider_x"] = r["slider_x"]
        save_dict[prefix + "slider_y"] = r["slider_y"]
        save_dict[prefix + "slider_theta"] = r["slider_theta"]
        save_dict[prefix + "times"] = r["times"]
    save_dict["push_speeds"] = np.array(PUSH_SPEEDS)
    save_dict["critical_speed"] = critical_speed

    npz_path = OUT_DIR / "speed_limit_data.npz"
    np.savez_compressed(npz_path, **save_dict)
    print(f"Data saved: {npz_path}")

    # --- Plot ---
    plot_results(results, critical_speed)


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_results(results: list[dict], critical_speed: float) -> None:
    speeds = np.array([r["push_speed"] for r in results])

    fig, axes = plt.subplots(2, 3, figsize=(20, 12))

    # Color map for trajectories
    cmap = plt.cm.viridis
    colors = [cmap(i / max(1, len(results) - 1)) for i in range(len(results))]

    # --- Panel 1: x-drift vs push speed ---
    ax = axes[0, 0]
    x_drifts = [r["x_drift"] for r in results]
    ax.plot(speeds, x_drifts, "o-", color="tab:blue", linewidth=1.5, markersize=5)
    ax.axvline(critical_speed, color="red", linestyle="--", alpha=0.6, label="critical")
    ax.set_xlabel("Push speed (m/s)")
    ax.set_ylabel("|x drift| (m)")
    ax.set_title("Lateral drift vs push speed")
    ax.set_xscale("log")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # --- Panel 2: theta error vs push speed ---
    ax = axes[0, 1]
    th_final = [r["theta_final_deg"] for r in results]
    th_rms = [r["theta_rms_deg"] for r in results]
    ax.plot(
        speeds,
        th_final,
        "o-",
        color="tab:red",
        linewidth=1.5,
        markersize=5,
        label="final |theta|",
    )
    ax.plot(
        speeds,
        th_rms,
        "s--",
        color="tab:orange",
        linewidth=1.5,
        markersize=5,
        label="theta RMS",
    )
    ax.axvline(critical_speed, color="red", linestyle="--", alpha=0.6, label="critical")
    ax.set_xlabel("Push speed (m/s)")
    ax.set_ylabel("Angle (deg)")
    ax.set_title("Orientation error vs push speed")
    ax.set_xscale("log")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # --- Panel 3: Contact loss vs push speed ---
    ax = axes[0, 2]
    c_loss = [r["contact_loss_count"] for r in results]
    ax.bar(range(len(speeds)), c_loss, color="tab:purple", alpha=0.7)
    ax.set_xticks(range(len(speeds)))
    ax.set_xticklabels([f"{s:.2f}" for s in speeds], rotation=45, fontsize=7)
    ax.set_xlabel("Push speed (m/s)")
    ax.set_ylabel("Contact loss events (force < 0.1 N)")
    ax.set_title("Contact loss count vs push speed")
    ax.grid(True, alpha=0.3, axis="y")

    # --- Panel 4: Quasi-static validity ratio ---
    ax = axes[1, 0]
    qs_ratios = [r["qs_ratio"] for r in results]
    ax.plot(speeds, qs_ratios, "o-", color="tab:green", linewidth=1.5, markersize=5)
    ax.axhline(
        0.1, color="orange", linestyle="--", linewidth=1.5, label="questionable (0.1)"
    )
    ax.axhline(0.5, color="red", linestyle="--", linewidth=1.5, label="violated (0.5)")
    ax.axvline(
        critical_speed, color="red", linestyle=":", alpha=0.6, label="critical speed"
    )
    ax.set_xlabel("Push speed (m/s)")
    ax.set_ylabel("max_accel / (mu * g)")
    ax.set_title("Quasi-static validity ratio")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # --- Panel 5: Superimposed slider trajectories (x vs y) ---
    ax = axes[1, 1]
    for i, r in enumerate(results):
        label = f"{r['push_speed']:.2f} m/s"
        ax.plot(
            r["slider_x"], r["slider_y"], color=colors[i], linewidth=1.2, label=label
        )
    ax.axvline(0.0, color="gray", linestyle="--", linewidth=0.5)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Slider trajectories (x vs y)")
    ax.legend(fontsize=6, loc="upper left", ncol=2)
    ax.grid(True, alpha=0.3)

    # --- Panel 6: Tracking error & max acceleration ---
    ax = axes[1, 2]
    trk = [r["tracking_err_rms"] for r in results]
    max_a = [r["max_accel"] for r in results]
    ax2 = ax.twinx()
    ln1 = ax.plot(
        speeds,
        trk,
        "o-",
        color="tab:cyan",
        linewidth=1.5,
        markersize=5,
        label="tracking RMS",
    )
    ln2 = ax2.plot(
        speeds,
        max_a,
        "s--",
        color="tab:red",
        linewidth=1.5,
        markersize=5,
        label="max accel",
    )
    ax.axvline(critical_speed, color="red", linestyle="--", alpha=0.6)
    ax.set_xlabel("Push speed (m/s)")
    ax.set_ylabel("Tracking error RMS (m)", color="tab:cyan")
    ax2.set_ylabel("Max acceleration (m/s^2)", color="tab:red")
    ax.set_xscale("log")
    ax.grid(True, alpha=0.3)
    lns = ln1 + ln2
    labs = [l.get_label() for l in lns]
    ax.legend(lns, labs, fontsize=8)

    # Mark critical speed in title
    fig.suptitle(
        f"Quasi-Static Speed Limit Analysis  "
        f"(critical speed ~ {critical_speed:.3f} m/s)",
        fontsize=14,
        fontweight="bold",
    )

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    out_path = OUT_DIR / "speed_limit_summary.png"
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Plot saved: {out_path}")


if __name__ == "__main__":
    main()
