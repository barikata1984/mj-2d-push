"""
Experiment: effect of non-uniform pressure distribution on pusher-slider MPC.

Non-uniform pressure is modelled by shifting the slider's center of mass (CoM)
in MuJoCo. For each CoM configuration we run MPC push in two modes:

  - Aware:   MPC uses a corrected limit-surface parameter c that accounts for
             the shifted pressure centroid (parallel axis theorem).
  - Unaware: MPC uses the default c computed for uniform pressure -- mismatch
             with the actual physics.

Metrics: final |Delta-theta|, x-drift, success, theta oscillation.
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from pusher_slider import paths
from pusher_slider.config import SimConfig
from pusher_slider.controllers import PusherSliderMPC
from pusher_slider.kinematics import (
    damped_pinv,
    get_jacobian,
    pusher_in_slider_body,
    slider_pose_from_data,
)

# ---------------------------------------------------------------------------
# Physical constants
# ---------------------------------------------------------------------------
SLIDER_A = 0.08  # half-size along x is 0.04; full size 80 mm
SLIDER_B = 0.06  # half-size along y is 0.03; full size 60 mm
SLIDER_MASS = 1.05
SLIDER_HALF_H = 0.015  # z half-height

# Default (uniform) diaginertia from XML: Ixx, Iyy, Izz
# For a solid box (a x b x h, full dims):
#   Ixx = m/12 * (b^2 + h^2), Iyy = m/12 * (a^2 + h^2), Izz = m/12 * (a^2 + b^2)
DEFAULT_DIAGINERTIA = np.array([0.000394, 0.000525, 0.000164])

# Uniform c
C_UNIFORM = np.sqrt((SLIDER_A**2 + SLIDER_B**2) / 12.0)

OUT_DIR = paths.results_dir() / "exp_pressure_distribution"


# ---------------------------------------------------------------------------
# Pressure distribution cases
# ---------------------------------------------------------------------------
@dataclass
class PressureCase:
    name: str
    label: str
    com_shift: tuple[float, float, float]  # dx, dy, dz shift from geometric center


CASES = [
    PressureCase("U", "Uniform (default)", (0.0, 0.0, 0.0)),
    PressureCase("E1", "Edge-heavy +x", (0.015, 0.0, 0.0)),
    PressureCase("E2", "Edge-heavy -x", (-0.015, 0.0, 0.0)),
    PressureCase("E3", "Edge-heavy +y", (0.0, 0.01, 0.0)),
    PressureCase("C", "Corner-heavy +x+y", (0.015, 0.01, 0.0)),
]


# ---------------------------------------------------------------------------
# Corrected limit-surface c for shifted CoM
# ---------------------------------------------------------------------------
def compute_corrected_c(dx: float, dy: float) -> float:
    """Compute the effective limit-surface c for a pressure distribution
    whose centroid is shifted by (dx, dy) from the geometric center.

    Under the parallel-axis theorem for the pressure integral:
      c_eff^2 = c_uniform^2 + dx^2 + dy^2
    This is exact when the pressure is uniform but evaluated about a shifted
    pivot, and a reasonable first-order approximation for concentrated mass.
    """
    return np.sqrt(C_UNIFORM**2 + dx**2 + dy**2)


# ---------------------------------------------------------------------------
# XML modification
# ---------------------------------------------------------------------------
def read_base_xml() -> str:
    """Read scene XML with the include inlined so from_xml_string works."""
    import re as _re

    with open(paths.scene_path("stage1_scene.xml")) as f:
        scene = f.read()
    with open(paths.scene_path("legacy/ur5e_with_pusher.xml")) as f:
        robot = f.read()

    # Inline: replace <include file="ur5e_with_pusher.xml"/> with robot content
    scene = scene.replace('<include file="ur5e_with_pusher.xml"/>', "")

    # Extract robot body content (everything inside <worldbody>...</worldbody>)
    robot_wb = _re.search(r"<worldbody>(.*?)</worldbody>", robot, _re.DOTALL)
    robot_assets = _re.search(r"<asset>(.*?)</asset>", robot, _re.DOTALL)
    robot_defaults = _re.search(r"<default>(.*?)</default>", robot, _re.DOTALL)
    robot_actuator = _re.search(r"<actuator>(.*?)</actuator>", robot, _re.DOTALL)
    robot_keyframe = _re.search(r"<keyframe>(.*?)</keyframe>", robot, _re.DOTALL)

    # Set absolute meshdir
    scene = scene.replace(
        '<compiler angle="radian" autolimits="true"/>',
        f'<compiler angle="radian" autolimits="true" meshdir="{paths.ASSETS_DIR}"/>',
    )

    # Inject robot parts into scene
    if robot_defaults:
        scene = scene.replace(
            "<default>",
            f"<default>{robot_defaults.group(1)}\n",
            1,
        )
    if robot_assets:
        scene = scene.replace(
            "<asset>",
            f"<asset>{robot_assets.group(1)}\n",
            1,
        )
    if robot_wb:
        scene = scene.replace(
            "<worldbody>",
            f"<worldbody>{robot_wb.group(1)}\n",
            1,
        )
    if robot_actuator:
        scene = scene.replace(
            "</mujoco>",
            f"<actuator>{robot_actuator.group(1)}</actuator>\n</mujoco>",
        )

    return scene


def modify_xml_for_case(xml: str, case: PressureCase) -> str:
    """Replace the slider <inertial> element to shift CoM and adjust inertia."""
    dx, dy, dz = case.com_shift
    m = SLIDER_MASS

    # Parallel axis theorem: I_new = I_cm + m * d_perp^2 for each axis
    # Ixx rotates about x => perpendicular distances are dy, dz
    # Iyy rotates about y => perpendicular distances are dx, dz
    # Izz rotates about z => perpendicular distances are dx, dy
    ixx = DEFAULT_DIAGINERTIA[0] + m * (dy**2 + dz**2)
    iyy = DEFAULT_DIAGINERTIA[1] + m * (dx**2 + dz**2)
    izz = DEFAULT_DIAGINERTIA[2] + m * (dx**2 + dy**2)

    new_inertial = (
        f'<inertial mass="{m}" pos="{dx} {dy} {dz}"\n'
        f'        diaginertia="{ixx:.6f} {iyy:.6f} {izz:.6f}"/>'
    )

    # Replace the existing <inertial ... /> line(s) in the slider body
    pattern = r'<inertial\s+mass="1\.05"[^/]*/>'
    result = re.sub(pattern, new_inertial, xml)
    return result


# ---------------------------------------------------------------------------
# Run a single MPC push simulation (headless, no rendering)
# ---------------------------------------------------------------------------
def run_push(
    xml_string: str,
    c_value: float,
    cfg: SimConfig | None = None,
) -> dict:
    """Run a full MPC push and return trajectory data.

    Args:
        xml_string: modified MuJoCo XML.
        c_value: limit-surface c to use in the MPC controller.
        cfg: simulation configuration.

    Returns:
        dict with keys: time, slider_x, slider_y, slider_theta, success.
    """
    if cfg is None:
        cfg = SimConfig()

    m = mujoco.MjModel.from_xml_string(xml_string)
    d = mujoco.MjData(m)

    tip_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pusher_tip_site")
    slider_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "slider")
    key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "ready")
    pusher_geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "pusher_tip")
    slider_geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "slider_geom")

    mujoco.mj_resetDataKeyframe(m, d, key_id)
    mujoco.mj_forward(m, d)

    ctrl = d.ctrl[:6].copy()
    substeps = int(cfg.mpc.dt / m.opt.timestep)

    # --- Phase 0: approach ---
    slider_pos_init, _ = slider_pose_from_data(d, slider_body_id)
    tip_init = d.site_xpos[tip_site_id].copy()
    slider_face_y = slider_pos_init[1] - 0.03
    approach_target = np.array([slider_pos_init[0], slider_face_y - 0.0005, tip_init[2]])

    for _ in range(200):
        mujoco.mj_forward(m, d)
        tip = d.site_xpos[tip_site_id].copy()
        err = approach_target - tip
        if np.linalg.norm(err) < 0.001:
            break
        step = err * cfg.robot.ik_gain
        step_norm = np.linalg.norm(step)
        if step_norm > cfg.robot.ik_max_step:
            step *= cfg.robot.ik_max_step / step_norm
        J = get_jacobian(m, d, tip_site_id)
        dq = damped_pinv(J, cfg.robot.damping) @ step
        ctrl = ctrl + dq
        d.ctrl[:6] = ctrl
        for __ in range(substeps):
            mujoco.mj_step(m, d)

    mujoco.mj_forward(m, d)

    # --- Phase 1: MPC push ---
    mpc = PusherSliderMPC(
        slider_dims=(SLIDER_A, SLIDER_B),
        mass=SLIDER_MASS,
        mu_pusher=0.3,
        mu_ground=0.35,
        dt=cfg.mpc.dt,
        horizon_N=cfg.mpc.horizon,
        Q_weights=np.array([30.0, 10.0, 15.0, 0.1]),
        R_weights=np.array([0.1, 0.1]),
        Q_terminal_scale=10.0,
        v_max=cfg.mpc.v_max,
        contact_face="-y",
    )
    # Override c with the desired value
    mpc.c = c_value

    tip_z_ref = d.site_xpos[tip_site_id][2]
    t_start = d.time

    times = []
    slider_xs = []
    slider_ys = []
    slider_thetas = []
    step_count = 0
    success = False

    while True:
        mujoco.mj_forward(m, d)
        tip_pos = d.site_xpos[tip_site_id].copy()
        slider_pos, slider_theta = slider_pose_from_data(d, slider_body_id)
        pusher_body = pusher_in_slider_body(tip_pos, slider_pos, slider_theta)

        times.append(d.time - t_start)
        slider_xs.append(slider_pos[0])
        slider_ys.append(slider_pos[1])
        slider_thetas.append(slider_theta)

        if slider_pos[1] >= cfg.push.y_goal:
            success = True
            break
        if d.time - t_start > cfg.push.max_sim_time:
            break

        pusher_body_clamped = np.array([np.clip(pusher_body[0], -0.038, 0.038), -0.03])
        target_y_now = min(
            slider_pos[1] + cfg.push.push_speed * cfg.mpc.dt * cfg.mpc.horizon,
            cfg.push.y_goal,
        )
        current_target = np.array([0.0, target_y_now, 0.0])

        try:
            vn, vt = mpc.compute_control(
                slider_pose=np.array([slider_pos[0], slider_pos[1], slider_theta]),
                pusher_pos_body=pusher_body_clamped,
                target_pose=current_target,
            )
        except Exception:
            vn, vt = cfg.push.push_speed, 0.0

        vn = max(vn, 0.005)
        v_world_xy = mpc.contact_to_world(vn, vt, slider_theta)
        z_error = tip_z_ref - tip_pos[2]
        v_des_3d = np.array([v_world_xy[0], v_world_xy[1], 5.0 * z_error])

        J = get_jacobian(m, d, tip_site_id)
        dq = damped_pinv(J, cfg.robot.damping) @ (v_des_3d * cfg.mpc.dt)
        ctrl = ctrl + dq
        d.ctrl[:6] = ctrl

        for _ in range(substeps):
            mujoco.mj_step(m, d)

        step_count += 1

    return {
        "time": np.array(times),
        "slider_x": np.array(slider_xs),
        "slider_y": np.array(slider_ys),
        "slider_theta": np.array(slider_thetas),
        "success": success,
        "steps": step_count,
    }


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def compute_metrics(data: dict) -> dict:
    theta = data["slider_theta"]
    x = data["slider_x"]
    final_dtheta = abs(np.degrees(theta[-1]))
    x_drift = abs(x[-1] - x[0])

    # Theta oscillation: standard deviation of theta derivative (smoothness)
    if len(theta) > 2:
        dt = np.diff(data["time"])
        dt = np.where(dt > 0, dt, 1e-6)
        dtheta_dt = np.diff(theta) / dt
        oscillation = float(np.std(dtheta_dt))
    else:
        oscillation = 0.0

    return {
        "final_dtheta_deg": final_dtheta,
        "x_drift_m": x_drift,
        "success": data["success"],
        "theta_oscillation": oscillation,
    }


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_trajectories(all_results: dict, out_dir: Path) -> None:
    """5 subplots (one per case), each showing aware vs unaware trajectory."""
    fig, axes = plt.subplots(1, 5, figsize=(22, 4.5), sharey=True)

    for idx, case in enumerate(CASES):
        ax = axes[idx]
        key_aware = (case.name, "aware")
        key_unaware = (case.name, "unaware")

        if key_aware in all_results:
            d = all_results[key_aware]
            ax.plot(d["slider_x"], d["slider_y"], "b-", linewidth=1.2, label="Aware")
        if key_unaware in all_results:
            d = all_results[key_unaware]
            ax.plot(d["slider_x"], d["slider_y"], "r--", linewidth=1.2, label="Unaware")

        ax.axvline(0, color="gray", linestyle=":", linewidth=0.5)
        ax.set_xlabel("x (m)")
        if idx == 0:
            ax.set_ylabel("y (m)")
        ax.set_title(f"{case.name}: {case.label}", fontsize=9)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal")

    fig.suptitle(
        "Pusher-slider trajectory: aware vs unaware MPC (per pressure case)",
        fontsize=11,
    )
    plt.tight_layout()
    fig.savefig(out_dir / "trajectories.png", dpi=150)
    plt.close(fig)
    print(f"Saved: {out_dir / 'trajectories.png'}")


def plot_bar_chart(all_metrics: dict, out_dir: Path) -> None:
    """Bar chart of |Delta-theta| for all cases, grouped by aware/unaware."""
    case_names = [c.name for c in CASES]
    aware_vals = []
    unaware_vals = []
    for cn in case_names:
        a = all_metrics.get((cn, "aware"), {}).get("final_dtheta_deg", 0)
        u = all_metrics.get((cn, "unaware"), {}).get("final_dtheta_deg", 0)
        aware_vals.append(a)
        unaware_vals.append(u)

    x = np.arange(len(case_names))
    width = 0.35

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.bar(x - width / 2, aware_vals, width, label="Aware", color="steelblue")
    ax.bar(x + width / 2, unaware_vals, width, label="Unaware", color="indianred")
    ax.set_xticks(x)
    ax.set_xticklabels(case_names)
    ax.set_ylabel("|Delta-theta| (deg)")
    ax.set_title("Final rotation magnitude: aware vs unaware MPC")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    plt.tight_layout()
    fig.savefig(out_dir / "dtheta_bar.png", dpi=150)
    plt.close(fig)
    print(f"Saved: {out_dir / 'dtheta_bar.png'}")


def plot_theta_timeseries(all_results: dict, out_dir: Path) -> None:
    """5 subplots showing theta(t) for aware vs unaware."""
    fig, axes = plt.subplots(1, 5, figsize=(22, 4.5), sharey=True)

    for idx, case in enumerate(CASES):
        ax = axes[idx]
        for mode, style, color in [
            ("aware", "-", "steelblue"),
            ("unaware", "--", "indianred"),
        ]:
            key = (case.name, mode)
            if key in all_results:
                d = all_results[key]
                ax.plot(
                    d["time"],
                    np.degrees(d["slider_theta"]),
                    linestyle=style,
                    color=color,
                    linewidth=1.2,
                    label=mode.capitalize(),
                )

        ax.axhline(0, color="gray", linestyle=":", linewidth=0.5)
        ax.set_xlabel("Time (s)")
        if idx == 0:
            ax.set_ylabel("Theta (deg)")
        ax.set_title(f"{case.name}: {case.label}", fontsize=9)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

    fig.suptitle("Slider orientation over time: aware vs unaware MPC", fontsize=11)
    plt.tight_layout()
    fig.savefig(out_dir / "theta_timeseries.png", dpi=150)
    plt.close(fig)
    print(f"Saved: {out_dir / 'theta_timeseries.png'}")


# ---------------------------------------------------------------------------
# Summary table
# ---------------------------------------------------------------------------
def print_summary_table(all_metrics: dict) -> None:
    header = (
        f"{'Case':<6} {'Mode':<10} {'|Dtheta| (deg)':>15} "
        f"{'x-drift (mm)':>13} {'Success':>8} {'Theta osc':>10}"
    )
    print("\n" + "=" * len(header))
    print("SUMMARY TABLE")
    print("=" * len(header))
    print(header)
    print("-" * len(header))

    for case in CASES:
        for mode in ["aware", "unaware"]:
            key = (case.name, mode)
            m = all_metrics.get(key, {})
            dth = m.get("final_dtheta_deg", float("nan"))
            xd = m.get("x_drift_m", float("nan")) * 1000
            suc = m.get("success", False)
            osc = m.get("theta_oscillation", float("nan"))
            print(
                f"{case.name:<6} {mode:<10} {dth:>15.3f} "
                f"{xd:>13.2f} {'Yes' if suc else 'No':>8} {osc:>10.4f}"
            )

    print("=" * len(header))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    cfg = SimConfig()
    # Speed up: shorter push, faster speed, smaller horizon, larger MPC dt
    cfg.push.push_speed = 0.06
    cfg.push.y_start = 0.235
    cfg.push.y_goal = 0.50
    cfg.push.max_sim_time = 15.0
    cfg.mpc.dt = 0.05
    cfg.mpc.horizon = 5
    cfg.mpc.v_max = 0.12

    base_xml = read_base_xml()

    all_results: dict[tuple[str, str], dict] = {}
    all_metrics: dict[tuple[str, str], dict] = {}

    total_runs = len(CASES) * 2
    run_idx = 0

    for case in CASES:
        xml = modify_xml_for_case(base_xml, case)
        dx, dy, _ = case.com_shift

        c_corrected = compute_corrected_c(dx, dy)

        for mode in ["aware", "unaware"]:
            run_idx += 1
            c_val = c_corrected if mode == "aware" else C_UNIFORM
            key = (case.name, mode)

            print(
                f"\n[{run_idx}/{total_runs}] Case={case.name} ({case.label}), "
                f"mode={mode}, c={c_val:.5f}"
            )
            t0 = time.monotonic()
            data = run_push(xml, c_val, cfg)
            elapsed = time.monotonic() - t0

            metrics = compute_metrics(data)
            all_results[key] = data
            all_metrics[key] = metrics

            print(
                f"  -> {elapsed:.1f}s, steps={data['steps']}, "
                f"success={data['success']}, "
                f"|Dtheta|={metrics['final_dtheta_deg']:.3f} deg, "
                f"x-drift={metrics['x_drift_m'] * 1000:.2f} mm"
            )

    # --- Summary ---
    print_summary_table(all_metrics)

    # --- Plots ---
    print("\nGenerating plots...")
    plot_trajectories(all_results, OUT_DIR)
    plot_bar_chart(all_metrics, OUT_DIR)
    plot_theta_timeseries(all_results, OUT_DIR)

    # --- Save raw data ---
    npz_data = {}
    for key, data in all_results.items():
        prefix = f"{key[0]}_{key[1]}"
        for field in ["time", "slider_x", "slider_y", "slider_theta"]:
            npz_data[f"{prefix}_{field}"] = data[field]
        npz_data[f"{prefix}_success"] = np.array([data["success"]])
    np.savez_compressed(OUT_DIR / "data.npz", **npz_data)
    print(f"Data saved: {OUT_DIR / 'data.npz'}")

    # --- Save metrics ---
    metrics_lines = []
    for key, m in all_metrics.items():
        metrics_lines.append(
            f"{key[0]},{key[1]},{m['final_dtheta_deg']:.4f},"
            f"{m['x_drift_m']:.6f},{m['success']},"
            f"{m['theta_oscillation']:.6f}"
        )
    metrics_path = OUT_DIR / "metrics.csv"
    with open(metrics_path, "w") as f:
        f.write("case,mode,final_dtheta_deg,x_drift_m,success,theta_oscillation\n")
        for line in metrics_lines:
            f.write(line + "\n")
    print(f"Metrics saved: {metrics_path}")

    print("\nDone.")


if __name__ == "__main__":
    main()
