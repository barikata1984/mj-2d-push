"""
Contact parameter sensitivity analysis for the MuJoCo pusher-slider system.

Varies one parameter at a time (mu_pusher, mu_ground, solref[0], solimp[0], cone type)
and records slider drift, orientation oscillation, and push success metrics.

Usage:
    pixi run python experiments/exp_contact_sensitivity.py
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass, field
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
# Defaults (Hogan 2016 values)
# ---------------------------------------------------------------------------
DEFAULTS = {
    "mu_pusher": 0.3,
    "mu_ground": 0.35,
    "solref0": 0.02,
    "solimp0": 0.9,
    "cone": "elliptic",
}

# Parameter sweep grid
SWEEPS: dict[str, list] = {
    "mu_pusher": [0.1, 0.2, 0.3, 0.4, 0.5],
    "mu_ground": [0.15, 0.25, 0.35, 0.45, 0.55],
    "solref0": [0.005, 0.01, 0.02, 0.04, 0.08],
    "solimp0": [0.8, 0.85, 0.9, 0.95, 0.99],
    "cone": ["pyramidal", "elliptic"],
}

OUTPUT_DIR = paths.results_dir() / "exp_contact_sensitivity"


# ---------------------------------------------------------------------------
# Result container
# ---------------------------------------------------------------------------
@dataclass
class RunResult:
    param_name: str
    param_value: object
    final_x: float = 0.0
    final_y: float = 0.0
    final_theta: float = 0.0
    x_drift: float = 0.0
    theta_osc_amplitude: float = 0.0
    theta_rms: float = 0.0
    sim_time: float = 0.0
    success: bool = False
    theta_history: list[float] = field(default_factory=list)


# ---------------------------------------------------------------------------
# XML modification helpers
# ---------------------------------------------------------------------------
def load_base_xml() -> str:
    """Load the two XML files and inline the include."""
    scene_path = Path(paths.scene_path("stage1_scene.xml"))
    ur5e_path = Path(paths.scene_path("legacy/ur5e_with_pusher.xml"))
    scene_xml = scene_path.read_text()
    ur5e_xml = ur5e_path.read_text()

    # Strip the outer <mujoco> wrapper from the included file so we can inline it.
    # Extract content between the first <mujoco ...> and the last </mujoco>.
    inner = re.sub(r"^\s*<mujoco[^>]*>\s*", "", ur5e_xml, count=1)
    inner = re.sub(r"\s*</mujoco>\s*$", "", inner)

    # Also strip duplicate <compiler> and <option> from the included file
    # since the scene already defines them.
    inner = re.sub(r"<compiler[^/]*/>\s*", "", inner)
    inner = re.sub(r"<option[^/]*/>\s*", "", inner)

    # Replace <include file="ur5e_with_pusher.xml"/> with the inline content
    scene_xml = scene_xml.replace('<include file="ur5e_with_pusher.xml"/>', inner)

    # Add meshdir to the scene compiler so from_xml_string can find mesh assets.
    scene_xml = scene_xml.replace(
        '<compiler angle="radian" autolimits="true"/>',
        f'<compiler angle="radian" autolimits="true" meshdir="{paths.ASSETS_DIR}"/>',
    )

    return scene_xml


def modify_xml(
    base_xml: str,
    mu_pusher: float = DEFAULTS["mu_pusher"],
    mu_ground: float = DEFAULTS["mu_ground"],
    solref0: float = DEFAULTS["solref0"],
    solimp0: float = DEFAULTS["solimp0"],
    cone: str = DEFAULTS["cone"],
) -> str:
    """Return modified XML string with the specified contact parameters."""
    xml = base_xml

    # --- Cone type ---
    xml = re.sub(r'cone="[^"]*"', f'cone="{cone}"', xml)

    # --- Pusher-slider pair (pusher_tip <-> slider_geom) ---
    # Match the pair element for pusher_tip and slider_geom
    def replace_pusher_pair(m: re.Match) -> str:
        s = m.group(0)
        # Replace friction: first two values are tangential (mu_pusher)
        s = re.sub(
            r'friction="[^"]*"',
            f'friction="{mu_pusher} {mu_pusher} 0.005 0.001 0.001"',
            s,
        )
        s = re.sub(r'solref="[^"]*"', f'solref="{solref0} 1"', s)
        s = re.sub(
            r'solimp="[^"]*"',
            f'solimp="{solimp0} 0.95 0.001 0.5 2"',
            s,
        )
        return s

    xml = re.sub(
        r'<pair\s+geom1="pusher_tip"\s+geom2="slider_geom"[^/]*/>',
        replace_pusher_pair,
        xml,
    )

    # --- Slider-table pair (slider_geom <-> table_surface) ---
    def replace_table_pair(m: re.Match) -> str:
        s = m.group(0)
        s = re.sub(
            r'friction="[^"]*"',
            f'friction="{mu_ground} {mu_ground} 0.005 0.001 0.001"',
            s,
        )
        s = re.sub(r'solref="[^"]*"', f'solref="{solref0} 1"', s)
        s = re.sub(
            r'solimp="[^"]*"',
            f'solimp="{solimp0} 0.95 0.001 0.5 2"',
            s,
        )
        return s

    xml = re.sub(
        r'<pair\s+geom1="slider_geom"\s+geom2="table_surface"[^/]*/>',
        replace_table_pair,
        xml,
    )

    # --- Also update the default class solref/solimp for slider_contact ---
    # This affects the slider_geom default contact properties
    def replace_slider_default(m: re.Match) -> str:
        s = m.group(0)
        s = re.sub(r'solref="[^"]*"', f'solref="{solref0} 1"', s)
        s = re.sub(
            r'solimp="[^"]*"',
            f'solimp="{solimp0} 0.95 0.001 0.5 2"',
            s,
        )
        return s

    xml = re.sub(
        r'<default\s+class="slider_contact">.*?</default>',
        replace_slider_default,
        xml,
        flags=re.DOTALL,
    )

    # --- Also update pusher_tip geom friction/solref/solimp in the robot definition ---
    def replace_pusher_tip_geom(m: re.Match) -> str:
        s = m.group(0)
        s = re.sub(
            r'friction="[^"]*"',
            f'friction="{mu_pusher} 0.005 0.001"',
            s,
        )
        s = re.sub(r'solref="[^"]*"', f'solref="{solref0} 1"', s)
        s = re.sub(
            r'solimp="[^"]*"',
            f'solimp="{solimp0} 0.95 0.001 0.5 2"',
            s,
        )
        return s

    xml = re.sub(r'<geom\s+name="pusher_tip"[^/]*/>', replace_pusher_tip_geom, xml)

    return xml


# ---------------------------------------------------------------------------
# Single simulation run (headless, no rendering)
# ---------------------------------------------------------------------------
def run_single(
    xml_string: str,
    mu_pusher: float,
    mu_ground: float,
    cfg: SimConfig | None = None,
) -> RunResult:
    """Run one push simulation and return metrics."""
    if cfg is None:
        cfg = SimConfig()

    m = mujoco.MjModel.from_xml_string(xml_string)
    d = mujoco.MjData(m)

    tip_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pusher_tip_site")
    slider_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "slider")
    key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "ready")

    mujoco.mj_resetDataKeyframe(m, d, key_id)
    mujoco.mj_forward(m, d)

    tip_init = d.site_xpos[tip_site_id].copy()
    slider_pos_init, _ = slider_pose_from_data(d, slider_body_id)

    ctrl = d.ctrl[:6].copy()
    substeps = int(cfg.mpc.dt / m.opt.timestep)

    # --- Phase 0: Approach ---
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

    # --- Phase 1: MPC push ---
    mpc = PusherSliderMPC(
        slider_dims=(0.08, 0.06),
        mass=1.05,
        mu_pusher=mu_pusher,
        mu_ground=mu_ground,
        dt=cfg.mpc.dt,
        horizon_N=cfg.mpc.horizon,
        Q_weights=np.array([30.0, 10.0, 15.0, 0.1]),
        R_weights=np.array([0.1, 0.1]),
        Q_terminal_scale=10.0,
        v_max=cfg.mpc.v_max,
        contact_face="-y",
    )

    t_start = d.time
    tip_z_ref = d.site_xpos[tip_site_id][2]
    theta_history: list[float] = []

    while True:
        mujoco.mj_forward(m, d)

        tip_pos = d.site_xpos[tip_site_id].copy()
        slider_pos, slider_theta = slider_pose_from_data(d, slider_body_id)
        pusher_body = pusher_in_slider_body(tip_pos, slider_pos, slider_theta)

        theta_history.append(slider_theta)

        if slider_pos[1] >= cfg.push.y_goal:
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

    # --- Compute metrics ---
    mujoco.mj_forward(m, d)
    slider_final, theta_final = slider_pose_from_data(d, slider_body_id)

    theta_arr = np.array(theta_history)
    n_half = len(theta_arr) // 2
    theta_tail = theta_arr[n_half:] if n_half > 0 else theta_arr

    result = RunResult(
        param_name="",
        param_value=None,
        final_x=slider_final[0],
        final_y=slider_final[1],
        final_theta=theta_final,
        x_drift=abs(slider_final[0]),
        theta_osc_amplitude=float(np.max(theta_tail) - np.min(theta_tail))
        if len(theta_tail) > 0
        else 0.0,
        theta_rms=float(np.sqrt(np.mean(theta_arr**2))) if len(theta_arr) > 0 else 0.0,
        sim_time=d.time - t_start,
        success=slider_final[1] >= 0.75,
        theta_history=theta_history,
    )
    return result


# ---------------------------------------------------------------------------
# Main sweep
# ---------------------------------------------------------------------------
def make_fast_config() -> SimConfig:
    """Config tuned for faster sweep runs (larger dt, shorter horizon)."""
    cfg = SimConfig()
    cfg.mpc.dt = 0.05
    cfg.mpc.horizon = 5
    cfg.push.max_sim_time = 40.0
    return cfg


def run_sweep() -> dict[str, list[RunResult]]:
    """Run all parameter sweeps and return results grouped by parameter name."""
    cfg = make_fast_config()
    base_xml = load_base_xml()
    all_results: dict[str, list[RunResult]] = {}

    for param_name, values in SWEEPS.items():
        print(f"\n{'=' * 60}")
        print(f"Sweeping: {param_name}")
        print(f"{'=' * 60}")
        results_for_param: list[RunResult] = []

        for val in values:
            # Build kwargs, overriding only the swept parameter
            kwargs = {
                "mu_pusher": DEFAULTS["mu_pusher"],
                "mu_ground": DEFAULTS["mu_ground"],
                "solref0": DEFAULTS["solref0"],
                "solimp0": DEFAULTS["solimp0"],
                "cone": DEFAULTS["cone"],
            }
            kwargs[param_name] = val

            xml = modify_xml(base_xml, **kwargs)

            label = f"{param_name}={val}"
            print(f"  Running {label} ...", end=" ", flush=True)
            t0 = time.monotonic()

            try:
                res = run_single(
                    xml,
                    mu_pusher=kwargs["mu_pusher"],
                    mu_ground=kwargs["mu_ground"],
                    cfg=cfg,
                )
                res.param_name = param_name
                res.param_value = val
            except Exception as e:
                print(f"FAILED: {e}")
                res = RunResult(param_name=param_name, param_value=val)
                res.success = False

            wall = time.monotonic() - t0
            status = "OK" if res.success else "FAIL"
            print(
                f"{status}  wall={wall:.1f}s  y={res.final_y:.4f}  "
                f"x_drift={res.x_drift:.4f}  "
                f"theta_osc={np.degrees(res.theta_osc_amplitude):.3f}deg"
            )
            results_for_param.append(res)

        all_results[param_name] = results_for_param

    return all_results


# ---------------------------------------------------------------------------
# Summary table
# ---------------------------------------------------------------------------
def print_summary(all_results: dict[str, list[RunResult]]) -> None:
    print(f"\n{'=' * 90}")
    print("CONTACT PARAMETER SENSITIVITY ANALYSIS — SUMMARY")
    print(f"{'=' * 90}")
    header = (
        f"{'Parameter':<12} {'Value':>8} {'Success':>8} {'final_y':>8} "
        f"{'x_drift':>9} {'theta_osc':>10} {'theta_rms':>10} {'sim_t':>7}"
    )
    print(header)
    print("-" * len(header))

    for param_name, results in all_results.items():
        for r in results:
            val_str = (
                f"{r.param_value}" if isinstance(r.param_value, str) else f"{r.param_value:.3f}"
            )
            print(
                f"{r.param_name:<12} {val_str:>8} {'YES' if r.success else 'NO':>8} "
                f"{r.final_y:>8.4f} {r.x_drift:>9.5f} "
                f"{np.degrees(r.theta_osc_amplitude):>10.4f}deg "
                f"{np.degrees(r.theta_rms):>9.4f}deg "
                f"{r.sim_time:>7.2f}s"
            )
        print("-" * len(header))


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_results(all_results: dict[str, list[RunResult]]) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Separate numeric vs categorical parameters
    numeric_params = ["mu_pusher", "mu_ground", "solref0", "solimp0"]
    cat_params = ["cone"]

    n_numeric = len(numeric_params)

    fig, axes = plt.subplots(3, n_numeric, figsize=(5 * n_numeric, 12))

    for col_idx, param_name in enumerate(numeric_params):
        results = all_results.get(param_name, [])
        if not results:
            continue

        values = [r.param_value for r in results]
        x_drifts = [r.x_drift * 1000 for r in results]  # mm
        theta_oscs = [np.degrees(r.theta_osc_amplitude) for r in results]
        successes = [1 if r.success else 0 for r in results]

        default_val = DEFAULTS[param_name]

        # Panel 1: x-drift
        ax = axes[0, col_idx]
        ax.plot(values, x_drifts, "o-", color="tab:blue", linewidth=1.5, markersize=6)
        ax.axvline(
            default_val,
            color="red",
            linestyle="--",
            linewidth=0.8,
            alpha=0.7,
            label="default",
        )
        ax.set_xlabel(param_name)
        ax.set_ylabel("x-drift (mm)")
        ax.set_title(f"x-drift vs {param_name}")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

        # Panel 2: theta oscillation
        ax = axes[1, col_idx]
        ax.plot(values, theta_oscs, "s-", color="tab:orange", linewidth=1.5, markersize=6)
        ax.axvline(
            default_val,
            color="red",
            linestyle="--",
            linewidth=0.8,
            alpha=0.7,
            label="default",
        )
        ax.set_xlabel(param_name)
        ax.set_ylabel("theta osc (deg)")
        ax.set_title(f"theta oscillation vs {param_name}")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

        # Panel 3: success
        ax = axes[2, col_idx]
        colors = ["tab:green" if s else "tab:red" for s in successes]
        ax.bar([str(v) for v in values], successes, color=colors, width=0.6)
        ax.set_xlabel(param_name)
        ax.set_ylabel("Success")
        ax.set_title(f"Push success vs {param_name}")
        ax.set_ylim(-0.1, 1.3)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(["Fail", "Success"])
        ax.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    fig.savefig(OUTPUT_DIR / "sensitivity_numeric.png", dpi=150)
    print(f"\nNumeric parameter plot saved: {OUTPUT_DIR / 'sensitivity_numeric.png'}")
    plt.close(fig)

    # --- Cone type (categorical) bar chart ---
    if "cone" in all_results:
        results = all_results["cone"]
        labels = [str(r.param_value) for r in results]
        x_drifts = [r.x_drift * 1000 for r in results]
        theta_oscs = [np.degrees(r.theta_osc_amplitude) for r in results]
        successes = [1 if r.success else 0 for r in results]

        fig2, axes2 = plt.subplots(1, 3, figsize=(12, 4))

        ax = axes2[0]
        ax.bar(labels, x_drifts, color=["tab:blue", "tab:cyan"], width=0.5)
        ax.set_ylabel("x-drift (mm)")
        ax.set_title("x-drift vs cone type")
        ax.grid(True, alpha=0.3, axis="y")

        ax = axes2[1]
        ax.bar(labels, theta_oscs, color=["tab:orange", "tab:red"], width=0.5)
        ax.set_ylabel("theta osc (deg)")
        ax.set_title("theta oscillation vs cone type")
        ax.grid(True, alpha=0.3, axis="y")

        ax = axes2[2]
        colors = ["tab:green" if s else "tab:red" for s in successes]
        ax.bar(labels, successes, color=colors, width=0.5)
        ax.set_ylabel("Success")
        ax.set_title("Push success vs cone type")
        ax.set_ylim(-0.1, 1.3)
        ax.set_yticks([0, 1])
        ax.set_yticklabels(["Fail", "Success"])
        ax.grid(True, alpha=0.3, axis="y")

        plt.tight_layout()
        fig2.savefig(OUTPUT_DIR / "sensitivity_cone.png", dpi=150)
        print(f"Cone type plot saved: {OUTPUT_DIR / 'sensitivity_cone.png'}")
        plt.close(fig2)

    # --- Combined multi-panel overview ---
    fig3, axes3 = plt.subplots(3, 1, figsize=(14, 10))

    # Collect all numeric results into a unified view
    all_labels: list[str] = []
    all_x_drifts: list[float] = []
    all_theta_oscs: list[float] = []
    all_successes: list[bool] = []
    group_boundaries: list[int] = []

    for param_name in list(numeric_params) + cat_params:
        results = all_results.get(param_name, [])
        group_boundaries.append(len(all_labels))
        for r in results:
            val_str = (
                f"{r.param_value}" if isinstance(r.param_value, str) else f"{r.param_value:.3f}"
            )
            all_labels.append(f"{param_name}\n{val_str}")
            all_x_drifts.append(r.x_drift * 1000)
            all_theta_oscs.append(np.degrees(r.theta_osc_amplitude))
            all_successes.append(r.success)

    x_pos = np.arange(len(all_labels))

    # Identify default values
    is_default = []
    for param_name in list(numeric_params) + cat_params:
        results = all_results.get(param_name, [])
        for r in results:
            if isinstance(r.param_value, str):
                is_default.append(r.param_value == DEFAULTS[param_name])
            else:
                is_default.append(abs(r.param_value - DEFAULTS[param_name]) < 1e-6)

    # Panel 1: x-drift
    ax = axes3[0]
    bar_colors = ["tab:red" if d else "tab:blue" for d in is_default]
    ax.bar(x_pos, all_x_drifts, color=bar_colors, width=0.7)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(all_labels, fontsize=7, rotation=0)
    ax.set_ylabel("x-drift (mm)")
    ax.set_title("x-drift across all parameter settings (red = default)")
    ax.grid(True, alpha=0.3, axis="y")
    for b in group_boundaries[1:]:
        ax.axvline(b - 0.5, color="gray", linestyle=":", linewidth=0.8)

    # Panel 2: theta oscillation
    ax = axes3[1]
    bar_colors = ["tab:red" if d else "tab:orange" for d in is_default]
    ax.bar(x_pos, all_theta_oscs, color=bar_colors, width=0.7)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(all_labels, fontsize=7, rotation=0)
    ax.set_ylabel("theta osc (deg)")
    ax.set_title("theta oscillation across all parameter settings (red = default)")
    ax.grid(True, alpha=0.3, axis="y")
    for b in group_boundaries[1:]:
        ax.axvline(b - 0.5, color="gray", linestyle=":", linewidth=0.8)

    # Panel 3: success/failure
    ax = axes3[2]
    bar_colors = ["tab:green" if s else "tab:red" for s in all_successes]
    ax.bar(x_pos, [1 if s else 0 for s in all_successes], color=bar_colors, width=0.7)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(all_labels, fontsize=7, rotation=0)
    ax.set_ylabel("Success")
    ax.set_title("Push success across all parameter settings")
    ax.set_ylim(-0.1, 1.3)
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["Fail", "Success"])
    ax.grid(True, alpha=0.3, axis="y")
    for b in group_boundaries[1:]:
        ax.axvline(b - 0.5, color="gray", linestyle=":", linewidth=0.8)

    plt.tight_layout()
    fig3.savefig(OUTPUT_DIR / "sensitivity_overview.png", dpi=150)
    print(f"Overview plot saved: {OUTPUT_DIR / 'sensitivity_overview.png'}")
    plt.close(fig3)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    all_results = run_sweep()
    print_summary(all_results)
    plot_results(all_results)
    print(f"\nAll outputs in: {OUTPUT_DIR}")
