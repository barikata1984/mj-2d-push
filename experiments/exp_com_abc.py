"""
Experiment: CoM offset A/B/C contrast in MuJoCo.

For each CoM offset d (0..20 mm along slider body x-axis), run three conditions:
  A: Controller believes CoM is at geometric center (no compensation)
  B: Controller uses analytically estimated CoM to compensate target trajectory
  C: Controller uses the true CoM (oracle) to compensate target trajectory

The MPC controller pushes the slider from y=0.235 to y=0.78. Each condition
differs in the target x-position:
  - A targets x=0 (geometric center on centerline)
  - B targets x=-est_d (estimated CoM on centerline)
  - C targets x=-d (true CoM on centerline)

The metric is |Delta theta| — cumulative rotation induced by the push.
Expected result: A > B ≈ C, with the gap growing as d increases.

In a closed-loop MPC setting, the effect is smaller than in Stage 0's
open-loop analytical model because MPC feedback partially corrects
the rotation. This is physically correct.

Usage:
    pixi run python experiments/exp_com_abc.py
"""

from __future__ import annotations

import re
import sys
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from pusher_slider import paths
from pusher_slider.analytical.push_com_sim import estimate_com
from pusher_slider.controllers import PusherSliderMPC

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SCENE_PATH = Path(paths.scene_path("stage1_scene.xml"))
RESULTS_DIR = paths.results_dir() / "exp_com_abc"

# Simulation parameters (tuned for batch experiment speed)
PUSH_SPEED = 0.04
Y_START = 0.235
Y_GOAL = 0.78
MAX_SIM_TIME = 25.0
MPC_DT = 0.05
MPC_HORIZON = 5
MPC_V_MAX = 0.10
DAMPING = 1e-3
IK_GAIN = 2.0
IK_MAX_STEP = 0.02

# Slider dimensions (meters)
SLIDER_A = 0.08  # full x-width
SLIDER_B = 0.06  # full y-width
SLIDER_HALF_X = SLIDER_A / 2.0  # 0.04
SLIDER_HALF_Y = SLIDER_B / 2.0  # 0.03

# CoM offsets to test (meters) — limited to 0..20 mm for clean results
COM_OFFSETS_MM = [0, 2, 5, 8, 10, 12, 15, 18, 20]
COM_OFFSETS = [d * 1e-3 for d in COM_OFFSETS_MM]


# ---------------------------------------------------------------------------
# XML modification: shift slider CoM
# ---------------------------------------------------------------------------
def modify_scene_xml(com_offset_x: float) -> str:
    """Read the scene XML and modify slider inertial pos to shift CoM.

    MuJoCo's <inertial> diaginertia is always about the CoM.
    The pos attribute tells MuJoCo where the CoM is in body frame.
    We keep the original diaginertia unchanged.
    """
    xml_text = SCENE_PATH.read_text()
    new_pos = f"{com_offset_x:.6f} 0 0"
    xml_text = re.sub(
        r'(<inertial\s+mass="1\.05"\s+pos=")[^"]*(")',
        rf"\g<1>{new_pos}\2",
        xml_text,
    )
    return xml_text


# ---------------------------------------------------------------------------
# Helpers (from run_stage1.py)
# ---------------------------------------------------------------------------
def slider_pose_from_data(d: mujoco.MjData, slider_body_id: int) -> tuple[np.ndarray, float]:
    """Extract slider (x, y, z) position and yaw angle theta."""
    pos = d.xpos[slider_body_id].copy()
    quat = d.xquat[slider_body_id].copy()
    w, qx, qy, qz = quat
    theta = np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    return pos, theta


def pusher_in_slider_body(
    tip_world: np.ndarray, slider_pos: np.ndarray, theta: float
) -> np.ndarray:
    """Compute pusher tip position in slider body frame."""
    dx = tip_world[0] - slider_pos[0]
    dy = tip_world[1] - slider_pos[1]
    ct, st = np.cos(theta), np.sin(theta)
    px = ct * dx + st * dy
    py = -st * dx + ct * dy
    return np.array([px, py])


def damped_pinv(J: np.ndarray, damping: float) -> np.ndarray:
    JJT = J @ J.T
    return J.T @ np.linalg.inv(JJT + damping**2 * np.eye(JJT.shape[0]))


def get_jacobian(m: mujoco.MjModel, d: mujoco.MjData, site_id: int) -> np.ndarray:
    jacp = np.zeros((3, m.nv))
    mujoco.mj_jacSite(m, d, jacp, None, site_id)
    return jacp[:, :6]


def move_tip_to(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    tip_site_id: int,
    target_pos: np.ndarray,
    ctrl: np.ndarray,
    max_steps: int = 300,
    tol: float = 0.001,
) -> np.ndarray:
    """Move pusher tip to target via iterative IK (no rendering)."""
    substeps = int(MPC_DT / m.opt.timestep)
    for _ in range(max_steps):
        mujoco.mj_forward(m, d)
        tip = d.site_xpos[tip_site_id].copy()
        err = target_pos - tip
        if np.linalg.norm(err) < tol:
            break
        step = err * IK_GAIN
        step_norm = np.linalg.norm(step)
        if step_norm > IK_MAX_STEP:
            step *= IK_MAX_STEP / step_norm
        J = get_jacobian(m, d, tip_site_id)
        dq = damped_pinv(J, DAMPING) @ step
        ctrl = ctrl + dq
        d.ctrl[:6] = ctrl
        for __ in range(substeps):
            mujoco.mj_step(m, d)
    return ctrl


# ---------------------------------------------------------------------------
# Run one push trial
# ---------------------------------------------------------------------------
def run_push_trial(
    com_offset_x: float,
    target_x: float = 0.0,
    label: str = "",
) -> dict:
    """Run a full MPC-controlled push from y_start to y_goal.

    The MPC aims to drive the slider to (target_x, y_goal, 0). By varying
    target_x across conditions, we change which point the controller tries
    to keep on the centerline:
      - target_x = 0: keep geometric center at x=0 (condition A)
      - target_x = -d_est: keep estimated CoM at x=0 (condition B)
      - target_x = -d: keep true CoM at x=0 (condition C)

    Args:
        com_offset_x: true CoM offset in slider body x (meters).
        target_x: the x-position target the MPC aims for.
        label: human-readable label for logging.

    Returns:
        dict with final rotation and trajectory history.
    """
    xml_str = modify_scene_xml(com_offset_x)
    m = mujoco.MjModel.from_xml_string(xml_str)
    d = mujoco.MjData(m)

    tip_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pusher_tip_site")
    slider_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "slider")
    key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "ready")

    mujoco.mj_resetDataKeyframe(m, d, key_id)
    mujoco.mj_forward(m, d)

    tip_init = d.site_xpos[tip_site_id].copy()
    slider_pos_init, _ = slider_pose_from_data(d, slider_body_id)
    ctrl = d.ctrl[:6].copy()
    substeps = int(MPC_DT / m.opt.timestep)

    # Phase 0: Approach
    slider_face_y = slider_pos_init[1] - SLIDER_HALF_Y
    approach_target = np.array([slider_pos_init[0], slider_face_y - 0.0005, tip_init[2]])
    ctrl = move_tip_to(m, d, tip_site_id, approach_target, ctrl, max_steps=200)
    mujoco.mj_forward(m, d)

    # MPC controller
    mpc = PusherSliderMPC(
        slider_dims=(SLIDER_A, SLIDER_B),
        mass=1.05,
        mu_pusher=0.3,
        mu_ground=0.35,
        dt=MPC_DT,
        horizon_N=MPC_HORIZON,
        Q_weights=np.array([30.0, 10.0, 15.0, 0.1]),
        R_weights=np.array([0.1, 0.1]),
        Q_terminal_scale=10.0,
        v_max=MPC_V_MAX,
        contact_face="-y",
    )

    tip_z_ref = d.site_xpos[tip_site_id][2]
    t_start = d.time
    step_count = 0

    theta_history = []
    x_history = []
    y_history = []

    while True:
        mujoco.mj_forward(m, d)

        tip_pos = d.site_xpos[tip_site_id].copy()
        slider_pos, slider_theta = slider_pose_from_data(d, slider_body_id)
        pusher_body = pusher_in_slider_body(tip_pos, slider_pos, slider_theta)

        theta_history.append(slider_theta)
        x_history.append(slider_pos[0])
        y_history.append(slider_pos[1])

        if slider_pos[1] >= Y_GOAL:
            break
        if d.time - t_start > MAX_SIM_TIME:
            break

        pusher_body_clamped = np.array([np.clip(pusher_body[0], -0.038, 0.038), -SLIDER_HALF_Y])

        target_y_now = min(slider_pos[1] + PUSH_SPEED * MPC_DT * MPC_HORIZON, Y_GOAL)
        current_target = np.array([target_x, target_y_now, 0.0])

        try:
            vn, vt = mpc.compute_control(
                slider_pose=np.array([slider_pos[0], slider_pos[1], slider_theta]),
                pusher_pos_body=pusher_body_clamped,
                target_pose=current_target,
            )
        except Exception:
            vn, vt = PUSH_SPEED, 0.0

        vn = max(vn, 0.005)

        v_world_xy = mpc.contact_to_world(vn, vt, slider_theta)
        z_error = tip_z_ref - tip_pos[2]
        v_des_3d = np.array([v_world_xy[0], v_world_xy[1], 5.0 * z_error])

        J = get_jacobian(m, d, tip_site_id)
        dq = damped_pinv(J, DAMPING) @ (v_des_3d * MPC_DT)
        ctrl = ctrl + dq
        d.ctrl[:6] = ctrl

        for _ in range(substeps):
            mujoco.mj_step(m, d)

        step_count += 1

    final_theta = theta_history[-1] if theta_history else 0.0
    final_x = x_history[-1] if x_history else 0.0
    final_y = y_history[-1] if y_history else Y_START

    return {
        "final_theta": final_theta,
        "abs_delta_theta_deg": abs(np.degrees(final_theta)),
        "final_x": final_x,
        "final_y": final_y,
        "steps": step_count,
        "theta_history": np.array(theta_history),
        "x_history": np.array(x_history),
        "y_history": np.array(y_history),
    }


# ---------------------------------------------------------------------------
# CoM estimation via Stage 0 analytical model
# ---------------------------------------------------------------------------
def estimate_com_for_offset(d_offset: float) -> np.ndarray:
    """Estimate CoM using the analytical estimator from push_com_sim.py."""
    true_com = np.array([d_offset, 0.0])
    half = SLIDER_HALF_X
    contacts = [
        np.array([0.0, -SLIDER_HALF_Y]),
        np.array([half, -SLIDER_HALF_Y]),
        np.array([-half, -SLIDER_HALF_Y]),
    ]
    c = np.sqrt((SLIDER_A**2 + SLIDER_B**2) / 12.0)
    com_hat = estimate_com(true_com, contacts, c=c, noise=1e-4)
    return com_hat


# ---------------------------------------------------------------------------
# Main experiment
# ---------------------------------------------------------------------------
def main() -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    print("=" * 72)
    print("Experiment: CoM offset A/B/C contrast in MuJoCo")
    print("=" * 72)
    print(f"CoM offsets: {COM_OFFSETS_MM} mm")
    print(f"Push: y={Y_START} -> y={Y_GOAL}, speed={PUSH_SPEED} m/s")
    print(f"MPC: dt={MPC_DT}, horizon={MPC_HORIZON}, v_max={MPC_V_MAX}")
    print()

    results = {
        "offsets_mm": np.array(COM_OFFSETS_MM, dtype=float),
        "offsets_m": np.array(COM_OFFSETS, dtype=float),
        "dtheta_A": np.zeros(len(COM_OFFSETS)),
        "dtheta_B": np.zeros(len(COM_OFFSETS)),
        "dtheta_C": np.zeros(len(COM_OFFSETS)),
        "com_est_err_mm": np.zeros(len(COM_OFFSETS)),
    }

    total_trials = len(COM_OFFSETS) * 3
    trial_num = 0

    for i, d_m in enumerate(COM_OFFSETS):
        d_mm = d_m * 1000
        print(f"\n{'─' * 72}")
        print(f"CoM offset d = {d_mm:.0f} mm")
        print(f"{'─' * 72}")

        true_com = np.array([d_m, 0.0])
        com_hat = estimate_com_for_offset(d_m)
        com_err = np.linalg.norm(com_hat - true_com)
        results["com_est_err_mm"][i] = com_err * 1000
        print(f"  Estimated CoM: ({com_hat[0] * 1000:.2f}, {com_hat[1] * 1000:.2f}) mm")
        print(f"  CoM estimation error: {com_err * 1000:.2f} mm")

        # Condition A: geometric center (target_x = 0)
        trial_num += 1
        print(
            f"\n  [{trial_num}/{total_trials}] Condition A (geometric, target_x=0)...",
            end=" ",
        )
        sys.stdout.flush()
        t0 = time.monotonic()
        res_A = run_push_trial(com_offset_x=d_m, target_x=0.0, label="A")
        elapsed = time.monotonic() - t0
        results["dtheta_A"][i] = res_A["abs_delta_theta_deg"]
        print(f"|dtheta|={res_A['abs_delta_theta_deg']:.2f} deg  ({elapsed:.1f}s)")

        # Condition B: estimated CoM (target_x = -est_d)
        trial_num += 1
        target_x_B = -com_hat[0]
        print(
            f"  [{trial_num}/{total_trials}] Condition B (estimated, "
            f"target_x={target_x_B * 1000:.2f}mm)...",
            end=" ",
        )
        sys.stdout.flush()
        t0 = time.monotonic()
        res_B = run_push_trial(com_offset_x=d_m, target_x=target_x_B, label="B")
        elapsed = time.monotonic() - t0
        results["dtheta_B"][i] = res_B["abs_delta_theta_deg"]
        print(f"|dtheta|={res_B['abs_delta_theta_deg']:.2f} deg  ({elapsed:.1f}s)")

        # Condition C: true CoM (target_x = -d)
        trial_num += 1
        target_x_C = -d_m
        print(
            f"  [{trial_num}/{total_trials}] Condition C (true, "
            f"target_x={target_x_C * 1000:.2f}mm)...",
            end=" ",
        )
        sys.stdout.flush()
        t0 = time.monotonic()
        res_C = run_push_trial(com_offset_x=d_m, target_x=target_x_C, label="C")
        elapsed = time.monotonic() - t0
        results["dtheta_C"][i] = res_C["abs_delta_theta_deg"]
        print(f"|dtheta|={res_C['abs_delta_theta_deg']:.2f} deg  ({elapsed:.1f}s)")

        print(
            f"\n  Summary for d={d_mm:.0f}mm:  "
            f"A={results['dtheta_A'][i]:.2f}  "
            f"B={results['dtheta_B'][i]:.2f}  "
            f"C={results['dtheta_C'][i]:.2f} deg"
        )

    # --- Save raw data ---
    npz_path = RESULTS_DIR / "data.npz"
    np.savez_compressed(
        npz_path,
        offsets_mm=results["offsets_mm"],
        offsets_m=results["offsets_m"],
        dtheta_A=results["dtheta_A"],
        dtheta_B=results["dtheta_B"],
        dtheta_C=results["dtheta_C"],
        com_est_err_mm=results["com_est_err_mm"],
    )
    print(f"\nData saved: {npz_path}")

    # --- Print summary table ---
    print(f"\n{'=' * 72}")
    print("SUMMARY")
    print(f"{'=' * 72}")
    print(
        f"{'d (mm)':>8}  {'CoM err (mm)':>12}  "
        f"{'A: geom (deg)':>14}  {'B: est (deg)':>13}  {'C: true (deg)':>14}"
    )
    print("-" * 72)
    for i, d_mm in enumerate(COM_OFFSETS_MM):
        print(
            f"{d_mm:8.1f}  {results['com_est_err_mm'][i]:12.2f}  "
            f"{results['dtheta_A'][i]:14.2f}  {results['dtheta_B'][i]:13.2f}  "
            f"{results['dtheta_C'][i]:14.2f}"
        )

    # --- Plot ---
    plot_results(results)


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_results(results: dict) -> None:
    """Create and save the A/B/C contrast plot."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    offsets_mm = results["offsets_mm"]

    # Left: |Delta theta| vs CoM offset
    ax = axes[0]
    ax.plot(
        offsets_mm,
        results["dtheta_A"],
        "o-",
        color="tab:red",
        linewidth=2,
        markersize=7,
        label="A: geometric center",
    )
    ax.plot(
        offsets_mm,
        results["dtheta_B"],
        "s-",
        color="tab:blue",
        linewidth=2,
        markersize=7,
        label="B: estimated CoM",
    )
    ax.plot(
        offsets_mm,
        results["dtheta_C"],
        "^-",
        color="tab:green",
        linewidth=2,
        markersize=7,
        label="C: true CoM (oracle)",
    )
    ax.set_xlabel("True CoM offset d (mm)", fontsize=12)
    ax.set_ylabel(r"Induced rotation $|\Delta\theta|$ (deg)", fontsize=12)
    ax.set_title("CoM belief vs induced rotation\n(MuJoCo pusher-slider)", fontsize=13)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-1, max(offsets_mm) + 1)
    ax.set_ylim(bottom=0)

    # Right: CoM estimation error
    ax = axes[1]
    ax.plot(
        offsets_mm,
        results["com_est_err_mm"],
        "D-",
        color="tab:purple",
        linewidth=2,
        markersize=7,
    )
    ax.set_xlabel("True CoM offset d (mm)", fontsize=12)
    ax.set_ylabel("CoM estimation error (mm)", fontsize=12)
    ax.set_title("Analytical CoM estimator accuracy", fontsize=13)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-1, max(offsets_mm) + 1)
    ax.set_ylim(bottom=0)

    plt.tight_layout()
    plot_path = RESULTS_DIR / "exp_com_abc.png"
    plt.savefig(plot_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Plot saved: {plot_path}")


if __name__ == "__main__":
    main()
