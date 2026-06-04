"""
Experiment: compare Delta-theta (rotation) trends between Stage 0 analytical
model and Stage 1 MuJoCo simulation.

Both models use matched slider parameters (80x60 mm, 1.05 kg).
The pusher applies constant +y velocity.

Two conditions:
  A) Center push (px=0): symmetry test -- analytical predicts theta=0
  B) Off-center push (px=+20mm): both models produce nonzero rotation
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

# ---------------------------------------------------------------------------
# Shared physical parameters
# ---------------------------------------------------------------------------
SLIDER_A = 0.08  # slider x-dimension [m]
SLIDER_B = 0.06  # slider y-dimension [m]
SLIDER_MASS = 1.05  # [kg]
MU_PUSHER = 0.3  # pusher-slider friction
MU_GROUND = 0.35  # slider-ground friction
PUSH_SPEED = 0.02  # constant pusher velocity in +y [m/s]
DT_ANALYTICAL = 0.001  # analytical model timestep [s] (fine for accuracy)
T_PUSH = 2.0  # push duration [s]

# Limit surface characteristic length: radius of gyration for uniform pressure
C_LS = np.sqrt((SLIDER_A**2 + SLIDER_B**2) / 12.0)


# ---------------------------------------------------------------------------
# Stage 0: analytical quasi-static model (from push_com_sim.py, adapted)
# ---------------------------------------------------------------------------
def _rot2(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def _body_twist(
    contact_rel_com: np.ndarray, vp_body: np.ndarray, c: float
) -> tuple[float, float, float]:
    """Sticking-contact body twist under ellipsoidal limit surface."""
    px, py = contact_rel_com
    M = np.array(
        [
            [1.0 + py * py / c**2, -px * py / c**2],
            [-px * py / c**2, 1.0 + px * px / c**2],
        ]
    )
    vx, vy = np.linalg.solve(M, vp_body)
    omega = (px * vy - py * vx) / c**2
    return float(vx), float(vy), float(omega)


def run_stage0(
    contact_geom: np.ndarray,
    dt: float = DT_ANALYTICAL,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Run analytical pusher-slider for a straight +y push, CoM at geometric center.

    Args:
        contact_geom: contact point in slider body frame, e.g. (px, -0.03).
        dt: integration timestep.

    Returns:
        (times, xs, ys, thetas) arrays of shape (N+1,).
    """
    steps = int(T_PUSH / dt)
    push_dir_world = np.array([0.0, 1.0])
    contact_rel_com = contact_geom.copy()  # CoM at origin

    pose = np.array([0.0, 0.0, 0.0])
    times = np.zeros(steps + 1)
    xs = np.zeros(steps + 1)
    ys = np.zeros(steps + 1)
    thetas = np.zeros(steps + 1)

    for i in range(steps):
        theta = pose[2]
        vp_world = PUSH_SPEED * push_dir_world
        vp_body = _rot2(theta).T @ vp_world
        vx, vy, omega = _body_twist(contact_rel_com, vp_body, C_LS)

        v_world = _rot2(theta) @ np.array([vx, vy])
        pose[:2] += v_world * dt
        pose[2] += omega * dt

        times[i + 1] = (i + 1) * dt
        xs[i + 1] = pose[0]
        ys[i + 1] = pose[1]
        thetas[i + 1] = pose[2]

    return times, xs, ys, thetas


# ---------------------------------------------------------------------------
# Stage 1: MuJoCo simulation helpers
# ---------------------------------------------------------------------------
def _slider_pose(d: mujoco.MjData, body_id: int) -> tuple[np.ndarray, float]:
    pos = d.xpos[body_id].copy()
    quat = d.xquat[body_id].copy()
    w, qx, qy, qz = quat
    theta = np.arctan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
    return pos, theta


def _damped_pinv(J: np.ndarray, damping: float = 1e-3) -> np.ndarray:
    JJT = J @ J.T
    return J.T @ np.linalg.inv(JJT + damping**2 * np.eye(JJT.shape[0]))


def _get_jacobian(m: mujoco.MjModel, d: mujoco.MjData, site_id: int) -> np.ndarray:
    jacp = np.zeros((3, m.nv))
    mujoco.mj_jacSite(m, d, jacp, None, site_id)
    return jacp[:, :6]


def _move_tip_to(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    tip_site_id: int,
    target: np.ndarray,
    ctrl: np.ndarray,
    mpc_dt: float = 0.03,
    max_steps: int = 300,
    tol: float = 0.001,
) -> np.ndarray:
    """Move pusher tip to target via iterative IK."""
    substeps = int(mpc_dt / m.opt.timestep)
    for _ in range(max_steps):
        mujoco.mj_forward(m, d)
        tip = d.site_xpos[tip_site_id].copy()
        err = target - tip
        if np.linalg.norm(err) < tol:
            break
        step = err * 2.0
        step_norm = np.linalg.norm(step)
        if step_norm > 0.02:
            step *= 0.02 / step_norm
        J = _get_jacobian(m, d, tip_site_id)
        dq = _damped_pinv(J) @ step
        ctrl = ctrl + dq
        d.ctrl[:6] = ctrl
        for __ in range(substeps):
            mujoco.mj_step(m, d)
    return ctrl


def run_mujoco(
    px_offset: float = 0.0,
    scene_path: str = "/workspace/stage1_scene.xml",
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Run MuJoCo sim with constant +y pusher velocity, record slider pose.

    Args:
        px_offset: x-offset of contact from slider center on -y face [m].
                   The pusher approaches (slider_x + px_offset, slider_face_y).

    Returns:
        (times, xs, ys, thetas) arrays relative to push start.
    """
    m = mujoco.MjModel.from_xml_path(scene_path)
    d = mujoco.MjData(m)

    tip_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "pusher_tip_site")
    slider_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "slider")
    key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "ready")

    mujoco.mj_resetDataKeyframe(m, d, key_id)
    mujoco.mj_forward(m, d)

    ctrl = d.ctrl[:6].copy()
    tip_init = d.site_xpos[tip_site_id].copy()
    slider_pos_init, slider_theta_init = _slider_pose(d, slider_body_id)

    print(
        f"  [MuJoCo] Initial slider: pos={slider_pos_init}, "
        f"theta={np.degrees(slider_theta_init):.3f} deg"
    )

    # Approach target: center of -y face, offset by px_offset in x
    slider_face_y = slider_pos_init[1] - 0.03
    approach_target = np.array(
        [
            slider_pos_init[0] + px_offset,
            slider_face_y - 0.0005,
            tip_init[2],
        ]
    )
    print(f"  [MuJoCo] Approach target: {approach_target}")

    mpc_dt = 0.03
    ctrl = _move_tip_to(m, d, tip_site_id, approach_target, ctrl, mpc_dt)

    mujoco.mj_forward(m, d)
    tip_after = d.site_xpos[tip_site_id].copy()
    print(f"  [MuJoCo] After approach:  tip={tip_after}")

    slider_pos_ref, slider_theta_ref = _slider_pose(d, slider_body_id)

    # --- Constant +y push ---
    ctrl_dt = mpc_dt
    ctrl_substeps = int(ctrl_dt / m.opt.timestep)
    n_ctrl_steps = int(T_PUSH / ctrl_dt)
    tip_z_ref = d.site_xpos[tip_site_id][2]

    times_list: list[float] = [0.0]
    xs_list: list[float] = [0.0]
    ys_list: list[float] = [0.0]
    thetas_list: list[float] = [0.0]

    t_push_start = d.time

    for _ in range(n_ctrl_steps):
        mujoco.mj_forward(m, d)
        tip_pos = d.site_xpos[tip_site_id].copy()

        z_err = tip_z_ref - tip_pos[2]
        v_des = np.array([0.0, PUSH_SPEED, 5.0 * z_err])

        J = _get_jacobian(m, d, tip_site_id)
        dq = _damped_pinv(J) @ (v_des * ctrl_dt)
        ctrl = ctrl + dq
        d.ctrl[:6] = ctrl

        for __ in range(ctrl_substeps):
            mujoco.mj_step(m, d)

        slider_pos, slider_theta = _slider_pose(d, slider_body_id)
        t_elapsed = d.time - t_push_start

        dx = slider_pos[0] - slider_pos_ref[0]
        dy = slider_pos[1] - slider_pos_ref[1]
        dtheta = slider_theta - slider_theta_ref

        times_list.append(t_elapsed)
        xs_list.append(dx)
        ys_list.append(dy)
        thetas_list.append(dtheta)

    return (
        np.array(times_list),
        np.array(xs_list),
        np.array(ys_list),
        np.array(thetas_list),
    )


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_two_conditions(
    results: dict[str, dict[str, tuple[np.ndarray, ...]]],
    out_dir: Path,
) -> None:
    """Create a 2x3 comparison plot (2 conditions x 3 variables)."""
    conditions = list(results.keys())
    fig, axes = plt.subplots(len(conditions), 3, figsize=(15, 5 * len(conditions)))
    if len(conditions) == 1:
        axes = axes[np.newaxis, :]

    for row, cond_name in enumerate(conditions):
        data = results[cond_name]
        t0, x0, y0, th0 = data["stage0"]
        t1, x1, y1, th1 = data["stage1"]
        th0_deg = np.degrees(th0)
        th1_deg = np.degrees(th1)

        # theta(t)
        ax = axes[row, 0]
        ax.plot(t0, th0_deg, "b-", linewidth=1.5, label="Stage 0 (analytical)")
        ax.plot(t1, th1_deg, "r-", linewidth=1.5, label="Stage 1 (MuJoCo)")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel(r"$\Delta\theta$ [deg]")
        ax.set_title(f"{cond_name}: " + r"$\Delta\theta(t)$")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # y(t)
        ax = axes[row, 1]
        ax.plot(t0, y0, "b-", linewidth=1.5, label="Stage 0")
        ax.plot(t1, y1, "r-", linewidth=1.5, label="Stage 1")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("y [m]")
        ax.set_title(f"{cond_name}: y-displacement")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

        # x(t)
        ax = axes[row, 2]
        ax.plot(t0, x0, "b-", linewidth=1.5, label="Stage 0")
        ax.plot(t1, x1, "r-", linewidth=1.5, label="Stage 1")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("x [m]")
        ax.set_title(f"{cond_name}: x-drift")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    fig.suptitle(
        f"Stage 0 vs Stage 1: straight push comparison\n"
        f"Slider {SLIDER_A * 1e3:.0f}x{SLIDER_B * 1e3:.0f} mm, "
        f"m={SLIDER_MASS} kg, "
        f"$\\mu_p$={MU_PUSHER}, "
        f"c={C_LS:.4f} m, "
        f"v={PUSH_SPEED} m/s",
        fontsize=11,
    )
    plt.tight_layout()
    out_path = out_dir / "dtheta_comparison.png"
    plt.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"\nPlot saved: {out_path}")


def compute_stats(
    cond_name: str,
    contact_geom: np.ndarray,
    t0: np.ndarray,
    th0: np.ndarray,
    t1: np.ndarray,
    th1: np.ndarray,
) -> None:
    """Print summary statistics for one condition."""
    th0_deg = np.degrees(th0)
    th1_deg = np.degrees(th1)

    print(f"\n--- {cond_name} ---")
    print(f"  Contact point (body): ({contact_geom[0]:.3f}, {contact_geom[1]:.3f})")
    print(f"  Stage 0 final theta: {th0_deg[-1]:+.4f} deg")
    print(f"  Stage 1 final theta: {th1_deg[-1]:+.4f} deg")

    if abs(th0_deg[-1]) > 0.01:
        print(f"  Ratio (Stage1 / Stage0): {th1_deg[-1] / th0_deg[-1]:.3f}")

    # Correlation on interpolated time grid
    th1_interp = np.interp(t0, t1, np.degrees(th1))
    if len(t0) > 2 and np.std(th0_deg) > 1e-10 and np.std(th1_interp) > 1e-10:
        corr = np.corrcoef(th0_deg, th1_interp)[0, 1]
        print(f"  Pearson correlation: {corr:.4f}")
    else:
        print("  Pearson correlation: N/A (insufficient variance)")

    print(f"  Stage 0 max |theta|: {np.max(np.abs(th0_deg)):.4f} deg")
    print(f"  Stage 1 max |theta|: {np.max(np.abs(th1_deg)):.4f} deg")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    out_dir = Path("/workspace/results/exp_dtheta_comparison")
    out_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Experiment: Delta-theta comparison (Stage 0 vs Stage 1)")
    print("=" * 60)
    print(
        f"Slider: {SLIDER_A * 1e3:.0f} x {SLIDER_B * 1e3:.0f} mm, mass={SLIDER_MASS} kg"
    )
    print(f"c (limit surface) = {C_LS:.5f} m")
    print(f"mu_pusher={MU_PUSHER}, mu_ground={MU_GROUND}")
    print(f"Push speed={PUSH_SPEED} m/s, duration={T_PUSH} s")

    # Define conditions: (label, contact_body_frame, px_offset_for_mujoco)
    conditions = [
        ("A: center push (px=0)", np.array([0.0, -0.03]), 0.0),
        ("B: off-center push (px=+20mm)", np.array([0.02, -0.03]), 0.02),
    ]

    results: dict[str, dict[str, tuple[np.ndarray, ...]]] = {}

    for cond_name, contact_geom, px_offset in conditions:
        print(f"\n{'=' * 60}")
        print(f"Condition: {cond_name}")
        print(f"{'=' * 60}")

        # Stage 0
        print("  Running Stage 0 (analytical) ...")
        t0, x0, y0, th0 = run_stage0(contact_geom)
        print(
            f"  Stage 0 final: x={x0[-1]:.6f}, y={y0[-1]:.6f}, "
            f"theta={np.degrees(th0[-1]):.4f} deg"
        )

        # Stage 1
        print("  Running Stage 1 (MuJoCo) ...")
        t1, x1, y1, th1 = run_mujoco(px_offset=px_offset)
        print(
            f"  Stage 1 final: dx={x1[-1]:.6f}, dy={y1[-1]:.6f}, "
            f"dtheta={np.degrees(th1[-1]):.4f} deg"
        )

        results[cond_name] = {
            "stage0": (t0, x0, y0, th0),
            "stage1": (t1, x1, y1, th1),
        }

    # --- Summary statistics ---
    print("\n" + "=" * 60)
    print("SUMMARY STATISTICS")
    print("=" * 60)
    for (cond_name, contact_geom, _), data in zip(conditions, results.values()):
        t0, x0, y0, th0 = data["stage0"]
        t1, x1, y1, th1 = data["stage1"]
        compute_stats(cond_name, contact_geom, t0, th0, t1, th1)

    # --- Plot ---
    plot_two_conditions(results, out_dir)

    # --- Save raw data ---
    npz_data = {}
    for i, (cond_name, _, _) in enumerate(conditions):
        prefix = f"cond{i}"
        data = results[cond_name]
        t0, x0, y0, th0 = data["stage0"]
        t1, x1, y1, th1 = data["stage1"]
        npz_data[f"{prefix}_stage0_time"] = t0
        npz_data[f"{prefix}_stage0_x"] = x0
        npz_data[f"{prefix}_stage0_y"] = y0
        npz_data[f"{prefix}_stage0_theta"] = th0
        npz_data[f"{prefix}_stage1_time"] = t1
        npz_data[f"{prefix}_stage1_x"] = x1
        npz_data[f"{prefix}_stage1_y"] = y1
        npz_data[f"{prefix}_stage1_theta"] = th1

    npz_path = out_dir / "data.npz"
    np.savez_compressed(npz_path, **npz_data)
    print(f"Data saved: {npz_path}")


if __name__ == "__main__":
    main()
