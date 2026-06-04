"""Six-pane diagnostic summary plot for a push run (``result.png``)."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ..config import SimConfig
from ..io import Log


def plot_results(log: Log, trial_dir: Path, cfg: SimConfig) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    t = np.array(log.time)
    sx = np.array(log.slider_x)
    sy = np.array(log.slider_y)
    stheta = np.degrees(np.array(log.slider_theta))
    pz = np.array(log.pusher_z)

    # (0,0) Slider trajectory x vs y
    ax = axes[0, 0]
    ax.plot(sx, sy, "b-", linewidth=1.5, label="Slider trajectory")
    ax.plot(
        [0, 0],
        [cfg.push.y_start, cfg.push.y_goal],
        "r--",
        linewidth=1,
        label="Target (x=0)",
    )
    n_arrows = min(20, len(t))
    if n_arrows > 0:
        arrow_idx = np.linspace(0, len(t) - 1, n_arrows, dtype=int)
        for i in arrow_idx:
            theta_rad = np.radians(stheta[i])
            ddx = 0.01 * np.cos(theta_rad)
            ddy = 0.01 * np.sin(theta_rad)
            ax.annotate(
                "",
                xy=(sx[i] + ddx, sy[i] + ddy),
                xytext=(sx[i], sy[i]),
                arrowprops=dict(arrowstyle="->", color="green", lw=1),
            )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Slider trajectory")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # (0,1) Slider theta over time
    ax = axes[0, 1]
    ax.plot(t, stheta, "b-", linewidth=1)
    ax.axhline(0, color="r", linestyle="--", linewidth=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Theta (deg)")
    ax.set_title("Slider orientation")
    ax.grid(True, alpha=0.3)

    # (0,2) MPC control inputs
    ax = axes[0, 2]
    vn = np.array(log.vn)
    vt_arr = np.array(log.vt)
    ax.plot(t, vn, "r-", linewidth=1, label="vn (normal)")
    ax.plot(t, vt_arr, "b-", linewidth=1, label="vt (tangent)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title("MPC control inputs")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (1,0) Pusher body-frame position
    ax = axes[1, 0]
    bpx = np.array(log.pusher_body_px)
    bpy = np.array(log.pusher_body_py)
    ax.plot(t, bpx, "r-", linewidth=1, label="px (body)")
    ax.plot(t, bpy, "b-", linewidth=1, label="py (body)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("Pusher in slider body frame")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (1,1) Contact forces
    ax = axes[1, 1]
    cf = np.array(log.contact_forces)
    if cf.ndim == 2 and cf.shape[0] > 0:
        ax.plot(t, cf[:, 0], "r-", linewidth=1, label="f_n")
        ax.plot(t, cf[:, 1], "g-", linewidth=1, label="f_t1")
        ax.plot(t, cf[:, 2], "b-", linewidth=1, label="f_t2")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (N)")
    ax.set_title("Pusher-slider contact force")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # (1,2) Pusher z height
    ax = axes[1, 2]
    ax.plot(t, pz, "b-", linewidth=1)
    ax.axhline(pz[0] if len(pz) > 0 else 0, color="r", linestyle="--", linewidth=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("z (m)")
    ax.set_title("Pusher tip z (height stability)")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = trial_dir / "result.png"
    plt.savefig(out_path, dpi=150)
    print(f"Plot saved: {out_path}")
