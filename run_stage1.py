"""
Stage 1: UR5e pusher-slider simulation with MPC control.

Loads the MuJoCo scene, runs the MPC controller in a closed loop,
renders frames, and produces video + data output.

Output structure:
  results/<trial_id>/
    pics/         frame images (PNG)
    push_sim.mp4  rendered video
    data.npz      all time-series data for post-hoc analysis
    result.png    summary plot
    config.json   simulation parameters
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np
from PIL import Image

sys.path.insert(0, "/workspace")
from pusher_slider_mpc import PusherSliderMPC


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
@dataclass
class Config:
    scene_path: str = "/workspace/stage1_scene.xml"

    # Push parameters
    push_speed: float = 0.03
    y_start: float = 0.235
    y_goal: float = 0.78
    max_sim_time: float = 30.0

    # MPC parameters
    mpc_dt: float = 0.03
    mpc_horizon: int = 10
    mpc_v_max: float = 0.08

    # Robot control
    damping: float = 1e-3
    ik_gain: float = 2.0
    ik_max_step: float = 0.02

    # Rendering
    render_width: int = 960
    render_height: int = 720
    render_camera: str = "side"
    render_fps: int = 30
    render_every_n_mpc: int = 1


# ---------------------------------------------------------------------------
# Trial directory setup
# ---------------------------------------------------------------------------
def create_trial_dir(base: str = "/workspace/results") -> Path:
    ts = datetime.now(tz=timezone.utc).strftime("%Y%m%d_%H%M%S")
    trial_dir = Path(base) / ts
    (trial_dir / "pics").mkdir(parents=True, exist_ok=True)
    return trial_dir


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def slider_pose_from_data(
    d: mujoco.MjData, slider_body_id: int
) -> tuple[np.ndarray, float]:
    pos = d.xpos[slider_body_id].copy()
    quat = d.xquat[slider_body_id].copy()
    w, qx, qy, qz = quat
    theta = np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    return pos, theta


def pusher_in_slider_body(
    tip_world: np.ndarray, slider_pos: np.ndarray, theta: float
) -> np.ndarray:
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


# ---------------------------------------------------------------------------
# Data logger
# ---------------------------------------------------------------------------
@dataclass
class Log:
    time: list[float] = field(default_factory=list)
    slider_x: list[float] = field(default_factory=list)
    slider_y: list[float] = field(default_factory=list)
    slider_theta: list[float] = field(default_factory=list)
    slider_quat: list[list[float]] = field(default_factory=list)
    pusher_x: list[float] = field(default_factory=list)
    pusher_y: list[float] = field(default_factory=list)
    pusher_z: list[float] = field(default_factory=list)
    pusher_body_px: list[float] = field(default_factory=list)
    pusher_body_py: list[float] = field(default_factory=list)
    vn: list[float] = field(default_factory=list)
    vt: list[float] = field(default_factory=list)
    target_y: list[float] = field(default_factory=list)
    joint_pos: list[list[float]] = field(default_factory=list)
    joint_ctrl: list[list[float]] = field(default_factory=list)
    contact_count: list[int] = field(default_factory=list)
    contact_forces: list[list[float]] = field(default_factory=list)

    def to_npz(self, path: Path) -> None:
        np.savez_compressed(
            path,
            time=np.array(self.time),
            slider_x=np.array(self.slider_x),
            slider_y=np.array(self.slider_y),
            slider_theta=np.array(self.slider_theta),
            slider_quat=np.array(self.slider_quat)
            if self.slider_quat
            else np.empty((0, 4)),
            pusher_x=np.array(self.pusher_x),
            pusher_y=np.array(self.pusher_y),
            pusher_z=np.array(self.pusher_z),
            pusher_body_px=np.array(self.pusher_body_px),
            pusher_body_py=np.array(self.pusher_body_py),
            vn=np.array(self.vn),
            vt=np.array(self.vt),
            target_y=np.array(self.target_y),
            joint_pos=np.array(self.joint_pos) if self.joint_pos else np.empty((0, 6)),
            joint_ctrl=np.array(self.joint_ctrl)
            if self.joint_ctrl
            else np.empty((0, 6)),
            contact_count=np.array(self.contact_count),
            contact_forces=np.array(self.contact_forces)
            if self.contact_forces
            else np.empty((0, 3)),
        )


# ---------------------------------------------------------------------------
# Renderer
# ---------------------------------------------------------------------------
class FrameRenderer:
    def __init__(self, m: mujoco.MjModel, cfg: Config, pics_dir: Path):
        self.renderer = mujoco.Renderer(
            m, height=cfg.render_height, width=cfg.render_width
        )
        self.cam_id = mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_CAMERA, cfg.render_camera
        )
        self.pics_dir = pics_dir
        self.frame_count = 0

    def capture(self, d: mujoco.MjData) -> None:
        self.renderer.update_scene(d, camera=self.cam_id)
        pixels = self.renderer.render()
        img = Image.fromarray(pixels)
        img.save(self.pics_dir / f"frame_{self.frame_count:06d}.png")
        self.frame_count += 1

    def close(self) -> None:
        del self.renderer


def encode_video(pics_dir: Path, output_path: Path, fps: int) -> None:
    cmd = [
        "ffmpeg",
        "-y",
        "-framerate",
        str(fps),
        "-i",
        str(pics_dir / "frame_%06d.png"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-crf",
        "18",
        "-preset",
        "fast",
        str(output_path),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"ffmpeg error: {result.stderr[-500:]}")
    else:
        print(
            f"Video saved: {output_path} ({output_path.stat().st_size / 1024:.0f} KB)"
        )


# ---------------------------------------------------------------------------
# Move tip to target via iterative IK
# ---------------------------------------------------------------------------
def move_tip_to(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    tip_site_id: int,
    target_pos: np.ndarray,
    ctrl: np.ndarray,
    cfg: Config,
    renderer: FrameRenderer | None = None,
    max_steps: int = 300,
    tol: float = 0.001,
) -> np.ndarray:
    substeps = int(cfg.mpc_dt / m.opt.timestep)
    for _ in range(max_steps):
        mujoco.mj_forward(m, d)
        tip = d.site_xpos[tip_site_id].copy()
        err = target_pos - tip
        if np.linalg.norm(err) < tol:
            break
        step = err * cfg.ik_gain
        step_norm = np.linalg.norm(step)
        if step_norm > cfg.ik_max_step:
            step *= cfg.ik_max_step / step_norm
        J = get_jacobian(m, d, tip_site_id)
        dq = damped_pinv(J, cfg.damping) @ step
        ctrl = ctrl + dq
        d.ctrl[:6] = ctrl
        for __ in range(substeps):
            mujoco.mj_step(m, d)
        if renderer is not None:
            renderer.capture(d)
    return ctrl


# ---------------------------------------------------------------------------
# Extract contact force between pusher_tip and slider_geom
# ---------------------------------------------------------------------------
def get_pusher_slider_contact_force(
    m: mujoco.MjModel, d: mujoco.MjData, pusher_geom_id: int, slider_geom_id: int
) -> np.ndarray:
    total = np.zeros(3)
    for i in range(d.ncon):
        c = d.contact[i]
        g1, g2 = c.geom1, c.geom2
        if (g1 == pusher_geom_id and g2 == slider_geom_id) or (
            g1 == slider_geom_id and g2 == pusher_geom_id
        ):
            force = np.zeros(6)
            mujoco.mj_contactForce(m, d, i, force)
            total += force[:3]
    return total


# ---------------------------------------------------------------------------
# Main simulation
# ---------------------------------------------------------------------------
def run(cfg: Config | None = None) -> tuple[Log, Path]:
    if cfg is None:
        cfg = Config()

    trial_dir = create_trial_dir()
    pics_dir = trial_dir / "pics"
    print(f"Trial directory: {trial_dir}")

    with open(trial_dir / "config.json", "w") as f:
        json.dump(asdict(cfg), f, indent=2)

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
    slider_pos_init, slider_theta_init = slider_pose_from_data(d, slider_body_id)
    print(f"Initial pusher tip: {tip_init}")
    print(
        f"Initial slider pos: {slider_pos_init}, "
        f"theta: {np.degrees(slider_theta_init):.2f} deg"
    )

    ctrl = d.ctrl[:6].copy()
    substeps = int(cfg.mpc_dt / m.opt.timestep)

    renderer = FrameRenderer(m, cfg, pics_dir)

    # --- Phase 0: Approach ---
    slider_face_y = slider_pos_init[1] - 0.03
    approach_target = np.array(
        [slider_pos_init[0], slider_face_y - 0.0005, tip_init[2]]
    )
    print(f"\n--- Phase 0: Approach (tip -> y={approach_target[1]:.4f}) ---")
    ctrl = move_tip_to(
        m, d, tip_site_id, approach_target, ctrl, cfg, renderer, max_steps=200
    )
    mujoco.mj_forward(m, d)
    tip_after = d.site_xpos[tip_site_id].copy()
    print(f"After approach: tip={tip_after}, time={d.time:.2f}s")

    # --- Phase 1: MPC push ---
    print("\n--- Phase 1: MPC push ---")

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

    log = Log()
    t_start = d.time
    step_count = 0
    mpc_call_time_total = 0.0
    tip_z_ref = d.site_xpos[tip_site_id][2]

    while True:
        mujoco.mj_forward(m, d)

        tip_pos = d.site_xpos[tip_site_id].copy()
        slider_pos, slider_theta = slider_pose_from_data(d, slider_body_id)
        slider_quat = d.xquat[slider_body_id].copy()
        pusher_body = pusher_in_slider_body(tip_pos, slider_pos, slider_theta)
        contact_force = get_pusher_slider_contact_force(
            m, d, pusher_geom_id, slider_geom_id
        )

        log.time.append(d.time)
        log.slider_x.append(slider_pos[0])
        log.slider_y.append(slider_pos[1])
        log.slider_theta.append(slider_theta)
        log.slider_quat.append(slider_quat.tolist())
        log.pusher_x.append(tip_pos[0])
        log.pusher_y.append(tip_pos[1])
        log.pusher_z.append(tip_pos[2])
        log.pusher_body_px.append(pusher_body[0])
        log.pusher_body_py.append(pusher_body[1])
        log.joint_pos.append(d.qpos[:6].copy().tolist())
        log.joint_ctrl.append(d.ctrl[:6].copy().tolist())
        log.contact_count.append(d.ncon)
        log.contact_forces.append(contact_force.tolist())

        if slider_pos[1] >= cfg.y_goal:
            print(f"Goal reached: slider_y={slider_pos[1]:.4f} >= {cfg.y_goal}")
            log.vn.append(0.0)
            log.vt.append(0.0)
            log.target_y.append(cfg.y_goal)
            break
        if d.time - t_start > cfg.max_sim_time:
            print(f"Max time reached: {cfg.max_sim_time}s")
            log.vn.append(0.0)
            log.vt.append(0.0)
            log.target_y.append(cfg.y_goal)
            break

        pusher_body_clamped = np.array([np.clip(pusher_body[0], -0.038, 0.038), -0.03])

        target_y_now = min(
            slider_pos[1] + cfg.push_speed * cfg.mpc_dt * cfg.mpc_horizon, cfg.y_goal
        )
        current_target = np.array([0.0, target_y_now, 0.0])

        t0 = time.monotonic()
        try:
            vn, vt = mpc.compute_control(
                slider_pose=np.array([slider_pos[0], slider_pos[1], slider_theta]),
                pusher_pos_body=pusher_body_clamped,
                target_pose=current_target,
            )
        except Exception as e:
            print(f"MPC failed at step {step_count}: {e}")
            vn, vt = cfg.push_speed, 0.0
        mpc_call_time_total += time.monotonic() - t0

        vn = max(vn, 0.005)
        log.vn.append(vn)
        log.vt.append(vt)
        log.target_y.append(target_y_now)

        v_world_xy = mpc.contact_to_world(vn, vt, slider_theta)
        z_error = tip_z_ref - tip_pos[2]
        v_des_3d = np.array([v_world_xy[0], v_world_xy[1], 5.0 * z_error])

        J = get_jacobian(m, d, tip_site_id)
        dq = damped_pinv(J, cfg.damping) @ (v_des_3d * cfg.mpc_dt)
        ctrl = ctrl + dq
        d.ctrl[:6] = ctrl

        for _ in range(substeps):
            mujoco.mj_step(m, d)

        if step_count % cfg.render_every_n_mpc == 0:
            renderer.capture(d)

        step_count += 1
        if step_count % 100 == 0:
            print(
                f"  t={d.time:.2f}s  slider=({slider_pos[0]:.4f}, {slider_pos[1]:.4f}) "
                f"theta={np.degrees(slider_theta):.2f}deg  "
                f"vn={vn:.4f} vt={vt:.4f}  contacts={d.ncon}"
            )

    renderer.close()

    # --- Final state ---
    mujoco.mj_forward(m, d)
    slider_final, theta_final = slider_pose_from_data(d, slider_body_id)
    tip_final = d.site_xpos[tip_site_id].copy()

    print(f"\n{'=' * 60}")
    print("FINAL STATE")
    print(f"{'=' * 60}")
    print(
        f"Slider: x={slider_final[0]:.4f}, y={slider_final[1]:.4f}, "
        f"theta={np.degrees(theta_final):.2f} deg"
    )
    print(f"Pusher tip: {tip_final}")
    print(
        f"Errors: x={abs(slider_final[0]):.4f}m, "
        f"y_from_goal={abs(slider_final[1] - cfg.y_goal):.4f}m, "
        f"theta={abs(np.degrees(theta_final)):.2f}deg"
    )
    print(f"Total sim time: {d.time:.2f}s, MPC steps: {step_count}")
    if step_count > 0:
        print(f"Avg MPC time: {mpc_call_time_total / step_count * 1000:.1f}ms")
    print(f"Frames rendered: {renderer.frame_count}")

    # --- Save data ---
    log.to_npz(trial_dir / "data.npz")
    print(f"Data saved: {trial_dir / 'data.npz'}")

    # --- Encode video ---
    print("\nEncoding video...")
    encode_video(pics_dir, trial_dir / "push_sim.mp4", cfg.render_fps)

    return log, trial_dir


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
def plot_results(log: Log, trial_dir: Path, cfg: Config) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))

    t = np.array(log.time)
    sx = np.array(log.slider_x)
    sy = np.array(log.slider_y)
    stheta = np.degrees(np.array(log.slider_theta))
    px = np.array(log.pusher_x)
    py_arr = np.array(log.pusher_y)
    pz = np.array(log.pusher_z)

    # (0,0) Slider trajectory x vs y
    ax = axes[0, 0]
    ax.plot(sx, sy, "b-", linewidth=1.5, label="Slider trajectory")
    ax.plot([0, 0], [cfg.y_start, cfg.y_goal], "r--", linewidth=1, label="Target (x=0)")
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


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    cfg = Config()
    log, trial_dir = run(cfg)
    plot_results(log, trial_dir, cfg)
