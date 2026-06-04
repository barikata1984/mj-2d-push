"""MuJoCo closed-loop push runner.

Loads the scene, drives the slider to the goal with a controller (default: the
Hogan-2016 MPC) through a Jacobian resolved-rate IK execution layer that holds
tool0 vertical, renders in-loop side-view frames, and writes ``data.npz`` +
``config.json`` to a fresh trial directory.

Coordinate convention: world frame, +y is the push direction. The arm base
rotation is absorbed by the Jacobian IK, so all control math is world-frame.
"""

from __future__ import annotations

import subprocess
import time
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image

from ..config import SimConfig
from ..controllers import make_controller
from ..io import Log, create_trial_dir, dump_config
from ..kinematics import (
    ORI_GAIN,
    damped_pinv,
    get_jacobian6,
    orientation_error,
    pusher_in_slider_body,
    slider_pose_from_data,
)

# gripper_pinch sits PINCH_TO_PAD_FRONT behind the closed pad front face (the
# surface that actually contacts the slider). The keyframe starts the pad ~7 mm
# clear of the slider face, so settling leaves the slider undisturbed.
PINCH_TO_PAD_FRONT = 0.011  # measured at closed + vertical pose (y 0.469->0.480)


class FrameRenderer:
    def __init__(self, m: mujoco.MjModel, cfg: SimConfig, pics_dir: Path):
        self.renderer = mujoco.Renderer(m, height=cfg.render.height, width=cfg.render.width)
        self.cam_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cfg.render.camera)
        self.pics_dir = pics_dir
        self.frame_count = 0

    def capture(self, d: mujoco.MjData) -> None:
        self.renderer.update_scene(d, camera=self.cam_id)
        pixels = self.renderer.render()
        Image.fromarray(pixels).save(self.pics_dir / f"frame_{self.frame_count:06d}.png")
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
        print(f"Video saved: {output_path} ({output_path.stat().st_size / 1024:.0f} KB)")


def move_tip_to(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    tip_site_id: int,
    target_pos: np.ndarray,
    ctrl: np.ndarray,
    cfg: SimConfig,
    renderer: FrameRenderer | None = None,
    max_steps: int = 300,
    tol: float = 0.001,
) -> np.ndarray:
    """Iterative 6-DOF IK move: tip to target_pos while holding tool0 vertical."""
    substeps = int(cfg.mpc.dt / m.opt.timestep)
    tool0_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
    for _ in range(max_steps):
        mujoco.mj_forward(m, d)
        tip = d.site_xpos[tip_site_id].copy()
        perr = target_pos - tip
        oerr = orientation_error(d, tool0_site_id)
        if np.linalg.norm(perr) < tol and np.linalg.norm(oerr) < 0.01:
            break
        pstep = perr * cfg.robot.ik_gain
        pn = np.linalg.norm(pstep)
        if pn > cfg.robot.ik_max_step:
            pstep *= cfg.robot.ik_max_step / pn
        ostep = np.clip(oerr * ORI_GAIN, -0.1, 0.1)
        J = get_jacobian6(m, d, tip_site_id, tool0_site_id)
        dq = damped_pinv(J, cfg.robot.damping) @ np.concatenate([pstep, ostep])
        ctrl = ctrl + dq
        d.ctrl[:6] = ctrl
        d.ctrl[6] = 255  # keep gripper closed
        for __ in range(substeps):
            mujoco.mj_step(m, d)
        if renderer is not None:
            renderer.capture(d)
    return ctrl


def get_pusher_slider_contact_force(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    pad_geom_ids: list[int],
    slider_geom_id: int,
) -> np.ndarray:
    total = np.zeros(3)
    pad_set = set(pad_geom_ids)
    for i in range(d.ncon):
        c = d.contact[i]
        g1, g2 = c.geom1, c.geom2
        if (g1 in pad_set and g2 == slider_geom_id) or (g1 == slider_geom_id and g2 in pad_set):
            force = np.zeros(6)
            mujoco.mj_contactForce(m, d, i, force)
            total += force[:3]
    return total


def run(cfg: SimConfig | None = None) -> tuple[Log, Path]:
    if cfg is None:
        cfg = SimConfig()

    trial_dir = create_trial_dir()
    pics_dir = trial_dir / "pics"
    print(f"Trial directory: {trial_dir}")
    dump_config(cfg, trial_dir)

    m = mujoco.MjModel.from_xml_path(cfg.scene_path)
    d = mujoco.MjData(m)

    tip_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "gripper_pinch")
    tool0_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
    slider_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "slider")
    key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "ready")
    pad_geom_ids = [
        mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in [
            "gripper_right_pad1",
            "gripper_right_pad2",
            "gripper_left_pad1",
            "gripper_left_pad2",
        ]
    ]
    slider_geom_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "slider_geom")

    mujoco.mj_resetDataKeyframe(m, d, key_id)
    mujoco.mj_forward(m, d)

    tip_init = d.site_xpos[tip_site_id].copy()
    slider_pos_init, slider_theta_init = slider_pose_from_data(d, slider_body_id)
    print(f"Initial pusher tip (kinematic): {tip_init}")
    print(f"Initial slider pos: {slider_pos_init}, theta: {np.degrees(slider_theta_init):.2f} deg")

    ctrl = d.ctrl[:6].copy()
    d.ctrl[6] = 255
    substeps = int(cfg.mpc.dt / m.opt.timestep)

    renderer = FrameRenderer(m, cfg, pics_dir)

    # --- Phase -1: Gravity settle ---
    print("\n--- Phase -1: Gravity settle (2s) ---")
    settle_steps = int(2.0 / m.opt.timestep)
    for _ in range(settle_steps):
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    slider_pos_init, _ = slider_pose_from_data(d, slider_body_id)
    slider_face_y = slider_pos_init[1] - 0.03
    print(f"After settle: tip={d.site_xpos[tip_site_id].copy()}, slider={slider_pos_init}")

    # --- Phase 0: Approach (pad front stops 0.5 mm short of the slider face) ---
    approach_target = np.array(
        [slider_pos_init[0], slider_face_y - PINCH_TO_PAD_FRONT - 0.0005, slider_pos_init[2]]
    )
    print(f"\n--- Phase 0: Approach (tip -> {approach_target}) ---")
    ctrl = move_tip_to(m, d, tip_site_id, approach_target, ctrl, cfg, renderer, max_steps=500)
    mujoco.mj_forward(m, d)
    print(f"After approach: tip={d.site_xpos[tip_site_id].copy()}, time={d.time:.2f}s")

    # --- Phase 1: MPC push ---
    print("\n--- Phase 1: MPC push ---")
    mpc = make_controller(
        "mpc",
        slider_dims=cfg.mpc.slider_dims,
        mass=cfg.mpc.mass,
        mu_pusher=cfg.mpc.mu_pusher,
        mu_ground=cfg.mpc.mu_ground,
        dt=cfg.mpc.dt,
        horizon_N=cfg.mpc.horizon,
        Q_weights=cfg.q_weights_arr,
        R_weights=cfg.r_weights_arr,
        Q_terminal_scale=cfg.mpc.q_terminal_scale,
        v_max=cfg.mpc.v_max,
        contact_face=cfg.mpc.contact_face,
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
        contact_force = get_pusher_slider_contact_force(m, d, pad_geom_ids, slider_geom_id)

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

        if slider_pos[1] >= cfg.push.y_goal:
            print(f"Goal reached: slider_y={slider_pos[1]:.4f} >= {cfg.push.y_goal}")
            log.vn.append(0.0)
            log.vt.append(0.0)
            log.target_y.append(cfg.push.y_goal)
            break
        if d.time - t_start > cfg.push.max_sim_time:
            print(f"Max time reached: {cfg.push.max_sim_time}s")
            log.vn.append(0.0)
            log.vt.append(0.0)
            log.target_y.append(cfg.push.y_goal)
            break

        # The closed gripper is mechanically symmetric and aims to push through the
        # slider centerline, so the intended tangential contact offset is ~0. Measuring
        # it from gripper_pinch (~40 mm behind the contact along the face normal) leaks
        # sin(theta)*standoff into px when the slider rotates; that spurious off-center
        # offset makes the MPC predict straight pushing adds +theta and collapse to u=0.
        # Feed the intended centered contact instead. Lateral drift is still corrected
        # via the slider-x state vs target.
        pusher_body_clamped = np.array([0.0, -0.03])

        target_y_now = min(
            slider_pos[1] + cfg.push.push_speed * cfg.mpc.dt * cfg.mpc.horizon,
            cfg.push.y_goal,
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
            vn, vt = cfg.push.push_speed, 0.0
        mpc_call_time_total += time.monotonic() - t0

        vn = max(vn, cfg.push.push_speed)
        log.vn.append(vn)
        log.vt.append(vt)
        log.target_y.append(target_y_now)

        v_world_xy = mpc.contact_to_world(vn, vt, slider_theta)
        z_error = tip_z_ref - tip_pos[2]
        v_des_3d = np.array([v_world_xy[0], v_world_xy[1], 5.0 * z_error])

        # Hold tool0 vertical (R_TOOL0_DES) while executing the MPC's 2D push velocity.
        omega = ORI_GAIN * orientation_error(d, tool0_site_id)
        v6 = np.concatenate([v_des_3d, omega])
        J = get_jacobian6(m, d, tip_site_id, tool0_site_id)
        dq = damped_pinv(J, cfg.robot.damping) @ (v6 * cfg.mpc.dt)
        ctrl = ctrl + dq
        d.ctrl[:6] = ctrl
        d.ctrl[6] = 255  # keep gripper closed

        for _ in range(substeps):
            mujoco.mj_step(m, d)

        if step_count % cfg.render.every_n_mpc == 0:
            renderer.capture(d)

        step_count += 1
        if step_count % 100 == 0:
            print(
                f"  t={d.time:.2f}s  slider=({slider_pos[0]:.4f}, {slider_pos[1]:.4f}) "
                f"theta={np.degrees(slider_theta):.2f}deg  "
                f"vn={vn:.4f} vt={vt:.4f}  contacts={d.ncon}"
            )

    renderer.close()

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
        f"y_from_goal={abs(slider_final[1] - cfg.push.y_goal):.4f}m, "
        f"theta={abs(np.degrees(theta_final)):.2f}deg"
    )
    print(f"Total sim time: {d.time:.2f}s, MPC steps: {step_count}")
    if step_count > 0:
        print(f"Avg MPC time: {mpc_call_time_total / step_count * 1000:.1f}ms")
    print(f"Frames rendered: {renderer.frame_count}")

    log.to_npz(trial_dir / "data.npz")
    print(f"Data saved: {trial_dir / 'data.npz'}")

    print("\nEncoding video...")
    encode_video(pics_dir, trial_dir / "push_sim.mp4", cfg.render.fps)

    return log, trial_dir
