"""Compute a ready keyframe for the UR5e + Robotiq 2F-85 gripper.

Uses 6-DOF damped-pseudoinverse IK (position + tool0 orientation) to place
the gripper as a vertical pusher: gripper_pinch behind the slider's -y face
on the work surface, with tool0 held straight down (R_TOOL0_DES) so the closed
finger axis is perpendicular to the push direction. The orientation constraint
and Jacobian helpers are shared with run_stage1 so the generated keyframe and
the runtime execution layer agree on the vertical-pusher convention.

The pinch is retracted so the closed pad front stops ~7 mm clear of the slider
face; otherwise the gravity settle at the start of the push run rams the slider.

The gripper is physically closed via simulation before IK so that the equality
constraints settle the 4-bar linkage.

Gravity handling:
  The UR5e PD actuators (gain=2000, bias1=-2000) produce ~6mm tip sag
  under gravity at the operating configuration. Analytical gravity
  compensation (ctrl = q + gravity(q)/gain) is applied to ctrl.
  Keyframe qpos stores the IK solution (exact FK position at t=0);
  the ~6mm PD droop during simulation is handled by the MPC controller.
"""

from __future__ import annotations

import mujoco
import numpy as np

from .. import paths
from ..kinematics import get_jacobian6, orientation_error


def damped_pinv(J: np.ndarray, damping: float = 1e-3) -> np.ndarray:
    JJT = J @ J.T
    return J.T @ np.linalg.inv(JJT + damping**2 * np.eye(JJT.shape[0]))


def close_gripper_sim(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    settle_time: float = 2.0,
) -> np.ndarray:
    """Simulate gripper closing and return settled gripper joint values (qpos[6:14])."""
    mujoco.mj_resetData(m, d)
    d.qpos[:6] = np.array([-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
    d.ctrl[:6] = d.qpos[:6].copy()
    d.ctrl[6] = 255.0
    substeps = int(settle_time / m.opt.timestep)
    for _ in range(substeps):
        mujoco.mj_step(m, d)
    gripper_qpos = d.qpos[6:14].copy()
    print(f"Gripper closed joint values: {np.round(gripper_qpos, 6)}")
    return gripper_qpos


def solve_ik(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    tip_site_id: int,
    ori_site_id: int,
    target_pos: np.ndarray,
    q_init: np.ndarray,
    gripper_qpos: np.ndarray,
    max_iter: int = 2000,
    pos_tol: float = 5e-4,
    ori_tol: float = 1e-3,
    gain: float = 1.0,
    max_step: float = 0.05,
    damping: float = 1e-3,
) -> tuple[np.ndarray, float, float]:
    """6-DOF IK: drive gripper_pinch to target_pos and tool0 to R_TOOL0_DES.

    Returns (arm_qpos, pos_err_m, ori_err_rad).
    """
    d.qpos[:6] = q_init.copy()
    d.qpos[6:14] = gripper_qpos.copy()
    d.ctrl[:6] = q_init.copy()
    d.ctrl[6] = 255.0
    mujoco.mj_forward(m, d)

    for i in range(max_iter):
        tip = d.site_xpos[tip_site_id].copy()
        perr = target_pos - tip
        oerr = orientation_error(d, ori_site_id)
        pdist = np.linalg.norm(perr)
        odist = np.linalg.norm(oerr)
        if pdist < pos_tol and odist < ori_tol:
            print(f"  IK converged at iter {i}, pos_err={pdist:.6f}m ori_err={odist:.6f}")
            break

        pstep = perr * gain
        pn = np.linalg.norm(pstep)
        if pn > max_step:
            pstep *= max_step / pn
        ostep = np.clip(oerr * gain, -0.1, 0.1)

        J = get_jacobian6(m, d, tip_site_id, ori_site_id)
        dq = damped_pinv(J, damping) @ np.concatenate([pstep, ostep])
        d.qpos[:6] += dq
        d.qpos[6:14] = gripper_qpos.copy()
        d.ctrl[:6] = d.qpos[:6].copy()
        d.ctrl[6] = 255.0
        mujoco.mj_forward(m, d)

    tip_final = d.site_xpos[tip_site_id].copy()
    pos_err = np.linalg.norm(target_pos - tip_final)
    ori_err = np.linalg.norm(orientation_error(d, ori_site_id))
    return d.qpos[:6].copy(), pos_err, ori_err


def compute_gravity_ctrl(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    q_arm: np.ndarray,
    gripper_qpos: np.ndarray,
) -> np.ndarray:
    """Compute arm ctrl values that hold q_arm under gravity (analytical).

    For UR5e PD actuators: force = gain*ctrl + bias1*qpos (at steady state).
    To counteract gravity: ctrl = (grav_torque - bias1*qpos) / gain.
    """
    mujoco.mj_resetData(m, d)
    d.qpos[:6] = q_arm.copy()
    d.qpos[6:14] = gripper_qpos.copy()
    d.qvel[:] = 0
    d.qacc[:] = 0
    mujoco.mj_forward(m, d)

    grav_torque = d.qfrc_bias[:6].copy()
    ctrl_arm = np.zeros(6)
    for i in range(6):
        gain_i = m.actuator_gainprm[i, 0]
        bias1_i = m.actuator_biasprm[i, 1]
        ctrl_arm[i] = (grav_torque[i] - bias1_i * q_arm[i]) / gain_i
    return ctrl_arm


def main() -> None:
    m = mujoco.MjModel.from_xml_path(paths.scene_path())
    d = mujoco.MjData(m)

    tip_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "gripper_pinch")
    ori_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
    base_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")

    print(f"Model nq={m.nq}, nv={m.nv}, nu={m.nu}")
    print("Base body pos:", m.body_pos[base_id])

    # Step 1: Close gripper via simulation to get settled joint values
    print("\nClosing gripper via simulation...")
    gripper_qpos = close_gripper_sim(m, d)

    # Target: pinch retracted so the closed pad front stops ~7 mm clear of the
    # slider's -y face. gripper_pinch sits PINCH_TO_PAD_FRONT behind the pad
    # front, so pinch target = face - CLEARANCE - PINCH_TO_PAD_FRONT.
    # (face 0.470 - 0.007 - 0.011 = 0.452). This keeps the gravity settle in
    # run_stage1 from ramming the slider before the approach phase.
    slider_y = 0.5
    slider_half_b = 0.03
    slider_face_y = slider_y - slider_half_b
    PINCH_TO_PAD_FRONT = 0.011
    CLEARANCE = 0.007
    target_tip = np.array([0.0, slider_face_y - CLEARANCE - PINCH_TO_PAD_FRONT, 0.342])
    print(f"\nTarget tip position: {target_tip}")

    # Step 2: Multi-start IK. The first seed is the known vertical-pusher config
    # (recovers the tool-down branch reliably); the rest broaden the basin.
    candidates = [
        np.array([1.1803, -1.6653, 2.4917, -2.3972, -1.5708, -0.3905]),
        np.array([0.0, -1.0, 1.5, -2.0, -1.5708, 0.0]),
        np.array([0.5, -1.2, 1.8, -2.1, -1.5708, 0.0]),
        np.array([-0.5, -1.0, 1.5, -2.0, -1.5708, 0.0]),
        np.array([0.0, -0.8, 1.0, -1.8, -1.5708, 0.0]),
        np.array([0.0, -1.5, 2.0, -2.0, -1.5708, 0.0]),
        np.array([1.0, -1.5, 2.0, -2.0, -1.5708, 0.0]),
        np.array([-1.0, -1.5, 2.0, -2.0, -1.5708, 0.0]),
        np.array([0.3, -1.8, 2.5, -2.2, -1.5708, 0.0]),
        np.array([-0.3, -1.8, 2.5, -2.2, -1.5708, 0.0]),
        np.array([0.0, -2.0, 2.5, -2.0, -1.5708, 0.0]),
        np.array([0.0, -2.2, 2.8, -2.2, -1.5708, 0.0]),
        np.array([0.0, -2.5, 3.0, -2.0, -1.5708, 0.0]),
        np.array([0.3, -2.0, 2.5, -2.0, -1.5708, 0.0]),
        np.array([-0.3, -2.0, 2.5, -2.0, -1.5708, 0.0]),
        np.array([0.0, -1.8, 2.0, -1.8, -1.5708, 0.0]),
        np.array([0.0, -2.0, 2.8, -2.4, -1.5708, 0.0]),
    ]

    best_q = None
    best_score = np.inf

    for i, q0 in enumerate(candidates):
        mujoco.mj_resetData(m, d)
        q, perr, oerr = solve_ik(m, d, tip_site_id, ori_site_id, target_tip, q0, gripper_qpos)
        # Score weights orientation (rad) and position (m) together so a candidate
        # that nails the target but tips the tool over is not selected.
        score = perr + 0.1 * oerr
        print(f"  Candidate {i}: pos_err={perr:.6f}m ori_err={oerr:.6f}rad qpos={np.round(q, 4)}")
        if score < best_score:
            best_score = score
            best_q = q.copy()

    # Re-evaluate the winner's individual errors for reporting.
    mujoco.mj_resetData(m, d)
    d.qpos[:6] = best_q
    d.qpos[6:14] = gripper_qpos
    d.ctrl[:6] = best_q
    d.ctrl[6] = 255.0
    mujoco.mj_forward(m, d)
    tip_fk = d.site_xpos[tip_site_id].copy()
    best_perr = np.linalg.norm(target_tip - tip_fk)
    best_oerr = np.linalg.norm(orientation_error(d, ori_site_id))
    if best_perr > 0.002 or best_oerr > 0.01:
        print(f"\nWARNING: best pos_err={best_perr:.4f}m ori_err={best_oerr:.4f}rad")
    else:
        print(f"\nBest pos_err={best_perr:.6f}m ori_err={best_oerr:.6f}rad")
    print(f"Best qpos (IK): {np.round(best_q, 4)}")
    print(f"FK verification - tip pos: {tip_fk}")

    # Step 3: Analytical gravity compensation for ctrl
    ctrl_arm = compute_gravity_ctrl(m, d, best_q, gripper_qpos)
    print(f"\nGravity-compensated ctrl: {np.round(ctrl_arm, 6)}")

    # Step 4: Verify gravity sag magnitude
    print("\nGravity sag check (1s settle from IK qpos)...")
    mujoco.mj_resetData(m, d)
    d.qpos[:6] = best_q.copy()
    d.qpos[6:14] = gripper_qpos.copy()
    d.ctrl[:6] = ctrl_arm.copy()
    d.ctrl[6] = 255.0
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)
    tip_1s = d.site_xpos[tip_site_id].copy()
    sag_err = np.linalg.norm(target_tip - tip_1s)
    print(f"  Tip after 1s: {tip_1s}")
    print(f"  Sag from target: {sag_err * 1000:.1f}mm (PD steady-state droop, handled by MPC)")

    # Output keyframe strings
    # qpos = IK solution (FK gives exact target position at t=0)
    # ctrl = gravity-compensated setpoint (minimizes initial transient)
    slider_qpos = f"0 {slider_y} 0.342 1 0 0 0"
    arm_qpos_str = " ".join(f"{v:.4f}" for v in best_q)
    gripper_qpos_str = " ".join(f"{v:.6f}" for v in gripper_qpos)
    ctrl_arm_str = " ".join(f"{v:.4f}" for v in ctrl_arm)

    print(f"\n{'=' * 60}")
    print("KEYFRAME VALUES:")
    print(f"{'=' * 60}")
    print(f'qpos="{arm_qpos_str}')
    print(f"       {gripper_qpos_str}")
    print(f'       {slider_qpos}"')
    print(f'ctrl="{ctrl_arm_str} 255"')
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
