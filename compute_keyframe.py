"""Compute a ready keyframe for the UR5e + Robotiq 2F-85 gripper.

Uses damped-pseudoinverse IK to place the gripper pinch site just behind
the slider's -y face, at the correct height on the table surface.
The gripper is physically closed via simulation before IK so that
equality constraints settle the 4-bar linkage.

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


def damped_pinv(J: np.ndarray, damping: float = 1e-3) -> np.ndarray:
    JJT = J @ J.T
    return J.T @ np.linalg.inv(JJT + damping**2 * np.eye(JJT.shape[0]))


def get_jacobian(m: mujoco.MjModel, d: mujoco.MjData, site_id: int) -> np.ndarray:
    jacp = np.zeros((3, m.nv))
    mujoco.mj_jacSite(m, d, jacp, None, site_id)
    return jacp[:, :6]  # Only arm joint columns


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
    target_pos: np.ndarray,
    q_init: np.ndarray,
    gripper_qpos: np.ndarray,
    max_iter: int = 2000,
    tol: float = 5e-4,
    gain: float = 1.0,
    max_step: float = 0.05,
    damping: float = 1e-3,
) -> tuple[np.ndarray, float]:
    d.qpos[:6] = q_init.copy()
    d.qpos[6:14] = gripper_qpos.copy()
    d.ctrl[:6] = q_init.copy()
    d.ctrl[6] = 255.0
    mujoco.mj_forward(m, d)

    for i in range(max_iter):
        tip = d.site_xpos[tip_site_id].copy()
        err = target_pos - tip
        dist = np.linalg.norm(err)
        if dist < tol:
            print(f"  IK converged at iter {i}, error={dist:.6f}m")
            break

        step = err * gain
        step_norm = np.linalg.norm(step)
        if step_norm > max_step:
            step *= max_step / step_norm

        J = get_jacobian(m, d, tip_site_id)
        dq = damped_pinv(J, damping) @ step
        d.qpos[:6] += dq
        d.qpos[6:14] = gripper_qpos.copy()
        d.ctrl[:6] = d.qpos[:6].copy()
        d.ctrl[6] = 255.0
        mujoco.mj_forward(m, d)

    tip_final = d.site_xpos[tip_site_id].copy()
    final_err = np.linalg.norm(target_pos - tip_final)
    return d.qpos[:6].copy(), final_err


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
    m = mujoco.MjModel.from_xml_path("/workspace/stage1_scene.xml")
    d = mujoco.MjData(m)

    tip_site_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "gripper_pinch")
    base_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")

    print(f"Model nq={m.nq}, nv={m.nv}, nu={m.nu}")
    print("Base body pos:", m.body_pos[base_id])

    # Step 1: Close gripper via simulation to get settled joint values
    print("\nClosing gripper via simulation...")
    gripper_qpos = close_gripper_sim(m, d)

    # Target: pinch site just behind slider's -y face
    slider_y = 0.235
    slider_half_b = 0.03
    target_tip = np.array([0.0, slider_y - slider_half_b - 0.001, 0.315])
    print(f"\nTarget tip position: {target_tip}")

    # Step 2: Multi-start IK
    candidates = [
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
    best_err = np.inf

    for i, q0 in enumerate(candidates):
        mujoco.mj_resetData(m, d)
        q, err = solve_ik(m, d, tip_site_id, target_tip, q0, gripper_qpos)
        print(f"  Candidate {i}: err={err:.6f}m, qpos={np.round(q, 4)}")
        if err < best_err:
            best_err = err
            best_q = q.copy()

    if best_err > 0.002:
        print(f"\nWARNING: Best IK error is {best_err:.4f}m, may need manual tuning")
    else:
        print(f"\nBest IK error: {best_err:.6f}m")
    print(f"Best qpos (IK): {np.round(best_q, 4)}")

    # FK verification
    mujoco.mj_resetData(m, d)
    d.qpos[:6] = best_q
    d.qpos[6:14] = gripper_qpos
    d.ctrl[:6] = best_q
    d.ctrl[6] = 255.0
    mujoco.mj_forward(m, d)
    tip_fk = d.site_xpos[tip_site_id].copy()
    print(f"FK verification - tip pos: {tip_fk}")
    print(f"FK verification - error: {np.linalg.norm(target_tip - tip_fk):.6f}m")

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
    print(
        f"  Sag from target: {sag_err * 1000:.1f}mm (PD steady-state droop, handled by MPC)"
    )

    # Output keyframe strings
    # qpos = IK solution (FK gives exact target position at t=0)
    # ctrl = gravity-compensated setpoint (minimizes initial transient)
    slider_qpos = f"0 {slider_y} 0.315 1 0 0 0"
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
