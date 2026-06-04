"""Compute a ready keyframe for the UR5e with base on the table.

Uses damped-pseudoinverse IK to place the pusher tip just behind the slider's
-y face, at the correct height on the table surface.
"""

from __future__ import annotations

import mujoco
import numpy as np


def damped_pinv(J: np.ndarray, damping: float = 1e-3) -> np.ndarray:
  JJT = J @ J.T
  return J.T @ np.linalg.inv(JJT + damping**2 * np.eye(JJT.shape[0]))


def get_jacobian(
  m: mujoco.MjModel, d: mujoco.MjData, site_id: int
) -> np.ndarray:
  jacp = np.zeros((3, m.nv))
  mujoco.mj_jacSite(m, d, jacp, None, site_id)
  return jacp[:, :6]


def solve_ik(
  m: mujoco.MjModel,
  d: mujoco.MjData,
  tip_site_id: int,
  target_pos: np.ndarray,
  q_init: np.ndarray,
  max_iter: int = 2000,
  tol: float = 5e-4,
  gain: float = 1.0,
  max_step: float = 0.05,
  damping: float = 1e-3,
) -> tuple[np.ndarray, float]:
  d.qpos[:6] = q_init.copy()
  d.ctrl[:6] = q_init.copy()
  mujoco.mj_forward(m, d)

  for i in range(max_iter):
    tip = d.site_xpos[tip_site_id].copy()
    err = target_pos - tip
    dist = np.linalg.norm(err)
    if dist < tol:
      print(f'  IK converged at iter {i}, error={dist:.6f}m')
      break

    step = err * gain
    step_norm = np.linalg.norm(step)
    if step_norm > max_step:
      step *= max_step / step_norm

    J = get_jacobian(m, d, tip_site_id)
    dq = damped_pinv(J, damping) @ step
    d.qpos[:6] += dq
    d.ctrl[:6] = d.qpos[:6].copy()
    mujoco.mj_forward(m, d)

  tip_final = d.site_xpos[tip_site_id].copy()
  final_err = np.linalg.norm(target_pos - tip_final)
  return d.qpos[:6].copy(), final_err


def find_ctrl_with_gravity(
  m: mujoco.MjModel,
  d: mujoco.MjData,
  tip_site_id: int,
  target_pos: np.ndarray,
  q_target: np.ndarray,
  settle_time: float = 2.0,
  max_iter: int = 20,
  tol: float = 1e-3,
) -> np.ndarray:
  """Find ctrl values that produce the target tip position under gravity."""
  ctrl = q_target.copy()
  substeps = int(settle_time / m.opt.timestep)

  for iteration in range(max_iter):
    mujoco.mj_resetData(m, d)
    d.qpos[:6] = q_target.copy()
    d.ctrl[:6] = ctrl.copy()
    for _ in range(substeps):
      mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)

    tip = d.site_xpos[tip_site_id].copy()
    err = target_pos - tip
    dist = np.linalg.norm(err)

    if dist < tol:
      print(
        f'  Gravity compensation converged at iter {iteration}, error={dist:.6f}m'
      )
      break

    J = get_jacobian(m, d, tip_site_id)
    dq = damped_pinv(J, 1e-3) @ (err * 0.5)
    ctrl += dq

  return ctrl


def main():
  m = mujoco.MjModel.from_xml_path('/workspace/stage1_scene.xml')
  d = mujoco.MjData(m)

  tip_site_id = mujoco.mj_name2id(
    m, mujoco.mjtObj.mjOBJ_SITE, 'pusher_tip_site'
  )
  base_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'base')
  slider_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'slider')

  print('Base body pos:', m.body_pos[base_id])

  slider_y = 0.235
  slider_half_b = 0.03
  target_tip = np.array([0.0, slider_y - slider_half_b - 0.0005, 0.315])
  print(f'Target tip position: {target_tip}')

  # Try multiple starting configurations
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
  ]

  best_q = None
  best_err = np.inf

  for i, q0 in enumerate(candidates):
    mujoco.mj_resetData(m, d)
    q, err = solve_ik(m, d, tip_site_id, target_tip, q0)
    print(f'  Candidate {i}: err={err:.6f}m, qpos={np.round(q, 4)}')
    if err < best_err:
      best_err = err
      best_q = q.copy()

  if best_err > 0.005:
    print(
      f'\nWARNING: Best IK error is {best_err:.4f}m, may need manual tuning'
    )
  else:
    print(f'\nBest IK error: {best_err:.6f}m')

  print(f'Best qpos: {np.round(best_q, 4)}')

  # Verify tip position with FK
  mujoco.mj_resetData(m, d)
  d.qpos[:6] = best_q
  d.ctrl[:6] = best_q
  mujoco.mj_forward(m, d)
  tip_fk = d.site_xpos[tip_site_id].copy()
  print(f'FK verification - tip pos: {tip_fk}')
  print(f'FK verification - error: {np.linalg.norm(target_tip - tip_fk):.6f}m')

  # Compensate for gravity sag
  print('\nFinding gravity-compensated ctrl values...')
  ctrl = find_ctrl_with_gravity(m, d, tip_site_id, target_tip, best_q)
  print(f'Gravity-compensated ctrl: {np.round(ctrl, 4)}')

  # Verify with gravity
  mujoco.mj_resetData(m, d)
  d.qpos[:6] = best_q
  d.ctrl[:6] = ctrl
  for _ in range(int(2.0 / m.opt.timestep)):
    mujoco.mj_step(m, d)
  mujoco.mj_forward(m, d)
  tip_grav = d.site_xpos[tip_site_id].copy()
  print(f'After gravity settle - tip pos: {tip_grav}')
  print(
    f'After gravity settle - error: {np.linalg.norm(target_tip - tip_grav):.6f}m'
  )

  # Output keyframe strings
  slider_qpos = f'0 {slider_y} 0.315 1 0 0 0'
  qpos_str = ' '.join(f'{v:.4f}' for v in best_q)
  ctrl_str = ' '.join(f'{v:.4f}' for v in ctrl)
  print(f'\n{"=" * 60}')
  print('KEYFRAME VALUES:')
  print(f'{"=" * 60}')
  print(f'qpos="{qpos_str}')
  print(f'       {slider_qpos}"')
  print(f'ctrl="{ctrl_str}"')
  print(f'{"=" * 60}')


if __name__ == '__main__':
  main()
