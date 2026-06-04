"""Quick verification of stage1_scene.xml.

Loads the model, resets to the 'ready' keyframe, steps the simulation,
and prints key positions to verify scene integrity.

Requires:
  - /workspace/assets/ directory with UR5e + Robotiq mesh symlinks
"""

import mujoco
import numpy as np


def main():
    # ---- Load ----
    model_path = "stage1_scene.xml"
    m = mujoco.MjModel.from_xml_path(model_path)
    d = mujoco.MjData(m)
    print(f"Loaded {model_path}")
    print(f"  nq={m.nq}  nv={m.nv}  nu={m.nu}  nbody={m.nbody}  ngeom={m.ngeom}")

    # ---- Site / body IDs ----
    eef_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
    tip_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "gripper_pinch")
    slider_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "slider_center")

    # ---- Reset to ready keyframe ----
    key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "ready")
    mujoco.mj_resetDataKeyframe(m, d, key_id)
    mujoco.mj_forward(m, d)

    eef_pos = d.site_xpos[eef_id].copy()
    tip_pos = d.site_xpos[tip_id].copy()
    slider_pos = d.site_xpos[slider_id].copy()

    print("\n=== After keyframe reset (t=0) ===")
    print(f"  EEF (attachment_site) : {eef_pos}")
    print(f"  Gripper pinch            : {tip_pos}")
    print(f"  Slider center         : {slider_pos}")
    print(f"  Contacts              : {d.ncon}")

    # ---- Step 1 second ----
    n_steps = int(1.0 / m.opt.timestep)
    for _ in range(n_steps):
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)

    eef_1 = d.site_xpos[eef_id].copy()
    tip_1 = d.site_xpos[tip_id].copy()
    slider_1 = d.site_xpos[slider_id].copy()

    print(f"\n=== After 1 s ({n_steps} steps) ===")
    print(f"  EEF (attachment_site) : {eef_1}")
    print(f"  Gripper pinch            : {tip_1}")
    print(f"  Slider center         : {slider_1}")
    print(f"  Contacts              : {d.ncon}")

    # ---- Stability checks ----
    eef_drift = np.linalg.norm(eef_1 - eef_pos)
    slider_drift = np.linalg.norm(slider_1 - slider_pos)
    tip_drift = np.linalg.norm(tip_1 - tip_pos)

    print(f"\n=== Drift over 1 s ===")
    print(f"  EEF drift    : {eef_drift:.6f} m")
    print(f"  Tip drift    : {tip_drift:.6f} m")
    print(f"  Slider drift : {slider_drift:.6f} m")

    # ---- Step 9 more seconds (total 10 s) ----
    for _ in range(9 * n_steps):
        mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d)

    eef_10 = d.site_xpos[eef_id].copy()
    tip_10 = d.site_xpos[tip_id].copy()
    slider_10 = d.site_xpos[slider_id].copy()

    print(f"\n=== After 10 s ===")
    print(f"  EEF (attachment_site) : {eef_10}")
    print(f"  Gripper pinch            : {tip_10}")
    print(f"  Slider center         : {slider_10}")
    print(
        f"  Slider z              : {slider_10[2]:.5f} (table at 0.3, expected ~0.315)"
    )

    eef_drift_10 = np.linalg.norm(eef_10 - eef_pos)
    slider_drift_10 = np.linalg.norm(slider_10 - slider_pos)

    print(f"\n=== Drift over 10 s ===")
    print(f"  EEF drift    : {eef_drift_10:.6f} m")
    print(f"  Slider drift : {slider_drift_10:.6f} m")

    # ---- Joint error ----
    target_qpos = np.array([-2.3034, -2.5143, 2.4770, -1.5301, -1.5708, 0.0])
    joint_error = d.qpos[:6] - target_qpos
    print(f"\n=== Joint errors at t=10 s ===")
    names = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3",
    ]
    for name, err in zip(names, joint_error):
        print(f"  {name:15s}: {err:+.6f} rad ({np.degrees(err):+.4f} deg)")

    # ---- Geometry sanity ----
    print("\n=== Geometry check ===")
    gap_y = slider_pos[1] - 0.03 - (tip_pos[1] + 0.005)
    print(f"  Gripper-slider gap (y): {gap_y * 1000:.2f} mm (positive = no contact)")
    print(
        f"  Gripper pinch z in slider range [{slider_pos[2] - 0.015:.4f}, {slider_pos[2] + 0.015:.4f}]: "
        f"{'OK' if slider_pos[2] - 0.015 <= tip_pos[2] <= slider_pos[2] + 0.015 else 'FAIL'}"
    )

    # ---- Summary ----
    ok = eef_drift_10 < 0.005 and slider_drift_10 < 0.005
    print(f"\n{'PASS' if ok else 'FAIL'}: Scene is {'stable' if ok else 'unstable'}")


if __name__ == "__main__":
    main()
