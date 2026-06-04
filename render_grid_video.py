"""Render a trial's push trajectory as a polished 2x2 grid video.

Standard experiment-video format for this project: replays the joint/slider
trajectory stored in ``<trial>/data.npz`` and renders four synchronized panes
(overview / top / front / side) with glare suppressed (matte materials, dimmed
lights, planar reflection off) and the goal marker visible. The overview pane
carries a time / slider-y overlay.

Usage:
    pixi run python render_grid_video.py <trial_dir> [scene.xml]

Expects ``data.npz`` to contain ``joint_pos`` (N x 6 arm angles) and the slider
pose series ``slider_x``, ``slider_y``, ``slider_quat`` (N x 4, w x y z), as
produced by run_stage1.py. Output: ``<trial_dir>/result_grid.mp4``.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

SCENE_DEFAULT = "/workspace/stage1_scene.xml"
PANE_W, PANE_H = 480, 360
FPS = 30
SLIDER_Z = 0.342  # slider rests on the work surface; z is not logged

# (label, azimuth, elevation, distance, lookat) — tuned to the Stage-1 workspace.
CAMERAS = [
    ("overview", 130, -22, 1.90, (0.0, 0.60, 0.34)),
    ("top", 90, -90, 1.55, (0.0, 0.62, 0.33)),
    ("front", 270, -7, 1.50, (0.0, 0.62, 0.40)),
    ("side", 0, -7, 1.62, (0.0, 0.55, 0.40)),
]


def apply_antiglare(m: mujoco.MjModel) -> None:
    """Matte materials + dim lights (render-only; physics and XML unchanged)."""
    m.mat_reflectance[:] = 0.0
    m.mat_specular[:] = 0.0
    m.vis.headlight.diffuse[:] = 0.35
    m.vis.headlight.specular[:] = 0.0
    m.vis.headlight.ambient[:] = 0.4
    for i in range(m.nlight):
        m.light_diffuse[i] = [0.35, 0.35, 0.35]
        m.light_specular[i] = [0.0, 0.0, 0.0]


def render_grid_video(trial_dir: Path, scene: str = SCENE_DEFAULT) -> Path:
    m = mujoco.MjModel.from_xml_path(scene)
    apply_antiglare(m)
    d = mujoco.MjData(m)

    key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "ready")
    grip_closed = m.key_qpos[key][6:14].copy()

    z = np.load(trial_dir / "data.npz")
    jp, sx, sy, squat, t = (
        z["joint_pos"],
        z["slider_x"],
        z["slider_y"],
        z["slider_quat"],
        z["time"],
    )
    n = len(jp)
    print(f"replaying {n} frames from {trial_dir}")

    frames_dir = Path("/tmp/grid_video_frames")
    frames_dir.mkdir(exist_ok=True)
    for old in frames_dir.glob("f_*.png"):
        old.unlink()

    renderer = mujoco.Renderer(m, height=PANE_H, width=PANE_W)
    try:
        font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22
        )
    except OSError:
        font = ImageFont.load_default()

    def pane(az: float, el: float, dist: float, lookat: tuple) -> Image.Image:
        cam = mujoco.MjvCamera()
        cam.lookat[:] = lookat
        cam.distance, cam.azimuth, cam.elevation = dist, az, el
        renderer.update_scene(d, camera=cam)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        return Image.fromarray(renderer.render())

    for i in range(n):
        d.qpos[:6] = jp[i]
        d.qpos[6:14] = grip_closed
        d.qpos[14:17] = [sx[i], sy[i], SLIDER_Z]
        d.qpos[17:21] = squat[i]
        mujoco.mj_forward(m, d)

        grid = Image.new("RGB", (2 * PANE_W, 2 * PANE_H), (20, 20, 20))
        for k, (name, az, el, dist, la) in enumerate(CAMERAS):
            img = pane(az, el, dist, la)
            dr = ImageDraw.Draw(img)
            dr.rectangle([0, 0, 120, 30], fill=(0, 0, 0))
            dr.text((6, 4), name, fill=(255, 255, 255), font=font)
            if name == "overview":
                dr.text(
                    (6, PANE_H - 28),
                    f"t={t[i] - t[0]:4.1f}s  y={sy[i]:.3f}",
                    fill=(255, 255, 0),
                    font=font,
                )
            grid.paste(img, ((k % 2) * PANE_W, (k // 2) * PANE_H))
        grid.save(frames_dir / f"f_{i:05d}.png")

    out = trial_dir / "result_grid.mp4"
    cmd = [
        "ffmpeg",
        "-y",
        "-framerate",
        str(FPS),
        "-i",
        str(frames_dir / "f_%05d.png"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-crf",
        "18",
        "-preset",
        "fast",
        str(out),
    ]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f"ffmpeg failed: {res.stderr[-400:]}")
    print(f"saved: {out} ({out.stat().st_size / 1024:.0f} KB)")
    return out


if __name__ == "__main__":
    trial = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    if trial is None or not (trial / "data.npz").exists():
        sys.exit("usage: pixi run python render_grid_video.py <trial_dir> [scene.xml]")
    render_grid_video(trial, sys.argv[2] if len(sys.argv) > 2 else SCENE_DEFAULT)
