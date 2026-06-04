"""CLI: render the polished 2x2 grid video for a trial directory.

pixi run python scripts/render_video.py <trial_dir> [scene.xml]
"""

from __future__ import annotations

import sys
from pathlib import Path

from pusher_slider import paths
from pusher_slider.viz.grid_video import render_grid_video


def main() -> None:
    if len(sys.argv) < 2 or not (Path(sys.argv[1]) / "data.npz").exists():
        sys.exit("usage: python scripts/render_video.py <trial_dir> [scene.xml]")
    trial = Path(sys.argv[1])
    scene = sys.argv[2] if len(sys.argv) > 2 else paths.scene_path()
    render_grid_video(trial, scene)


if __name__ == "__main__":
    main()
