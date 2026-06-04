"""CLI: recompute the 'ready' keyframe via 6-DOF tool-down IK.

Prints the keyframe qpos/ctrl strings to paste into scenes/stage1_scene.xml.
"""

from __future__ import annotations

from pusher_slider.sim.keyframe import main

if __name__ == "__main__":
    main()
