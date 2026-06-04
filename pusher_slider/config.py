"""Run configuration as nested dataclasses (consumed by ``tyro`` for the CLI).

The scripts build a :class:`SimConfig` with ``tyro.cli(SimConfig)``, so every
field below is overridable on the command line, e.g.::

    python scripts/run_push.py --push.y-goal 0.85 --mpc.v-max 0.1 --render.camera top

The resolved config is dumped into each trial directory for reproducibility.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from . import paths


@dataclass
class PushConfig:
    """Push task definition (world frame; +y is the push direction)."""

    push_speed: float = 0.03
    # Base-frame (0, 0.4) -> (0, 0.8) maps to world y 0.5 -> 0.9 on the work surface.
    y_start: float = 0.5
    y_goal: float = 0.9
    max_sim_time: float = 30.0


@dataclass
class MPCConfig:
    """Hogan-2016 Family-of-Modes MPC parameters."""

    dt: float = 0.03
    horizon: int = 10
    v_max: float = 0.08
    contact_face: Literal["-y", "+y", "-x", "+x"] = "-y"
    slider_dims: tuple[float, float] = (0.08, 0.06)
    mass: float = 1.05
    mu_pusher: float = 0.3
    mu_ground: float = 0.35
    q_weights: tuple[float, float, float, float] = (30.0, 10.0, 15.0, 0.1)
    r_weights: tuple[float, float] = (0.1, 0.1)
    q_terminal_scale: float = 10.0


@dataclass
class RobotConfig:
    """Resolved-rate IK execution-layer gains."""

    damping: float = 1e-3
    ik_gain: float = 2.0
    ik_max_step: float = 0.02


@dataclass
class RenderConfig:
    """In-loop side-view rendering (the polished grid video is a separate step)."""

    width: int = 960
    height: int = 720
    camera: str = "side"
    fps: int = 30
    every_n_mpc: int = 1


@dataclass
class SimConfig:
    """Top-level configuration for a single push run."""

    scene: str = paths.DEFAULT_SCENE
    push: PushConfig = field(default_factory=PushConfig)
    mpc: MPCConfig = field(default_factory=MPCConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    render: RenderConfig = field(default_factory=RenderConfig)

    @property
    def scene_path(self) -> str:
        """Absolute path to the scene XML (resolved under ``scenes/``)."""
        return paths.scene_path(self.scene)

    @property
    def q_weights_arr(self) -> np.ndarray:
        return np.array(self.mpc.q_weights)

    @property
    def r_weights_arr(self) -> np.ndarray:
        return np.array(self.mpc.r_weights)
