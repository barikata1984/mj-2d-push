"""Trial-directory creation, config dump, and time-series logging."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, is_dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

from . import paths


def create_trial_dir(base: Path | str | None = None) -> Path:
    """Make a fresh timestamped trial directory ``results/<UTC-stamp>/pics``."""
    base = Path(base) if base is not None else paths.results_dir()
    ts = datetime.now(tz=timezone.utc).strftime("%Y%m%d_%H%M%S")
    trial_dir = base / ts
    (trial_dir / "pics").mkdir(parents=True, exist_ok=True)
    return trial_dir


def dump_config(cfg: object, trial_dir: Path) -> None:
    """Write the resolved config to ``<trial>/config.json`` (reproducibility anchor)."""
    data = asdict(cfg) if is_dataclass(cfg) else cfg
    with open(trial_dir / "config.json", "w") as f:
        json.dump(data, f, indent=2, default=str)


@dataclass
class Log:
    """Per-MPC-step time series, saved to ``data.npz`` for post-hoc analysis."""

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
            slider_quat=np.array(self.slider_quat) if self.slider_quat else np.empty((0, 4)),
            pusher_x=np.array(self.pusher_x),
            pusher_y=np.array(self.pusher_y),
            pusher_z=np.array(self.pusher_z),
            pusher_body_px=np.array(self.pusher_body_px),
            pusher_body_py=np.array(self.pusher_body_py),
            vn=np.array(self.vn),
            vt=np.array(self.vt),
            target_y=np.array(self.target_y),
            joint_pos=np.array(self.joint_pos) if self.joint_pos else np.empty((0, 6)),
            joint_ctrl=np.array(self.joint_ctrl) if self.joint_ctrl else np.empty((0, 6)),
            contact_count=np.array(self.contact_count),
            contact_forces=np.array(self.contact_forces)
            if self.contact_forces
            else np.empty((0, 3)),
        )
