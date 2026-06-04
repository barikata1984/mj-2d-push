"""CLI: run a closed-loop MPC push.

``tyro`` derives the CLI from the :class:`SimConfig` dataclass, so every nested
field is overridable, e.g.::

    pixi run python scripts/run_push.py --push.y-goal 0.85 --mpc.v-max 0.1
"""

from __future__ import annotations

import tyro

from pusher_slider.config import SimConfig
from pusher_slider.sim.runner import run
from pusher_slider.viz.plots import plot_results


def main() -> None:
    cfg = tyro.cli(SimConfig)
    log, trial_dir = run(cfg)
    plot_results(log, trial_dir, cfg)


if __name__ == "__main__":
    main()
