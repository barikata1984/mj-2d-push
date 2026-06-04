"""Pusher-slider manipulation: MuJoCo simulation, MPC control, and analysis.

A UR5e + Robotiq 2F-85 vertical pusher executes a quasi-static 2D push of a
slider on a work surface, driven by a Hogan-2016 Family-of-Modes MPC.

Subpackages:
  controllers/  pusher-slider controllers (MPC) + a small registry
  sim/          MuJoCo run loop and keyframe IK generation
  analytical/   Stage-0 analytical (limit-surface) pusher-slider simulator
  viz/          grid-video renderer and result plots
  config        tyro-friendly nested run configuration
  io            trial-directory, config dump, time-series logging
  kinematics    shared Jacobian / orientation IK helpers
  paths         repo-relative scene / asset / results resolution
"""

from __future__ import annotations
