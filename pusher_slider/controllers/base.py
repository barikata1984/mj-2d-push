"""Controller interface for the pusher-slider system.

A structural ``Protocol`` (not an ABC) so any object exposing ``compute_control``
is usable as a controller without forced inheritance.
"""

from __future__ import annotations

from typing import Protocol

import numpy as np


class Controller(Protocol):
    """Maps the current slider/pusher state to a contact-frame push velocity."""

    def compute_control(
        self,
        slider_pose: np.ndarray,
        pusher_pos_body: np.ndarray,
        target_pose: np.ndarray,
    ) -> tuple[float, float]:
        """Return (vn, vt): pusher velocity along the face normal / tangent."""
        ...
