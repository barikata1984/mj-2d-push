"""Controller registry.

Maps a config string (e.g. ``"mpc"``) to a controller class so the runner can
select a controller without ``if/else`` dispatch. Register new controllers with
the ``@register_controller("name")`` decorator; they then become available to
``make_controller`` and any CLI that exposes the controller name.
"""

from __future__ import annotations

from typing import Callable

from .base import Controller
from .mpc import PusherSliderMPC

_REGISTRY: dict[str, type] = {}


def register_controller(name: str) -> Callable[[type], type]:
    def deco(cls: type) -> type:
        _REGISTRY[name] = cls
        return cls

    return deco


def make_controller(name: str, **kwargs) -> Controller:
    """Instantiate a registered controller by name."""
    if name not in _REGISTRY:
        raise KeyError(f"unknown controller '{name}'; available: {sorted(_REGISTRY)}")
    return _REGISTRY[name](**kwargs)


def available_controllers() -> list[str]:
    return sorted(_REGISTRY)


# Built-in registrations. PusherSliderMPC predates the registry, so it is
# registered here rather than via the decorator (keeps mpc.py import-free).
_REGISTRY["mpc"] = PusherSliderMPC

__all__ = [
    "Controller",
    "PusherSliderMPC",
    "register_controller",
    "make_controller",
    "available_controllers",
]
