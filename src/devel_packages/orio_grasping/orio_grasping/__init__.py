"""Simulation environment for generating grasp-planning training data for the Orio workcell.

Import order matters: ``isaaclab`` can only be imported after the Isaac Sim app has been launched,
so the submodules that depend on it (``scene``, ``robot``, ``objects``, ``pile``, ``capture``,
``generator``) are imported explicitly by the scripts, after ``AppLauncher``. ``config`` and
``geometry`` are plain Python and can be imported anywhere.
"""

from .config import OrioCfg

__all__ = ["OrioCfg"]
