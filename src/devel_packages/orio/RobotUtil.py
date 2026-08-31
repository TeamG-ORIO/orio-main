import os as _os
import sys as _sys

_CORE_DIR = _os.path.abspath(_os.path.join(_os.path.dirname(__file__), "..", "orio_core"))
if _CORE_DIR not in _sys.path:
    _sys.path.insert(0, _CORE_DIR)

from orio_core import robot_util as _robot_util  # noqa: E402

# Re-export every public name so `rt.<name>` resolves exactly as before.
globals().update({_k: _v for _k, _v in vars(_robot_util).items()
                  if not _k.startswith("__")})
