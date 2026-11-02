"""Process environment for the pip PyTorch (CUDA 13) in the ``orio-grasping`` env.

Isaac Sim's setup puts its own CUDA 12 libraries on ``LD_LIBRARY_PATH``, which makes PyTorch load
the wrong cuDNN and fail at the first convolution (``CUDNN_STATUS_SUBLIBRARY_LOADING_FAILED``).
The loader reads that variable only at process start, so a script calls
:func:`drop_isaac_libraries` first thing and is restarted without those entries.
"""

from __future__ import annotations

import os
import sys


def drop_isaac_libraries() -> None:
    """Re-run this process without Isaac Sim's entries on ``LD_LIBRARY_PATH``."""
    paths = os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)
    kept = [p for p in paths if p and "isaacsim" not in p and "cuda-12" not in p]
    if len(kept) != len([p for p in paths if p]):
        env = dict(os.environ, LD_LIBRARY_PATH=os.pathsep.join(kept))
        os.execve(sys.executable, [sys.executable, *sys.argv], env)
