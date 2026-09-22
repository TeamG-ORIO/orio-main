"""Shared pytest fixtures for orio_core.

The whole point of orio_core is determinism: functions that need randomness take
an injected rng, so tests seed it and get repeatable results.
"""
import random

import numpy as np
import pytest


@pytest.fixture
def rng():
    """A seeded numpy Generator (exposes .uniform(low, high))."""
    return np.random.default_rng(0)


@pytest.fixture
def py_rng():
    """A seeded stdlib random.Random (also exposes .uniform(low, high)) — this is
    what the state machine node will inject, since the legacy code used the
    stdlib `random` module."""
    return random.Random(0)
