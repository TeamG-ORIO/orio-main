"""Tests for orio_core.planning.make_ik_seed (injected-RNG, deterministic)."""
import numpy as np

from orio_core import planning


def _bounds(n, lo=-2.0, hi=2.0):
    return [(lo, hi)] * n


def test_attempt0_is_zero_seed_with_fixed_joint(py_rng):
    seed = planning.make_ik_seed(chain_length=9, attempt=0, rng=py_rng,
                                 bounds=_bounds(9))
    assert seed[4] == -1.5                       # default fixed joint pinned down
    assert all(seed[i] == 0.0 for i in range(9) if i != 4)


def test_attempt0_does_not_consume_rng(py_rng):
    import random
    a = planning.make_ik_seed(9, 0, random.Random(0), _bounds(9))
    b = planning.make_ik_seed(9, 0, random.Random(999), _bounds(9))
    assert a == b                                # attempt 0 never touches the rng


def test_retry_randomises_non_fixed_and_respects_bounds(py_rng):
    seed = planning.make_ik_seed(9, attempt=1, rng=py_rng, bounds=_bounds(9, -0.3, 0.3))
    assert seed[4] == -1.5                       # fixed joint still pinned
    for i in range(9):
        if i == 4:
            continue
        assert -0.3 <= seed[i] <= 0.3


def test_deterministic_given_seeded_rng():
    import random
    s1 = planning.make_ik_seed(9, 1, random.Random(42), _bounds(9))
    s2 = planning.make_ik_seed(9, 1, random.Random(42), _bounds(9))
    assert s1 == s2                              # same seed -> identical output


def test_works_with_numpy_generator(rng):
    seed = planning.make_ik_seed(9, 1, rng, _bounds(9))
    assert len(seed) == 9
    assert seed[4] == -1.5


def test_custom_fixed_mapping():
    import random
    seed = planning.make_ik_seed(6, attempt=1, rng=random.Random(0),
                                 bounds=_bounds(6), fixed={0: 1.23, 5: -0.5})
    assert seed[0] == 1.23
    assert seed[5] == -0.5
