"""Pure planning helpers (IK seeding, etc.).

Honest extraction of the IK random-restart seed logic currently inlined in
``state_machine.py`` ``compute_pick_joints`` (lines ~291-299). The key change
that makes it testable: the random-number generator is *injected* as an argument
instead of the module-global ``random``. The node creates and seeds one RNG once
at start-up (the dishonest part) and passes it in; tests pass a seeded RNG for
determinism.
"""

DEFAULT_FIXED = {4: -1.5}  # joint[4] pinned so the gripper points down


def make_ik_seed(chain_length, attempt, rng, bounds, fixed=None):
    """Build an IK initial-guess vector.

    Args:
        chain_length: number of links in the ikpy chain (len(chain.links)).
        attempt: retry index. attempt 0 uses the plain zero+fixed seed; attempts
            > 0 randomise the non-fixed joints (random restart).
        rng: a random generator exposing ``uniform(low, high)`` — e.g.
            ``random.Random(seed)`` or ``numpy.random.default_rng(seed)``.
            Injected so the function is deterministic given a seeded rng.
        bounds: sequence of ``(lo, hi)`` per link (from ``chain.links[i].bounds``).
        fixed: mapping ``{index: value}`` of joints to pin. Defaults to
            ``{4: -1.5}`` (gripper down), matching the current behaviour.

    Returns:
        list[float] of length ``chain_length``.
    """
    if fixed is None:
        fixed = DEFAULT_FIXED
    seed = [0.0] * chain_length
    for idx, val in fixed.items():
        seed[idx] = val
    if attempt > 0:
        for i in range(chain_length):
            if i in fixed:
                continue
            lo, hi = bounds[i]
            seed[i] = rng.uniform(lo, hi)
    return seed
