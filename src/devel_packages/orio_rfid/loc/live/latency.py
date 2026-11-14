#!/usr/bin/env python3
"""
latency.py - how far the positions on the map are behind the reads, from a run's own logs.

    python3 latency.py <run folder with engine_rounds.csv> [--offset S] [--arrivals <receiver's run folder>]

engine_rounds.csv (written by engine.py) has, per round, when the positions were published and the robot-clock time of
the newest read they are based on. A read made at time r first shows in the positions published at the end of the next
round that includes it, so its delay is (that publish time - r). Printed: the delay of the newest read of each round
(best case), of the oldest read of each round (worst case) and of a read in the middle, plus the round length and how
much of each round the maths took. --offset is (this machine's clock - the robot's clock) in seconds, needed when the
localiser ran on the PC; --arrivals adds how long the reads took to reach the PC.
The window adds its own polling on top: on average 0.65 s, at most 1.3 s (it asks the receiver once a second and
redraws four times a second).
"""
import csv, os, sys
import numpy as np
a = [x for x in sys.argv[1:] if not x.startswith('--')]; off = float(sys.argv[sys.argv.index('--offset') + 1]) if '--offset' in sys.argv else 0.0
R = [r for r in csv.DictReader(open(os.path.join(a[0], 'engine_rounds.csv'))) if float(r['newest_read_t']) > 0]
tp = np.array([float(r['t_published']) for r in R]) - off; nr = np.array([float(r['newest_read_t']) for r in R]); rows = np.array([int(r['rows']) for r in R])
live = rows > 0                                   # rounds that took reads in (the last rounds only re-estimate)
tp, nr, R = tp[live], nr[live], [r for r, k in zip(R, live) if k]
newest = tp - nr; oldest = tp[1:] - nr[:-1]; mid = tp[1:] - (nr[1:] + nr[:-1]) / 2; period = np.diff(tp)
work = np.array([float(r['evidence_s']) + float(r['estimate_s']) for r in R]); left = np.array([int(r['objects_left_dirty']) for r in R]); wait = np.array([int(r['waiting']) for r in R])
q = lambda v, p: float(np.quantile(v, p))
print('%d rounds over %.0f s of run' % (len(R), nr[-1] - nr[0]))
print('  delay of a read until it is in the published positions:  typical %.1f s  (newest read of a round %.1f s, oldest %.1f s; worst round %.1f s)' % (
    q(mid, .5), q(newest, .5), q(oldest, .5), oldest.max()))
print('  delay in the first / middle / last third of the run:     %.1f / %.1f / %.1f s' % tuple(q(mid[k], .5) for k in np.array_split(np.arange(len(mid)), 3)))
print('  a round every %.1f s (median), of which maths %.1f s (evidence %.1f + estimates %.1f); longest round %.1f s' % (
    q(period, .5), q(work, .5), q([float(r['evidence_s']) for r in R], .5), q([float(r['estimate_s']) for r in R], .5), period.max()))
print('  objects read but not re-estimated in the same round: median %d, worst %d of %s;  reads waiting to be taken in: worst %d' % (q(left, .5), left.max(), R[-1]['objects_total'], wait.max()))
if 'tags_updated' in R[0]:              # per tag: from its oldest read still waiting to the moment its new position is published
    k = np.array([int(r['tags_updated']) for r in R]) > 0; tpk = tp[k]
    med = tpk - np.array([float(r['oldest_waiting_read_median_t']) for r in R])[k]; worst = tpk - np.array([float(r['oldest_waiting_read_min_t']) for r in R])[k]
    print('  PER ITEM, from a read to the item\'s new position being published:  typical %.1f s, 90 %% of rounds within %.1f s; the slowest item of a round: typical %.1f s, worst %.1f s' % (
        q(med, .5), q(med, .9), q(worst, .5), worst.max()))
    c = {x[5:]: float(R[-1][x]) for x in R[-1] if x.startswith('cost_')}; tot = sum(c.values()) or 1
    print('  where the maths time went: ' + ', '.join('%s %.0f s (%.0f %%)' % (n, v, 100 * v / tot) for n, v in sorted(c.items(), key=lambda x: -x[1]) if v > 0.05) + ';  total %.0f s for %.0f s of run = %.2f x real time' % (tot, nr[-1] - nr[0], tot / (nr[-1] - nr[0])))
if '--arrivals' in sys.argv:
    A = np.array([[float(x) for x in l.split(',')] for l in open(os.path.join(sys.argv[sys.argv.index('--arrivals') + 1], 'arrivals.csv'))])
    d = A[:, 0] - off - A[:, 1]; print('  reads reach the PC %.2f s after they were made (median; 95 %% within %.2f s), in batches of %d' % (q(d, .5), q(d, .95), q(A[:, 2], .5)))
