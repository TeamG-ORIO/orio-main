"""Per-run statistics for full_sequence.py, shown on the dashboard.

Per item name:
  grasps, per attempt          bin picks where the cup held the item after lifting out
                               of the bin / every bin pick tried (each automatic
                               missed-pick retry counts)
  grasps, with error recovery  pick sequences that got the item / pick sequences: a
                               sequence is a first try and its automatic retries, and
                               fails only when a recovery window opens during it
  labelled                     items that reached the output box labelled with no human
                               help on the way / items finished (that, plus items that
                               failed). Items still in the cell are counted apart.

For the whole run: seconds per labelled item = active run time / items labelled, worked
out when an item is labelled and held until the next one is. Active time leaves out time
the robot waits on a person (see full_sequence._stats_counting).

An item fails once a recovery window opens for the arm handling it (an error or
INTERVENE), when the operator sets its zone's state by hand, or when it is lost.

Plain Python, no ROS: full_sequence.py calls the hooks from its state threads and the
status loop reads snapshot().
"""

import threading


class _Item:
    __slots__ = ('id', 'name', 'labelled', 'failed_why', 'extra')

    def __init__(self, id_, name, extra=False):
        self.id, self.name, self.extra = id_, name, extra
        self.labelled = False
        self.failed_why = None      # set once human help touched it: it can no longer pass


class RunStats:
    def __init__(self, emit=None):
        self._lock = threading.RLock()
        self._emit = emit or (lambda *a, **kw: None)
        self._names = {}            # name -> counters, in first-seen order
        self._ids = 0
        self._trying = None         # name of the bin pick in flight (cup on, not yet checked)
        self._seq = None            # name of the open pick sequence (a miss is being retried)
        self._held = None           # the item in arm 1's cup
        self._zone = {'A': None, 'B': None}
        self._active_s = 0.0
        self._last_tick = None
        self._labelled = 0
        self._per_item_s = None     # active time / items labelled, as of the last one

    # ── internals ────────────────────────────────────────────────────────────
    def _c(self, name):
        c = self._names.get(name)
        if c is None:
            c = self._names[name] = dict(tries=0, tries_ok=0, picks=0, picks_ok=0,
                                         items=0, ok=0, failed=0)
        return c

    def _new_item(self, name, extra=False):
        self._ids += 1
        self._c(name)['items'] += 1
        return _Item(self._ids, name, extra)

    def _finish(self, item, ok, why=None):
        c = self._c(item.name)
        c['ok' if ok else 'failed'] += 1
        if ok:
            self._labelled += 1
            self._per_item_s = self._active_s / self._labelled
        why = None if ok else (why or item.failed_why or "not labelled")
        self._emit('stats', 'item', id=item.id, item=item.name, ok=ok, why=why,
                   extra=item.extra)

    def _taint(self, item, why):
        if item is not None and item.failed_why is None:
            item.failed_why = why

    def _close_seq(self, ok):
        if self._seq is None:
            return
        c = self._c(self._seq)
        c['picks'] += 1
        c['picks_ok'] += int(ok)
        self._emit('stats', 'pick', item=self._seq, ok=ok)
        self._seq = None

    def _end_try(self, ok):
        name, self._trying = self._trying, None
        c = self._c(name)
        c['tries'] += 1
        c['tries_ok'] += int(ok)
        self._emit('stats', 'try', item=name, ok=ok)

    # ── bin picks (arm 1) ────────────────────────────────────────────────────
    def grasp_try(self, name):
        """Arm 1 is going down into the bin for `name`."""
        with self._lock:
            self._trying = name            # an unresolved earlier try (a stop) is dropped
            if self._seq is None:
                self._seq = name

    def grasp_held(self):
        """The cup held the item after lifting out of the bin."""
        with self._lock:
            if self._trying is None:
                return
            name = self._trying
            self._end_try(True)
            self._close_seq(True)
            if self._held is not None:     # never put down where we saw: lost
                self._finish(self._held, False, "lost")
            self._held = self._new_item(name)

    def grasp_missed(self):
        """The cup came up empty and arm 1 tries again on its own."""
        with self._lock:
            if self._trying is not None:
                self._end_try(False)

    # ── item flow ────────────────────────────────────────────────────────────
    def dropped(self, zone):
        """Arm 1 put its item in `zone`."""
        with self._lock:
            if self._zone[zone] is not None:
                self._finish(self._zone[zone], False, "lost")
            self._zone[zone], self._held = self._held, None

    def labelled(self, zone):
        with self._lock:
            if self._zone[zone] is not None:
                self._zone[zone].labelled = True

    def retrieved(self, zone):
        """Arm 1 picked the labelled item out of `zone`."""
        with self._lock:
            if self._held is not None:
                self._finish(self._held, False, "lost")
            self._held, self._zone[zone] = self._zone[zone], None

    def leftover(self, zone, fallback_name):
        """Another item seen in `zone` after arm 1's pick: a second item the bin pick
        lifted. It counts as picked, under the name of the item it came with."""
        with self._lock:
            if self._zone[zone] is None:
                name = self._held.name if self._held is not None else fallback_name
                self._zone[zone] = self._new_item(name, extra=True)

    def vanished(self, zone):
        """The item thought left in `zone` is not there (a false sighting): forget it."""
        with self._lock:
            item, self._zone[zone] = self._zone[zone], None
            if item is not None and item.extra:
                self._c(item.name)['items'] -= 1

    def placed(self):
        """Arm 1 released its item in the output box."""
        with self._lock:
            item, self._held = self._held, None
            if item is None:
                return
            ok = item.labelled and item.failed_why is None
            self._finish(item, ok)

    # ── people ───────────────────────────────────────────────────────────────
    def recovery(self, arm, failed_state, zones):
        """A recovery window opened for `arm` (error or INTERVENE) while it worked on
        `zones` (the zones it had claimed). Every item it was handling fails."""
        with self._lock:
            why = "%s recovery (%s)" % (arm, failed_state)
            if arm == 'arm1':
                if self._trying is not None:
                    self._end_try(False)
                if failed_state == 'FETCH_INPUT':
                    self._close_seq(False)
                self._taint(self._held, why)
            for z in zones:
                self._taint(self._zone.get(z), why)

    def zone_set(self, zone, old, new):
        """The operator set `zone`'s state by hand."""
        if old == new:
            return
        with self._lock:
            item = self._zone[zone]
            if new == 'EMPTY':
                if item is not None:
                    self._zone[zone] = None
                    self._finish(item, False, "taken out by hand")
                return
            if item is not None:
                self._taint(item, "zone %s set by hand" % zone)
                if new == 'READY_FOR_PICKUP':
                    item.labelled = True   # labelled by hand: still a failure
            elif self._held is not None and self._held.failed_why is not None:
                # most likely the item arm 1 dropped, put in the zone by hand
                self._zone[zone], self._held = self._held, None

    # ── time ─────────────────────────────────────────────────────────────────
    def tick(self, now, counting):
        """Called often (the status loop): adds the time since the last call to the
        active run time while `counting`."""
        with self._lock:
            if self._last_tick is not None and counting:
                self._active_s += max(0.0, now - self._last_tick)
            self._last_tick = now

    # ── read ─────────────────────────────────────────────────────────────────
    def snapshot(self):
        with self._lock:
            names = []
            for name, c in self._names.items():
                in_cell = c['items'] - c['ok'] - c['failed']
                names.append(dict(c, name=name, in_cell=max(0, in_cell)))
            return {'names': names, 'active_s': self._active_s, 'labelled': self._labelled,
                    'per_item_s': self._per_item_s}


def ratio(num, den):
    return "%d/%d (%s)" % (num, den, "%.0f%%" % (100.0 * num / den) if den else "n/a")


def summary_lines(snap):
    """The run's stats as log lines, for the end of the run."""
    lines = []
    for n in snap['names']:
        lines.append("%s: grasps per attempt %s, with error recovery %s, labelled %s%s" % (
            n['name'], ratio(n['tries_ok'], n['tries']), ratio(n['picks_ok'], n['picks']),
            ratio(n['ok'], n['ok'] + n['failed']),
            ", %d still in the cell" % n['in_cell'] if n['in_cell'] else ""))
    per = snap['per_item_s']
    lines.append("time per labelled item: %s (%d labelled; %.0f s active in all)" % (
        "%.1f s" % per if per is not None else "n/a", snap['labelled'], snap['active_s']))
    return lines
