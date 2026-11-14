#!/usr/bin/env python3
"""
read_item.py - hold ONE item to the antenna and get the item numbers of every tag on it.

An object can carry up to three tags (regular, on-metal, liquid). Unlike register_tags.py, which
wants a single clear winner, this lists every registered tag that is close to the strongest one,
checks the set against the groups in tag_registry.csv, and prints what to write on the item.

    python3 read_item.py                 # antenna 4, lowest power, 3 s per read
    python3 read_item.py --within 15     # accept tags up to 15 dB below the strongest (default 12)

Keys:  Enter = read   b = background scan (nothing in front)   q = quit
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import register_tags as rt  # noqa: E402


def item_no(name):
    return name.split()[-1] if name.startswith('Item ') else name


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ip', default='192.168.0.20')
    ap.add_argument('--antennas', type=int, nargs='+', default=[4])
    ap.add_argument('--power', type=float, default=10.0, help='dBm; 10 is the R700 minimum')
    ap.add_argument('--duration', type=float, default=3.0)
    ap.add_argument('--bg-duration', type=float, default=10.0)
    ap.add_argument('--within', type=float, default=12.0, help='dB below the strongest tag that still counts as "on this item"')
    ap.add_argument('--min-reads', type=int, default=5)
    ap.add_argument('--registry', default=rt.REGISTRY)
    args = ap.parse_args()

    registry = rt.load_registry(args.registry)
    print('%d tag(s) in registry' % len(registry))
    reader, col, used = rt.connect(args)
    background = {}
    print('\nHold ONE item against the antenna and press Enter. b = background scan, q = quit\n')
    try:
        while True:
            try:
                key = input('> ').strip().lower()
            except EOFError:
                break
            if key == 'q':
                break
            if key == 'b':
                print('background scan, %.0f s - keep items (and yourself) away ...' % args.bg_duration)
                rows = rt.summarize(col.window(args.bg_duration))
                background.update({r['epc']: r['rssi_mean'] for r in rows if r['rssi_mean'] is not None})
                print(rt.fmt_table(rows, registry, {}, n=10) if rows else '-> nothing seen. Good.')
                continue
            if key != '':
                print('unknown key %r' % key)
                continue

            print('reading %.0f s ...' % args.duration)
            rows = [r for r in rt.summarize(col.window(args.duration)) if r['rssi_mean'] is not None]
            if not rows:
                print('NONE: no tags read. Hold the item closer to the antenna face.')
                continue
            top = rows[0]['rssi_mean']
            # drop background tags that have not got louder than they were with nothing in front
            live = [r for r in rows if not (r['epc'] in background and r['rssi_mean'] - background[r['epc']] < 6)]
            if not live:
                print('only background tags seen (%s). Hold the item closer.' % ', '.join(r['epc'][-6:] for r in rows))
                continue
            top = live[0]['rssi_mean']
            on = [r for r in live if top - r['rssi_mean'] <= args.within and r['reads'] >= args.min_reads]
            # siblings of a grouped object count as on the item whenever they were read at all: a tag on the
            # far face of the same bottle can be 15 dB below the one pressed against the panel
            sib_objs = {registry[r['epc']]['object'] for r in on if r['epc'] in registry and registry[r['epc']]['object']}
            for r in live:
                if r not in on and r['epc'] in registry and registry[r['epc']]['object'] in sib_objs and r['reads'] >= 2:
                    on.append(r)
            weak = [r for r in live if r not in on]

            print(rt.fmt_table(rows, registry, background, n=6))
            names = []
            print('ON THIS ITEM (within %.0f dB of the strongest):' % args.within)
            for r in on:
                reg = registry.get(r['epc'])
                if reg:
                    names.append(reg['name'])
                    print('  %-9s %-8s tag  %-9s  %6.1f dBm  %4d reads  %s' % (
                        reg['name'], reg['tag_type'], reg['category'], r['rssi_mean'], r['reads'],
                        ('object ' + reg['object']) if reg['object'] else ''))
                else:
                    print('  %-26s UNREGISTERED  %6.1f dBm  %4d reads' % (r['epc'], r['rssi_mean'], r['reads']))
            if weak:
                print('  also seen, weaker (probably not this item): ' + ', '.join(
                    '%s %.1f dBm' % (registry[r['epc']]['name'] if r['epc'] in registry else r['epc'][-6:], r['rssi_mean']) for r in weak))

            # group check against the registry
            objs = {registry[r['epc']]['object'] for r in on if r['epc'] in registry and registry[r['epc']]['object']}
            notes = []
            for obj in sorted(objs):
                members = [v['name'] for v in registry.values() if v['object'] == obj]
                missing = [m for m in members if m not in names]
                extra = [n for n in names if registry_name_obj(registry, n) not in ('', obj)]
                if missing:
                    notes.append('object %s should also have %s - not read (hidden side? try turning the item)' % (obj, ', '.join(missing)))
                if extra:
                    notes.append('%s belong(s) to a different object - is a second item in range?' % ', '.join(extra))
            unreg = [r['epc'] for r in on if r['epc'] not in registry]
            if unreg:
                notes.append('unregistered tag(s) read with it: %s - register them or keep them away' % ', '.join(e[-8:] for e in unreg))
            if len(objs) > 1:
                notes.append('tags from %d different objects (%s): move the other items away and read again' % (len(objs), ', '.join(sorted(objs))))
            if not objs and len(names) > 1:
                notes.append('%d single-tag items read together: move the others away and read again' % len(names))
            for n in notes:
                print('  ! ' + n)
            order = {'regular': 0, 'metal': 1, 'liquid': 2}
            tagged = sorted(((registry_name_type(registry, n), n) for n in names), key=lambda t: order.get(t[0], 9))
            line = '   '.join('%s = %s' % (t.upper(), item_no(n)) for t, n in tagged)
            if names and not notes:
                print('\n  ==> WRITE ON IT:   %s\n' % line)
            elif names:
                print('\n  ==> probable:      %s   (check the notes above)\n' % line)
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndisconnecting ...')
        reader.disconnect()


def registry_name_type(registry, name):
    for v in registry.values():
        if v['name'] == name:
            return v['tag_type']
    return '?'


def registry_name_obj(registry, name):
    for v in registry.values():
        if v['name'] == name:
            return v['object']
    return ''


if __name__ == '__main__':
    main()
