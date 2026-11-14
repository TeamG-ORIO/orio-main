#!/usr/bin/env python3
"""
register_reference_tags.py - register the four calibration (reference) tags of each shelf.

Same reader setup and CLEAR test as register_tags.py (lowest power, 3 s window, big RSSI margin),
but the names are fixed: viewed from the FRONT of the shelf the order is
    top-left  ->  top-right  ->  bottom-left  ->  bottom-right
and the tags are written to tag_registry.csv as
    name = "Ref S<shelf> <TL|TR|BL|BR>"   category = "reference"   object = "Shelf <shelf>"
so any later code can pick them with  category == "reference".

    python3 register_reference_tags.py              # starts at the first shelf with no reference tags yet
    python3 register_reference_tags.py --shelf 3    # start (or redo) at shelf 3

Keys:
    Enter   scan 3 s and register the tag in front as the corner shown in the prompt
    r       undo the previous corner (deletes it, asks for it again)
    b       background scan (nothing in front)
    q       quit
"""
import argparse
import re
import sys
import time

sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
import register_tags as rt  # noqa: E402

CORNERS = [('TL', 'top-left'), ('TR', 'top-right'), ('BL', 'bottom-left'), ('BR', 'bottom-right')]


def ref_name(shelf, code):
    return 'Ref S%d %s' % (shelf, code)


def shelves_in(registry):
    return sorted({int(m.group(1)) for r in registry.values()
                   for m in [re.match(r'Ref S(\d+) ', r.get('name', ''))] if m})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ip', default='192.168.0.20')
    ap.add_argument('--antennas', type=int, nargs='+', default=[4])
    ap.add_argument('--power', type=float, default=10.0, help='dBm; 10 is the R700 minimum')
    ap.add_argument('--duration', type=float, default=3.0)
    ap.add_argument('--bg-duration', type=float, default=10.0)
    ap.add_argument('--margin', type=float, default=6.0)
    ap.add_argument('--min-reads', type=int, default=5)
    ap.add_argument('--registry', default=rt.REGISTRY)
    ap.add_argument('--shelf', type=int, default=None, help='shelf number to start at (default: next shelf without reference tags)')
    args = ap.parse_args()

    registry = rt.load_registry(args.registry)
    done = shelves_in(registry)
    shelf = args.shelf if args.shelf is not None else (max(done) + 1 if done else 1)
    print('%d tag(s) in %s; shelves with reference tags so far: %s' % (
        len(registry), args.registry, ', '.join(map(str, done)) or 'none'))
    reader, col, used = rt.connect(args)
    power_dbm = min(used.values()) if used else args.power
    background = {}
    history = []  # names registered in this session, for r
    print('\nViewed from the FRONT of the shelf: top-left, top-right, bottom-left, bottom-right.\n'
          'Enter = scan   r = undo previous corner   b = background   q = quit\n')
    try:
        while True:
            # first corner of this shelf that is not registered yet
            pending = [(c, label) for c, label in CORNERS if not any(r['name'] == ref_name(shelf, c) for r in registry.values())]
            if not pending:
                ans = input('Shelf %d complete. Enter = next shelf (%d), q = quit > ' % (shelf, shelf + 1)).strip().lower()
                if ans == 'q':
                    break
                shelf += 1
                continue
            code, label = pending[0]
            try:
                key = input('Shelf %d %s (%s): hold the tag to the antenna, Enter to scan > ' % (shelf, label, ref_name(shelf, code))).strip().lower()
            except EOFError:
                break
            if key == 'q':
                break
            if key == 'r':
                if not history:
                    print('nothing to undo in this session')
                    continue
                name = history.pop()
                epc = next((e for e, r in registry.items() if r['name'] == name), None)
                if epc:
                    del registry[epc]
                    rt.save_registry(args.registry, registry)
                    print('removed %s (%s), asking for it again' % (name, epc))
                    m = re.match(r'Ref S(\d+) ', name)
                    shelf = int(m.group(1))
                continue
            if key == 'b':
                print('background scan, %.0f s - keep tags (and yourself) away from the antenna ...' % args.bg_duration)
                rows = rt.summarize(col.window(args.bg_duration))
                background.update({r['epc']: r['rssi_mean'] for r in rows if r['rssi_mean'] is not None})
                print(rt.fmt_table(rows, registry, {}, n=10) if rows else '-> nothing seen. Good.')
                continue
            if key != '':
                print('unknown key %r' % key)
                continue

            print('scanning %.0f s ...' % args.duration)
            rows = rt.summarize(col.window(args.duration))
            print(rt.fmt_table(rows, registry, background))
            verdict, reasons = rt.decide(rows, args.margin, args.min_reads, background)
            if verdict != 'CLEAR':
                print('%s: %s  -> NOT registered, press Enter to retry' % (verdict, '; '.join(reasons)))
                continue
            top = rows[0]
            if top['epc'] in registry:
                r = registry[top['epc']]
                print('CLEAR but that tag is already "%s" (%s) - not using it as %s. Hold a different tag.'
                      % (r['name'], r['category'], ref_name(shelf, code)))
                continue
            name = ref_name(shelf, code)
            row = {'epc': top['epc'], 'name': name, 'category': 'reference', 'tag_type': rt.tag_type(top['epc']),
                   'object': 'Shelf %d' % shelf, 'registered_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                   'rssi_dbm': '%.1f' % top['rssi_mean'], 'reads': top['reads'],
                   'antenna': ','.join(map(str, args.antennas)), 'power_dbm': '%.2f' % power_dbm}
            rt.append_registry(args.registry, row)
            registry[top['epc']] = row
            history.append(name)
            print('CLEAR (%s)  ->  %s = %s  (%s tag)' % ('; '.join(reasons), name, top['epc'], row['tag_type']))
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndisconnecting ...')
        reader.disconnect()
        refs = sorted((r['name'] for r in registry.values() if r['category'] == 'reference'), key=lambda n: (int(re.search(r'S(\d+)', n).group(1)), n))
        print('%d reference tag(s): %s' % (len(refs), ', '.join(refs) or 'none'))


if __name__ == '__main__':
    main()
