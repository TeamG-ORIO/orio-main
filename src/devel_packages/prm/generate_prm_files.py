#!/usr/bin/env python3
"""
Generate 6 PRM files in prm_files/ by varying the zone_block position (or
removing it entirely) in a temporary copy of orio_dual_scene.xml, then running
the PRM builder for each configuration in parallel.

Generated files:
  prm_files/myPRM_arm1_labelZone1.p   arm1, zone_block y=+0.273
  prm_files/myPRM_arm1_labelZone2.p   arm1, zone_block y=-0.273
  prm_files/myPRM_arm1_free.p         arm1, no zone_block
  prm_files/myPRM_arm2_labelZone1.p   arm2, zone_block y=-0.273
  prm_files/myPRM_arm2_labelZone2.p   arm2, zone_block y=+0.273
  prm_files/myPRM_arm2_free.p         arm2, no zone_block

Usage:
  python generate_prm_files.py
"""

import os
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
import multiprocessing as mp

# ── Paths ─────────────────────────────────────────────────────────────────────

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SCENE_XML  = os.path.join(SCRIPT_DIR, "mujoco_files", "orio_dual_scene.xml")
OUT_DIR    = os.path.join(SCRIPT_DIR, "prm_files_dense")

os.makedirs(OUT_DIR, exist_ok=True)

# ── Configurations ────────────────────────────────────────────────────────────

CONFIGS = [
    # (output_filename,            arm, zone_y,   include_zone)
    ("myPRM_arm1_labelZone1.p",    1,   0.273,    True),
    ("myPRM_arm1_labelZone2.p",    1,  -0.273,    True),
    ("myPRM_arm1_free.p",          1,   None,     False),
    ("myPRM_arm2_labelZone1.p",    2,  -0.273,    True),
    ("myPRM_arm2_labelZone2.p",    2,   0.273,    True),
    ("myPRM_arm2_free.p",          2,   None,     False),
]

# ── XML helpers ───────────────────────────────────────────────────────────────

def _build_scene_xml(zone_y, include_zone):
    ET.register_namespace("", "")
    tree = ET.parse(SCENE_XML)
    root = tree.getroot()
    for body in root.iter("body"):
        for geom in list(body.findall("geom")):
            if geom.get("name") == "zone_block":
                if not include_zone:
                    body.remove(geom)
                else:
                    parts    = geom.get("pos", "0 0 0").split()
                    parts[1] = str(zone_y)
                    geom.set("pos", " ".join(parts))
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".xml",
        dir=os.path.join(SCRIPT_DIR, "mujoco_files"),
        delete=False
    )
    tree.write(tmp.name, encoding="unicode", xml_declaration=False)
    tmp.close()
    return tmp.name


# ── Worker ────────────────────────────────────────────────────────────────────

def _worker(arm_number, scene_xml_path, out_path, queue):
    """
    Runs in a separate process. Patches scene_obstacles._SCENE_XML locally
    (safe — each process has its own memory), then calls PRMGenerator with a
    progress_callback that pushes counts onto the queue.
    """
    sys.path.insert(0, SCRIPT_DIR)
    import scene_obstacles as so
    from PRMGenerator_DLS_JPI import PRMGenerator

    so._SCENE_XML = scene_xml_path

    def _progress(n):
        queue.put((out_path, n))

    PRMGenerator(arm_number, out_path=out_path, progress_cb=_progress)
    queue.put((out_path, "done"))


# ── Progress renderer ─────────────────────────────────────────────────────────

def _render_bars(counts, total=10000, width=30):
    lines = []
    for label, n in counts.items():
        pct = n / total
        bar = int(pct * width)
        lines.append(f"  {label:35s} [{'█'*bar}{'░'*(width-bar)}] {n}/{total}")
    # Move cursor up to overwrite previous render
    if hasattr(_render_bars, "_drawn"):
        sys.stdout.write(f"\033[{len(lines)}A")
    _render_bars._drawn = True
    print("\n".join(lines), flush=True)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    t0 = time.time()

    queue    = mp.Queue()
    tmp_xmls = []
    procs    = []
    counts   = {}

    for fname, arm, zone_y, include_zone in CONFIGS:
        out_path = os.path.join(OUT_DIR, fname)
        tmp_xml  = _build_scene_xml(zone_y, include_zone)
        tmp_xmls.append(tmp_xml)
        counts[fname] = 0
        p = mp.Process(target=_worker, args=(arm, tmp_xml, out_path, queue))
        p.start()
        procs.append(p)

    print(f"Launched {len(procs)} workers\n")
    # Print initial blank bars
    _render_bars(counts)

    done = set()
    while len(done) < len(CONFIGS):
        msg = queue.get()
        label, val = os.path.basename(msg[0]), msg[1]
        if val == "done":
            counts[label] = 1000
            done.add(label)
        else:
            counts[label] = val
        _render_bars(counts)

    for p in procs:
        p.join()
    for tmp in tmp_xmls:
        os.unlink(tmp)

    print(f"\nAll done in {time.time() - t0:.1f}s")


if __name__ == "__main__":
    main()
