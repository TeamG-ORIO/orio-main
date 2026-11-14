#!/usr/bin/env python3
"""
make_scene.py - build the map scene of the demo window from the run-2 replay page's files (one-off, per site).

    python3 make_scene.py <meta.json> <cloud.b64.txt> [name, default scene_v2]

Inputs are the two files viz_pack_v2.py writes on abuela (~/rfid_db/loc/viz/site_v2/): the shelf boxes and the scene
point cloud of the replay page. Writes <name>.json (shelf boxes, room frame, metres) and <name>_cloud.npy
(float32 x, y, z, grey 0..1: the robot's dense map cloud of 2026-11-06, cropped and thinned). demo/scene_view.py
draws both; live_site_v2.json names the scene with its "scene" key.
"""
import base64, json, sys
import numpy as np
meta, cloud, name = sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else 'scene_v2'
m = json.load(open(meta))
C = np.frombuffer(base64.b64decode(open(cloud).read().strip()), np.float32).reshape(-1, 4)
np.save(name + '_cloud.npy', C)
json.dump({'_about': 'Map scene for demo/scene_view.py, made by make_scene.py from the run-2 replay page (abuela ~/rfid_db/loc/viz/site_v2). '
                     'Shelf box = front-bottom-left corner a, along-shelf unit u, outward normal nrm, width W, height H, bottom z0, depth into the shelf.',
           'shelves': m['shelves'], 'cloud_file': name + '_cloud.npy', 'cloud_points': int(len(C))}, open(name + '.json', 'w'), indent=1)
print('%s.json: %d shelves; %s_cloud.npy: %d points' % (name, len(m['shelves']), name, len(C)))
