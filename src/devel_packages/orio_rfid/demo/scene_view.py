#!/usr/bin/env python3
"""
scene_view.py - the map of the scan demo window: the room with its shelves and point cloud, the robot, and where
each item is thought to be, drawn as a picture. One renderer, two cameras: 2D (straight down) and 3D (orbiting).

It is the scene of the run-2 replay page (same room frame, same shelf colours, same point cloud), drawn here with
numpy and Pillow instead of a browser so that it sits inside the tkinter window. scan_dashboard.py owns the mouse and
hands in what changes; this file only turns it into a picture:

    scene = Scene('<orio_rfid>/loc/site')            # shelves, cloud, tape-measured positions, map -> room tie
    view = SceneView(scene); view.mode = '3d'
    image = view.render(width, height, data)         # PIL image; view.picks = what is under the mouse afterwards

data (all positions in the ROOM frame, metres):
    items      [{uid, name, x, y, z, shelf, r90, state, note}]   state: 'new' placed in this scan, 'old' last known place,
                                                                'missing' not seen in a finished scan; note = second half of the label
    robot      (x, y, yaw, antenna z, live) or None      path  [(x, y), ...]      waypoints  [(x, y, name, state)]
    selected, hover   uid or None        cloud, regions, truth   switches        hud   text lines for the corner

Standard library + numpy + Pillow. Try it without the window:   python3 scene_view.py out.png [2d|3d]
"""
import colorsys
import csv
import json
import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

STAGE, GRID, ROBOT, PATH = (6, 8, 12), (26, 34, 47), (127, 211, 255), (137, 220, 235)
GREEN, YELLOW, RED, GREY, TEXT, SUB = (166, 227, 161), (249, 226, 175), (243, 139, 168), (108, 112, 134), (205, 214, 244), (166, 173, 200)
NEAR, FOV = 0.15, 50.0
EDGES = ((0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7))


def _font(size):
    for name in ('DejaVuSans.ttf', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


class Scene:
    """what does not change during a run: shelf boxes, the map's point cloud, the tape-measured positions, the map -> room tie"""

    def __init__(self, site_dir, site_file='live_site_v2.json'):
        site = json.load(open(os.path.join(site_dir, site_file)))
        self.R, self.t = np.array(site['map_to_room']['R'], float), np.array(site['map_to_room']['t'], float)
        self.yaw0 = math.atan2(self.R[1, 0], self.R[0, 0]); self.off = site['antenna_offset']
        sc = json.load(open(os.path.join(site_dir, site['scene'])))
        ids = sorted(int(s['id']) for s in sc['shelves']); self.shelves = {}
        for s in sc['shelves']:              # one hue per shelf, as on the replay page
            hue = ((275 - ids.index(int(s['id'])) * 360 / len(ids)) % 360) / 360
            a, u, n = np.array(s['a'], float), np.array(s['u'], float), np.array(s['nrm'], float); z0, z1 = s['z0'], s['z0'] + s['H']
            c = np.array([np.r_[a + al * u - dp * n, z] for z in (z0, z1) for al, dp in ((0, 0), (s['W'], 0), (s['W'], s['depth']), (0, s['depth']))])
            self.shelves[int(s['id'])] = dict(color=tuple(int(255 * v) for v in colorsys.hls_to_rgb(hue, 0.58, 0.78)), corners=c, hue=hue)
        path = os.path.join(site_dir, sc.get('cloud_file', ''))
        C = np.load(path) if os.path.isfile(path) else np.zeros((0, 4), np.float32)
        self.cloud = C[:, :3].astype(float); g = 0.14 + 0.6 * C[:, 3:4]
        self.cloud_rgb = (np.clip(g * np.array([0.92, 0.96, 1.08]), 0, 1) * 255).astype(np.uint8)
        self.truth, self.tag_color = {}, {}  # rfid_uid -> tape-measured room position (items only); rfid_uid -> colour
        try:
            by = {}
            for r in csv.DictReader(open(os.path.join(site_dir, site['ground_truth']))):
                by.setdefault(int(r['shelf']), []).append(r)
                if r.get('category') != 'reference' and r.get('X_room_m'):
                    self.truth[r['epc'].strip().upper()] = (float(r['X_room_m']), float(r['Y_room_m']), float(r['Z_room_m']))
            for sid, rows in by.items():     # the replay page's colours: the shelf's hue, a shade per tag from dark to light in the
                if sid not in self.shelves:  # page's tag order (corner tags first, then the items by number)
                    continue
                rows.sort(key=lambda r: (not r['name'].startswith('Ref '), int(r['name'].split()[-1]) if r['name'].startswith('Item') and r['name'].split()[-1].isdigit() else 0, r['name']))
                for j, r in enumerate(rows):
                    light = 0.36 + 0.4 * (j / (len(rows) - 1) if len(rows) > 1 else 0.5)
                    self.tag_color[r['epc'].strip().upper()] = tuple(int(255 * v) for v in colorsys.hls_to_rgb(self.shelves[sid]['hue'], light, 0.78))
        except (OSError, KeyError, ValueError):
            pass
        allc = np.concatenate([s['corners'][:, :2] for s in self.shelves.values()])
        self.lo, self.hi = allc.min(0), allc.max(0)

    def room_xy(self, xy):
        """map-frame points (N, 2) -> room frame"""
        return np.asarray(xy, float).reshape(-1, 2) @ self.R.T + self.t


class SceneView:
    def __init__(self, scene):
        self.s, self.mode = scene, '2d'
        self.f_small, self.f_label = _font(11), _font(12)
        self.picks = ([], np.zeros((0, 2)))
        self.W, self.H, self.F = 640, 400, 0.5 * 400 / math.tan(math.radians(FOV / 2))      # replaced at every render
        self.reset()

    # ------------------------------------------------------------------ camera
    def reset(self):
        c = (self.s.lo + self.s.hi) / 2; span = float(max(self.s.hi - self.s.lo)) + 1.5
        off = np.array([-0.25 * span, -0.85 * span, span - 0.5])        # the replay page's opening view
        self.target = np.array([c[0], c[1], 0.5]); self.dist = float(np.linalg.norm(off))
        self.az, self.el = math.atan2(off[1], off[0]), math.asin(off[2] / self.dist)
        self.c2, self.s2, self.fit2 = c.copy(), 40.0, True      # 2D: centre (room x, y), pixels per metre; fit2 = keep fitting the
                                                                # shelves to the picture until the view is moved by hand

    def _basis(self):
        ce = math.cos(self.el); d = np.array([ce * math.cos(self.az), ce * math.sin(self.az), math.sin(self.el)])
        f = -d; r = np.cross(f, [0.0, 0.0, 1.0]); r /= np.linalg.norm(r)
        return self.target + self.dist * d, r, np.cross(r, f), f

    def project(self, P):
        """room points (N, 3) -> screen x, screen y, depth (smaller = nearer; <= NEAR is behind the 3D camera), pixels per metre there"""
        P = np.asarray(P, float).reshape(-1, 3)
        if self.mode == '2d':
            return self.W / 2 + self.s2 * (P[:, 0] - self.c2[0]), self.H / 2 - self.s2 * (P[:, 1] - self.c2[1]), 50.0 - P[:, 2], np.full(len(P), self.s2)
        eye, r, u, f = self.b; q = P - eye; z = q @ f; zc = np.maximum(z, 1e-6)
        return self.W / 2 + self.F * (q @ r) / zc, self.H / 2 - self.F * (q @ u) / zc, z, self.F / zc

    def drag(self, dx, dy, pan=False):
        if self.mode == '2d':
            self.c2 = self.c2 - np.array([dx, -dy]) / self.s2; self.fit2 = False
        elif pan:
            _, r, _, f = self._basis(); g = np.array([f[0], f[1], 0.0]); g /= max(np.linalg.norm(g), 1e-6)
            self.target = self.target + (-dx * r + dy * g) * self.dist / self.F
        else:
            self.az -= dx * 0.008; self.el = min(max(self.el + dy * 0.008, 0.08), 1.45)

    def zoom(self, steps, mx, my):
        k = 1.15 ** steps
        if self.mode == '2d':
            w = self.c2 + np.array([mx - self.W / 2, -(my - self.H / 2)]) / self.s2      # the point under the mouse stays put
            self.s2 = min(max(self.s2 * k, 8.0), 900.0); self.c2 = w - np.array([mx - self.W / 2, -(my - self.H / 2)]) / self.s2
            self.fit2 = False
        else:
            self.dist = min(max(self.dist / k, 0.8), 40.0)

    # ------------------------------------------------------------------ drawing
    def _line(self, d, A, B, fill, width=1):
        A, B = np.asarray(A, float), np.asarray(B, float)
        if self.mode == '3d':                # cut the segment at the near plane, or drop it if it is all behind the camera
            eye, _, _, f = self.b; za, zb = (A - eye) @ f, (B - eye) @ f
            if za < NEAR and zb < NEAR:
                return
            if za < NEAR:
                A = B + (A - B) * (zb - NEAR) / (zb - za)
            elif zb < NEAR:
                B = A + (B - A) * (za - NEAR) / (za - zb)
        sx, sy, _, _ = self.project([A, B])
        d.line([(sx[0], sy[0]), (sx[1], sy[1])], fill=fill, width=width)

    def _poly(self, d, P, fill=None, outline=None):
        sx, sy, z, _ = self.project(P)
        if self.mode == '3d' and z.min() < NEAR:
            return
        d.polygon(list(zip(sx, sy)), fill=fill, outline=outline)

    def _label(self, d, x, y, text, fill, font=None, anchor='lt'):
        font = font or self.f_small; b = d.textbbox((0, 0), text, font=font); w, h = b[2] - b[0], b[3] - b[1]
        if anchor == 'mb':
            x, y = x - w / 2, y - h - 4
        x, y = min(max(x, 2), self.W - w - 6), min(max(y, 2), self.H - h - 6)
        d.rectangle([x - 3, y - 2, x + w + 3, y + h + 4], fill=(10, 13, 18, 205))
        d.text((x, y - b[1]), text, fill=fill, font=font)

    def render(self, W, H, data):
        s = self.s; self.W, self.H = W, H
        if self.fit2:                        # 2D: the shelves with a margin, whatever size the picture has
            ext = (s.hi - s.lo) + 1.6; self.s2 = float(min(W / ext[0], H / ext[1])); self.c2 = (s.lo + s.hi) / 2
        self.b, self.F = self._basis(), 0.5 * H / math.tan(math.radians(FOV / 2))
        flat = self.mode == '2d'
        im = Image.new('RGB', (W, H), STAGE); d = ImageDraw.Draw(im, 'RGBA')
        lo, hi = np.floor(s.lo) - 2, np.ceil(s.hi) + 2          # 1 m floor grid
        for gx in np.arange(lo[0], hi[0] + 1):
            self._line(d, (gx, lo[1], 0), (gx, hi[1], 0), GRID)
        for gy in np.arange(lo[1], hi[1] + 1):
            self._line(d, (lo[0], gy, 0), (hi[0], gy, 0), GRID)
        if data.get('cloud', True) and len(s.cloud):            # point cloud over the grid: far points first, near ones win
            step = 3 if data.get('fast') else 1                 # while the mouse is dragging: a third of the points
            keep = np.flatnonzero(s.cloud[:, 2] > 0.10)[::step] if flat else slice(None, None, step)
            sx, sy, z, _ = self.project(s.cloud[keep]); col = s.cloud_rgb[keep]
            m = (z > NEAR) & (sx >= 0) & (sx < W - 1) & (sy >= 0) & (sy < H - 1)
            xi, yi, zz, col = sx[m].astype(int), sy[m].astype(int), z[m], col[m]
            o = np.argsort(-zz); xi, yi, zz = xi[o], yi[o], zz[o]; col = (col[o] * 0.72).astype(np.uint8)
            a = np.array(im); a[yi, xi] = col
            big = np.ones(len(zz), bool) if flat else zz < 6.0  # near points are 2 x 2 pixels so the surfaces close up
            for ox, oy in ((1, 0), (0, 1), (1, 1)):
                a[yi[big] + oy, xi[big] + ox] = col[big]
            im = Image.fromarray(a); d = ImageDraw.Draw(im, 'RGBA')
        for sid, sh in s.shelves.items():                       # shelf boxes
            c, col = sh['corners'], sh['color']
            if flat:
                self._poly(d, c[:4], fill=col + (30,), outline=col + (200,))
                sx, sy, _, _ = self.project(c[:4].mean(0)); d.text((sx[0] - 8, sy[0] - 7), 'S%d' % sid, fill=col + (230,), font=self.f_label)
            else:
                self._poly(d, c[:4], fill=col + (22,)); self._poly(d, c[[3, 2, 6, 7]], fill=col + (18,))
                for i, j in EDGES:
                    self._line(d, c[i], c[j], col + (150,))
                sx, sy, z, _ = self.project(c[4:].mean(0))
                if z[0] > NEAR:
                    d.text((sx[0] - 8, sy[0] - 16), 'S%d' % sid, fill=col + (230,), font=self.f_label)
        path = data.get('path')
        if path is not None and len(path) > 1:                  # driven path on the floor
            sx, sy, z, _ = self.project(np.c_[np.asarray(path, float), np.full(len(path), 0.01)])
            run = []
            for x, y, zz in zip(sx, sy, z):
                if zz > NEAR:
                    run.append((x, y))
                else:
                    if len(run) > 1:
                        d.line(run, fill=PATH + (200,), width=2)
                    run = []
            if len(run) > 1:
                d.line(run, fill=PATH + (200,), width=2)
        ring = np.array([[math.cos(k * math.pi / 8), math.sin(k * math.pi / 8), 0.0] for k in range(16)])
        for x, y, name, st in data.get('waypoints') or ():
            col = {'ok': GREEN, 'goal': YELLOW, 'fail': RED}.get(st, GREY)
            self._poly(d, np.array([x, y, 0.01]) + 0.11 * ring, fill=col + (150,) if st in ('ok', 'fail', 'goal') else None, outline=col + (255,))
            if flat and name:
                sx, sy, _, _ = self.project([x, y, 0]); d.text((sx[0] + 8, sy[0] - 16), name, fill=SUB, font=self.f_small)
        rob = data.get('robot')
        if rob:                                                 # base, heading, mast, antenna panel and its boresight (robot's left)
            x, y, yaw, az, live = rob; col = ROBOT if live else GREY
            fw, lf = np.array([math.cos(yaw), math.sin(yaw), 0.0]), np.array([-math.sin(yaw), math.cos(yaw), 0.0]); base = np.array([x, y, 0.02])
            self._poly(d, base + 0.17 * ring, fill=(43, 62, 82, 235), outline=col + (255,))
            self._poly(d, [base + 0.30 * fw, base + 0.13 * fw + 0.09 * lf, base + 0.13 * fw - 0.09 * lf], fill=col + (255,))
            ant = np.array([x, y, 0.0]) + s.off[0] * fw + s.off[1] * lf; up = np.array([0.0, 0.0, 1.0])
            self._line(d, ant + 0.2 * up, ant + 1.5 * up, (110, 126, 150, 255), 2)
            c = ant + az * up
            self._poly(d, [c + 0.15 * fw + 0.15 * up, c - 0.15 * fw + 0.15 * up, c - 0.15 * fw - 0.15 * up, c + 0.15 * fw - 0.15 * up], fill=col + (210,), outline=col + (255,))
            self._line(d, c, c + 0.6 * lf, col + (230,), 2)
        items = data.get('items') or []
        uids, xy = [], []
        if items:
            P = np.array([[i['x'], i['y'], i['z']] for i in items], float); sx, sy, z, ppm = self.project(P)
            sel, hov, truth = data.get('selected'), data.get('hover'), data.get('truth') and s.truth
            for k in np.argsort(-z):
                i = items[k]
                if z[k] <= NEAR:
                    continue
                x, y = float(sx[k]), float(sy[k])
                # as on the replay page: the tag's own colour (its measured shelf's hue, its own shade); a tag the tape sheet
                # does not know takes the colour of the shelf it was placed on
                col = s.tag_color.get(i['uid']) or (s.shelves[i['shelf']]['color'] if i.get('shelf') in s.shelves else PATH)
                mark = i['uid'] in (sel, hov)
                if i.get('r90') and (mark or (data.get('regions') and i['state'] == 'new')):       # translucent, no hard edge
                    r = i['r90'] * float(ppm[k]); d.ellipse([x - r, y - r, x + r, y + r], fill=col + (46 if mark else 20,), outline=col + (150,) if mark else None)
                if truth and i['uid'] in s.truth and i['state'] == 'new':                           # diamond at the tape position, line to the estimate
                    tx, ty, tz, tp = self.project(s.truth[i['uid']])
                    if tz[0] > NEAR:
                        tx, ty = float(tx[0]), float(ty[0]); h = min(max(0.036 * float(tp[0]), 4.0), 12.0); w = 0.72 * h
                        d.line([(x, y), (tx, ty)], fill=col + (205,), width=1)
                        d.polygon([(tx, ty - h), (tx + w, ty), (tx, ty + h), (tx - w, ty)], fill=col + (255,))       # flat, in the tag's colour
                r = min(max(0.035 * float(ppm[k]), 3.0), 12.0) + (1 if mark else 0)                # a 3.5 cm ball, as on the page
                if i['state'] == 'new':
                    d.ellipse([x - r, y - r, x + r, y + r], fill=col + (255,))
                elif i['state'] == 'missing':
                    d.ellipse([x - r, y - r, x + r, y + r], outline=RED + (255,), width=2)
                    d.line([(x - 3, y - 3), (x + 3, y + 3)], fill=RED + (255,), width=2); d.line([(x - 3, y + 3), (x + 3, y - 3)], fill=RED + (255,), width=2)
                else:
                    d.ellipse([x - r, y - r, x + r, y + r], outline=GREY + (220,), width=1)
                if mark:
                    d.ellipse([x - r - 4, y - r - 4, x + r + 4, y + r + 4], outline=(255, 255, 255, 230), width=2)
                uids.append(i['uid']); xy.append((x, y))
            for uid in (hov, sel):                              # label on top of everything
                k = next((k for k, i in enumerate(items) if i['uid'] == uid), None)
                if k is not None and z[k] > NEAR:
                    i = items[k]; self._label(d, float(sx[k]), float(sy[k]) - 8, '%s   %s' % (i['name'], i.get('note', '')), TEXT, self.f_label, 'mb')
        self.picks = (uids, np.array(xy, float).reshape(-1, 2))
        y = 6
        for line in data.get('hud') or ():
            self._label(d, 8, y, line, SUB); y += 19
        hint = 'drag: move · wheel: zoom · double-click: reset' if flat else 'drag: orbit · right-drag: move · wheel: zoom · double-click: reset'
        d.text((8, H - 18), hint, fill=GREY + (255,), font=self.f_small)
        return im

    def pick(self, mx, my, radius=10):
        """uid of the item nearest to the mouse, within radius pixels"""
        uids, xy = self.picks
        if not len(uids):
            return None
        dd = np.hypot(xy[:, 0] - mx, xy[:, 1] - my); k = int(dd.argmin())
        return uids[k] if dd[k] <= radius else None


if __name__ == '__main__':       # a still picture of the empty room, to see the renderer without the window
    import sys
    sc = Scene(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'loc', 'site')); v = SceneView(sc)
    v.mode = sys.argv[2] if len(sys.argv) > 2 else '3d'
    v.render(900, 600, {'hud': ['%d shelves, %d cloud points' % (len(sc.shelves), len(sc.cloud))]}).save(sys.argv[1])
