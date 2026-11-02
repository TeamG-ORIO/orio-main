#!/usr/bin/env python3
"""Reference implementation of the revised label placement (see DESIGN.md).

Finds where to put a rectangular label on an item seen from above, so that the label
  1. lies wholly on the item (segmentation mask),
  2. does not cover text (text detector, and optionally the old gradient-edge check),
  3. sits on a flat part of the item (depth), ignoring the few-mm depth noise that stereo
     depth shows on glossy/clear plastic.

Inputs per capture folder (as saved by the robot's capture script):
    color.png   left camera image, 1920x1080 BGR
    depth.npy   float32 metres, same size as color.png, NaN where invalid
    K.npy       3x3 camera intrinsics (optional here)

Usage:
    python label_placement.py CAPTURE_DIR [CAPTURE_DIR ...] [--text none|ppocr] [--no-edges] [--out OUT_DIR]

Dependencies: numpy, opencv-python(-headless), scipy; for --text ppocr also
`rapidocr==3.10.0 onnxruntime` (models download on first use).
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt

# ── Parameters (pixels are at 1920x1080, ~0.65 mm/px at the 0.98 m camera height) ──────
LABEL_W, LABEL_H = 120, 40     # label footprint, px (~78 x 26 mm)
EDGE_MAX = 0.05                # max fraction of footprint pixels on gradient edges (old check)
TEXT_MAX = 0.0                 # max fraction of footprint pixels on detected text
PV_FLAT = 2.0                  # mm: unevenness at or below this counts as flat (noise level)
PV_MAX = 5.0                   # mm: above this the placement is flagged as unusable
MARGIN = 8                     # px added on each side of the footprint for the depth check (~5 mm)
MASK_ERODE = 10                # px the mask is shrunk by for the depth check (edge staircase)
DEPTH_DS = 2                   # depth is used at 1/2 resolution (stereo net runs at half res)
STRIDE = 6                     # px between footprint centres in the grid search
ALPHA = 0.2                    # weight of distance-to-boundary vs distance-to-edges (old code)


# ── Inputs ───────────────────────────────────────────────────────────────────────────
def load_capture(d):
    color = cv2.imread(os.path.join(d, 'color.png'))
    depth = np.load(os.path.join(d, 'depth.npy')).astype(np.float32)
    if color is None or depth.shape != color.shape[:2]:
        raise ValueError('%s: need color.png and depth.npy of the same size' % d)
    return color, depth


def segment_items_on_dark_background(color, min_area=4000, border=(150, 80, 120)):
    """TEST-ONLY stand-in for the real segmentation (GroundingDINO + SAM): items are much
    brighter than the black cloth they lie on. Returns a list of (mask uint8 0/1, bbox)."""
    g = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    m = (g > 60).astype(np.uint8)
    top, bottom, side = border                          # ignore the frame edges (clutter)
    m[:top] = 0; m[-bottom:] = 0; m[:, :side] = 0; m[:, -side:] = 0
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    return [((lab == i).astype(np.uint8), tuple(st[i, :4])) for i in range(1, n) if st[i, 4] >= min_area]


# ── Text / edge maps ─────────────────────────────────────────────────────────────────
def gradient_edges(color, thresh=100):
    """The current method's edge map: Sobel magnitude on grey > 100 (0/1)."""
    g = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    mag = cv2.convertScaleAbs(cv2.magnitude(cv2.Sobel(g, cv2.CV_64F, 1, 0), cv2.Sobel(g, cv2.CV_64F, 0, 1)))
    return (mag > thresh).astype(np.uint8)


_OCR = None


def text_mask_ppocr(crop, dilate_px=6):
    """Text regions from PaddleOCR's PP-OCRv5 server detector via RapidOCR (detection only,
    no recognition). Returns a 0/1 mask the size of crop."""
    global _OCR
    if _OCR is None:
        from rapidocr import RapidOCR, OCRVersion, ModelType, EngineType, LangDet
        _OCR = RapidOCR(params={'Det.ocr_version': OCRVersion.PPOCRV5, 'Det.model_type': ModelType.SERVER,
                                'Det.lang_type': LangDet.CH, 'Det.engine_type': EngineType.ONNXRUNTIME,
                                'Global.log_level': 'error'})
    out = _OCR(crop, use_det=True, use_cls=False, use_rec=False)
    m = np.zeros(crop.shape[:2], np.uint8)
    for b in ([] if out.boxes is None else out.boxes):
        cv2.fillPoly(m, [np.int32(np.round(b))], 1)
    if dilate_px:
        m = cv2.dilate(m, np.ones((2 * dilate_px + 1, 2 * dilate_px + 1), np.uint8))
    return m


# ── Depth flatness ───────────────────────────────────────────────────────────────────
def footprint_offsets(angle_deg, w, h):
    """(du, dv) integer offsets of a w x h rectangle rotated by angle_deg about 0."""
    k = int(np.ceil(np.hypot(w, h))) | 1
    c = k // 2
    m = np.zeros((k, k), np.uint8)
    cv2.fillPoly(m, [np.int32(cv2.boxPoints(((c, c), (w, h), angle_deg)))], 1)
    dv, du = np.nonzero(m)
    return du - c, dv - c


class DepthFlatness:
    """Unevenness of the surface under a label footprint.

    unevenness = peak-to-valley (98th - 2nd percentile) of the depth's residual from the
    least-squares plane fitted under the footprint (grown by `margin`), in mm. A plane over
    the whole label absorbs the broad, smooth wander of stereo depth on glossy/clear film;
    a fold, step or content edge leaves a residual about the size of the step."""

    def __init__(self, depth_m, mask, ds=DEPTH_DS, erode_px=MASK_ERODE):
        self.ds = ds
        self.z = depth_m[::ds, ::ds].astype(np.float64) * 1000.0
        m = (mask[::ds, ::ds] > 0) & np.isfinite(self.z)
        k = max(1, erode_px // ds)
        self.m = cv2.erode(m.astype(np.uint8), np.ones((2 * k + 1, 2 * k + 1), np.uint8)) > 0
        self._cache = {}

    def unevenness(self, x, y, angle_deg, w=LABEL_W, h=LABEL_H, margin=MARGIN, min_cover=0.6):
        """(peak-to-valley mm, tilt deg) at footprint centre (x, y) in full-res pixels.
        Uses the part of the grown footprint that is on the eroded mask; (inf, nan) if
        that part is under min_cover of the bare footprint's area."""
        key = (round(angle_deg, 1), w + 2 * margin, h + 2 * margin)
        if key not in self._cache:
            self._cache[key] = footprint_offsets(angle_deg, key[1] / self.ds, key[2] / self.ds)
        du, dv = self._cache[key]
        u, v = int(round(x / self.ds)) + du, int(round(y / self.ds)) + dv
        H, W = self.z.shape
        ok = (u >= 0) & (v >= 0) & (u < W) & (v < H)
        u, v, du, dv = u[ok], v[ok], du[ok], dv[ok]
        ok = self.m[v, u]
        if ok.sum() < min_cover * w * h / self.ds ** 2:
            return np.inf, np.nan
        zz = self.z[v[ok], u[ok]]
        A = np.stack([np.ones(ok.sum()), du[ok], dv[ok]], 1).astype(np.float64)
        coef = np.linalg.lstsq(A, zz, rcond=None)[0]
        r = zz - A @ coef
        lo, hi = np.percentile(r, [2, 98])
        mm_per_px = np.median(zz) / 1509.6 * self.ds        # fx of the ZED-M at 1080p
        return float(hi - lo), float(np.degrees(np.arctan(np.hypot(coef[1], coef[2]) / mm_per_px)))


# ── Search ───────────────────────────────────────────────────────────────────────────
def item_angles(mask):
    """Major and minor axis of the item's minimum-area rectangle, degrees (as current code)."""
    cnts, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    a = cv2.minAreaRect(max(cnts, key=cv2.contourArea))[2] % 180
    return [a, (a + 90) % 180]


def footprint_kernel(angle_deg, w=LABEL_W, h=LABEL_H):
    du, dv = footprint_offsets(angle_deg, w, h)
    r = int(max(np.abs(du).max(), np.abs(dv).max()))
    k = np.zeros((2 * r + 1, 2 * r + 1), np.uint8)
    k[dv + r, du + r] = 1
    return k


def place_label(crop, mask, depth, text=None, use_edges=True):
    """Best label footprint on one item.

    crop: BGR image region, mask: 0/1 item mask (same size), depth: metres (same size, or
    None for no depth term), text: 0/1 text mask or None.
    Returns dict(x, y, angle, unevenness_mm, tilt_deg, status, n_feasible, box) or None.
    status: 'flat' | 'uneven' (best was PV_FLAT..PV_MAX) | 'flagged' (best > PV_MAX)."""
    mask = (mask > 0).astype(np.uint8)
    edges = gradient_edges(crop) if use_edges else np.zeros_like(mask)
    obstacles = np.maximum(edges, text if text is not None else 0)
    # the current method's preference: far from the item boundary and from edges/text
    pref = ALPHA * distance_transform_edt(mask) + (1 - ALPHA) * distance_transform_edt(mask * (1 - obstacles))
    flat = DepthFlatness(depth, mask) if depth is not None else None
    feasible = []
    for a in item_angles(mask):
        k = footprint_kernel(a)
        kf = k.astype(np.float32) / k.sum()
        inside = cv2.erode(mask, k, borderValue=0)                  # footprint wholly on the item
        edge_frac = cv2.filter2D(edges.astype(np.float32), -1, kf, borderType=cv2.BORDER_CONSTANT)
        text_frac = (cv2.filter2D(text.astype(np.float32), -1, kf, borderType=cv2.BORDER_CONSTANT)
                     if text is not None else np.zeros_like(edge_frac))
        ys, xs = np.mgrid[0:mask.shape[0]:STRIDE, 0:mask.shape[1]:STRIDE]
        ok = (inside[ys, xs] > 0) & (edge_frac[ys, xs] <= EDGE_MAX) & (text_frac[ys, xs] <= TEXT_MAX + 1e-6)
        for x, y in zip(xs[ok], ys[ok]):
            pv, tilt = flat.unevenness(x, y, a) if flat is not None else (0.0, 0.0)
            if np.isfinite(pv):
                feasible.append((pv, tilt, int(x), int(y), float(a)))
    if not feasible:
        return None
    flat_ones = [f for f in feasible if f[0] <= PV_FLAT]
    if flat_ones:   # below the noise level depth has no say: keep the old preference
        pv, tilt, x, y, a = max(flat_ones, key=lambda f: pref[f[3], f[2]])
        status = 'flat'
    else:           # otherwise the least uneven, flagged if still too uneven
        pv, tilt, x, y, a = min(feasible)
        status = 'uneven' if pv <= PV_MAX else 'flagged'
    box = np.int32(cv2.boxPoints(((float(x), float(y)), (float(LABEL_W), float(LABEL_H)), a)))
    return dict(x=x, y=y, angle=a, unevenness_mm=pv, tilt_deg=tilt, status=status,
                n_feasible=len(feasible), box=box.tolist())


# ── CLI ──────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('captures', nargs='+')
    ap.add_argument('--text', choices=['none', 'ppocr'], default='none')
    ap.add_argument('--no-edges', action='store_true', help='drop the old gradient-edge check')
    ap.add_argument('--no-depth', action='store_true', help='drop the depth term')
    ap.add_argument('--out', default='placement_out')
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    results = []
    for d in args.captures:
        color, depth = load_capture(d)
        vis = color.copy()
        for i, (m, (x0, y0, w, h)) in enumerate(segment_items_on_dark_background(color)):
            pad = 20
            ys, xs = slice(max(0, y0 - pad), y0 + h + pad), slice(max(0, x0 - pad), x0 + w + pad)
            crop, mk = color[ys, xs], m[ys, xs]
            t = time.time()
            text = text_mask_ppocr(crop) if args.text == 'ppocr' else None
            r = place_label(crop, mk, None if args.no_depth else depth[ys, xs], text, not args.no_edges)
            dt = time.time() - t
            name = '%s item%d@(%d,%d)' % (os.path.basename(os.path.normpath(d)), i, x0, y0)
            if r is None:
                print('%-32s no feasible placement  [%.0f ms]' % (name, 1000 * dt))
                results.append(dict(capture=d, item=i, bbox=[int(x0), int(y0), int(w), int(h)], result=None))
                continue
            box = np.int32(r['box']) + [xs.start, ys.start]
            col = {'flat': (0, 255, 0), 'uneven': (0, 200, 255), 'flagged': (0, 0, 255)}[r['status']]
            cv2.polylines(vis, [box], True, col, 3)
            if text is not None:
                cnts, _ = cv2.findContours(text, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(vis[ys, xs], cnts, -1, (255, 0, 255), 1)
            print('%-32s %-7s unevenness %.1f mm, tilt %.0f deg, angle %.0f, %d feasible  [%.0f ms]'
                  % (name, r['status'], r['unevenness_mm'], r['tilt_deg'], r['angle'], r['n_feasible'], 1000 * dt))
            r.update(x=int(r['x'] + xs.start), y=int(r['y'] + ys.start), box=box.tolist())
            results.append(dict(capture=d, item=i, bbox=[int(x0), int(y0), int(w), int(h)], result=r))
        cv2.imwrite(os.path.join(args.out, os.path.basename(os.path.normpath(d)) + '.jpg'), vis)
    with open(os.path.join(args.out, 'results.json'), 'w') as f:
        json.dump(results, f, indent=1, default=lambda o: o.item() if hasattr(o, 'item') else str(o))


if __name__ == '__main__':
    sys.exit(main())
