"""Find the items in one label zone with the RGB-D segmentation model, and say which is on top.

The model is the grasp planner's (orio_grasping.rgbd_seg, RF-DETR with a height channel),
trained on the Xtion's view of the bin. It also works on the ZED's view of a label zone
when it is given the zone's colour and its height above the table; the table is a plane
fitted to the zone's own depth, so no camera calibration is involved.

Two items lying on each other are sometimes returned as one mask. The model is
deterministic, so the zone is segmented six times (as it is, flipped both ways, and turned
by 90, 180 and 270 deg) and each result is turned back. The zone holds two or more items
when at least MIN_AGREE of the six say so. Measured on 104 hand-placed zones
(snap_label_zones.sh, test/segment_label_zones.py): 57 of 60 double zones found, against 51
with one pass, and none of 44 single items split.

The top item of a pile is the one that is higher along the line where the masks meet. An
item's height over its whole area does not say it: a bulky pouch lying under another can
be the taller of the two. When the two heights are within TOP_TIE_MM the top item is the
one whose bounding rectangle is least covered by the other items: an item lying on top
shows all of itself, the one under it has a piece of its rectangle hidden. Against the top
item marked by hand in 57 zones (test/mark_top_items.py): 57 right, 56 without the
tie-break, 50 by whole-item height.

Pure numpy + OpenCV: the model is passed in as `predict`, so this imports without torch
(the perception node uses the zone boxes; label_seg_node.py and the offline test run it).
"""
import cv2
import numpy as np

# The labelling crop of the ZED frame; zone 1 (A) is its left half, zone 2 (B) its right.
LBL_CROP_XYXY = (160, 180, 1860, 1050)

SCORE_MIN = 0.5        # lowest mask confidence kept (lower adds junk masks, not missed items)
MAX_MASK_FRAC = 0.5    # a bigger share of the zone is the table itself
MIN_MASK_PX = 3000     # smaller is a fragment (a 3 x 3 cm patch at this range is ~2200 px)
DUPLICATE_IOU = 0.5    # two masks of one pass overlapping more are one item
MIN_AGREE = 2          # passes that must find 2+ items for the zone to count as holding 2+
EDGE_PX = 12           # width of the band beside another mask used for the top-item test
TOP_TIE_MM = 1.0       # edge heights closer than this do not decide which item is on top
TABLE_TOL_M = 0.002    # depth within this of the fitted plane is table

# name, into the model, back to the zone
PASSES = (
    ("plain",  lambda a: a,                 lambda a: a),
    ("flip_h", lambda a: a[:, ::-1],        lambda a: a[:, ::-1]),
    ("flip_v", lambda a: a[::-1],           lambda a: a[::-1]),
    ("rot180", lambda a: a[::-1, ::-1],     lambda a: a[::-1, ::-1]),
    ("rot90",  lambda a: np.rot90(a, 1),    lambda a: np.rot90(a, -1)),
    ("rot270", lambda a: np.rot90(a, -1),   lambda a: np.rot90(a, 1)),
)


def zone_boxes(shape, crop=LBL_CROP_XYXY):
    """{zone: (x1, y1, x2, y2)} in full-frame pixels, clipped to the frame."""
    h, w = shape[:2]
    x1, y1, x2, y2 = max(crop[0], 0), max(crop[1], 0), min(crop[2], w), min(crop[3], h)
    mid = x1 + (x2 - x1) // 2
    return {1: (x1, y1, mid, y2), 2: (mid, y1, x2, y2)}


def fit_table(depth, tol=TABLE_TOL_M, iters=200, seed=0):
    """Plane depth = a*u + b*v + c through the table of one zone, by RANSAC on a sample of
    its valid pixels (the table is most of a zone). Returns ((a, b, c) or None, inlier
    fraction)."""
    v, u = np.nonzero(np.isfinite(depth))
    if len(u) < 100:
        return None, 0.0
    rng = np.random.default_rng(seed)
    pick = rng.choice(len(u), min(len(u), 20000), replace=False)
    A = np.stack([u[pick], v[pick], np.ones(len(pick))], axis=1).astype(np.float64)
    z = depth[v[pick], u[pick]].astype(np.float64)
    best, best_n = None, -1
    for _ in range(iters):
        i = rng.choice(len(z), 3, replace=False)
        try:
            p = np.linalg.solve(A[i], z[i])
        except np.linalg.LinAlgError:
            continue
        n = int((np.abs(A @ p - z) < tol).sum())
        if n > best_n:
            best, best_n = p, n
    if best is None:
        return None, 0.0
    inl = np.abs(A @ best - z) < tol
    p = np.linalg.lstsq(A[inl], z[inl], rcond=None)[0]
    return p, float(inl.mean())


def height_map(depth):
    """(height above the fitted table in metres, NaN where the depth is invalid; the share
    of the zone that is table). All NaN when no table could be fitted."""
    p, frac = fit_table(depth)
    if p is None:
        return np.full(depth.shape, np.nan, np.float32), 0.0
    vv, uu = np.mgrid[0:depth.shape[0], 0:depth.shape[1]]
    return (p[0] * uu + p[1] * vv + p[2] - depth).astype(np.float32), frac


def _keep_masks(masks, scores):
    """The masks of one pass that count as items, most confident first."""
    keep = []
    for i in np.argsort(-np.asarray(scores)):
        m = masks[i]
        n = int(m.sum())
        if scores[i] < SCORE_MIN or n < MIN_MASK_PX or n > MAX_MASK_FRAC * m.size:
            continue
        if any((m & masks[j]).sum() / float((m | masks[j]).sum()) > DUPLICATE_IOU for j in keep):
            continue
        keep.append(int(i))
    return [np.ascontiguousarray(masks[i]) for i in keep], [float(scores[i]) for i in keep]


def _describe(mask, height, others):
    """One item's numbers. `others` is the union of the other items' masks."""
    ys, xs = np.nonzero(mask)
    cx, cy = float(xs.mean()), float(ys.mean())
    # Where a cup would go: the centroid, or the item's pixel nearest it when the centroid
    # falls outside (a bent or ring-shaped mask).
    i = int(np.argmin((xs - cx) ** 2 + (ys - cy) ** 2))
    h = height[mask]
    h = h[np.isfinite(h)]
    near = mask & cv2.dilate(others.astype(np.uint8), np.ones((2 * EDGE_PX + 1,) * 2, np.uint8)).astype(bool)
    hn = height[near]
    hn = hn[np.isfinite(hn)]
    # How much of the item's bounding rectangle the other items cover: about nothing for an
    # item lying on top, the hidden part for one lying under another.
    rect = np.zeros(mask.shape, np.uint8)
    box = cv2.boxPoints(cv2.minAreaRect(np.stack([xs, ys], axis=1).astype(np.int32)))
    cv2.fillPoly(rect, [np.round(box).astype(np.int32)], 1)
    return {"centroid_px": [cx, cy], "pick_px": [int(xs[i]), int(ys[i])], "area_px": int(mask.sum()),
            "height_mm": float(np.median(h) * 1000) if h.size else None,
            "edge_height_mm": float(np.median(hn) * 1000) if hn.size >= 50 else None,
            "covered_px": int((rect.astype(bool) & others & ~mask).sum())}


def segment_zone(predict, rgb, depth):
    """The items in one zone.

    predict(rgb uint8 HxWx3, height float32 HxW in metres, no NaN) -> (masks (N, H, W) bool,
    scores (N,)), the raw output of the model on that input.
    rgb, depth: the zone's crop; depth in metres, NaN where invalid.

    Returns {"n_items", "items": [...], "top": index into items or None, "masks": [HxW
    bool, ...] in the order of items, "votes": {pass: items it found}, "used": the pass the
    masks come from, "table_frac"}. Items are ordered most confident first; each has score,
    centroid_px and pick_px (zone pixels), area_px, height_mm, edge_height_mm, covered_px
    and top.
    """
    height, table_frac = height_map(depth)
    h_in = np.nan_to_num(height, nan=0.0).astype(np.float32)
    runs = {}
    for name, fwd, inv in PASSES:
        masks, scores = predict(np.ascontiguousarray(fwd(rgb)), np.ascontiguousarray(fwd(h_in)))
        masks = [np.ascontiguousarray(inv(m)) for m in np.asarray(masks, bool)]
        runs[name] = _keep_masks(masks, scores)
    votes = {name: len(s) for name, (_m, s) in runs.items()}
    multi = [n for n in runs if votes[n] >= 2]
    if len(multi) >= MIN_AGREE:
        # The count most of them agree on, then the pass surest of its weakest mask.
        counts = [votes[n] for n in multi]
        mode = max(set(counts), key=lambda c: (counts.count(c), -c))
        used = max((n for n in multi if votes[n] == mode), key=lambda n: min(runs[n][1]))
    else:
        single = [n for n in runs if votes[n] == 1]
        used = max(single, key=lambda n: runs[n][1][0]) if single else None
    masks, scores = runs[used] if used else ([], [])

    items = []
    for k, m in enumerate(masks):
        others = np.zeros_like(m)
        for j, o in enumerate(masks):
            if j != k:
                others |= o
        items.append(dict(_describe(m, height, others), score=scores[k], top=False))
    top = None
    if items:
        def rank(it):   # height where it meets another item; alone, its own height
            v = it["edge_height_mm"] if it["edge_height_mm"] is not None else it["height_mm"]
            return -1e9 if v is None else v
        ranks = [rank(it) for it in items]
        top = int(np.argmax(ranks))
        tied = [k for k in range(len(items)) if ranks[top] - ranks[k] < TOP_TIE_MM]
        if len(tied) > 1:
            top = min(tied, key=lambda k: items[k]["covered_px"])
        items[top]["top"] = True
    return {"n_items": len(items), "items": items, "top": top, "masks": masks, "votes": votes,
            "used": used, "table_frac": round(table_frac, 3)}
