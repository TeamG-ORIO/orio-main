"""Shared pieces of the shelf-map mixture prior (shelfprior.py, fuse_v5.py, viewer). Shelf geometry from the site map only."""
import numpy as np, eval_v1 as E
import pandas as pd
import os as _os
SHELF_GT = pd.read_csv(_os.environ.get('RFID_SHELFGT', '/home/student/rfid_db/ground_truth/ground_truth_20261026_full.csv')).set_index('name')   # shelf map source: always the
# c1 measurements, so a new run's (moved) item positions never shape its own prior
BL = {s: float(SHELF_GT[(SHELF_GT.shelf == s) & SHELF_GT.index.str.startswith('Ref ') & SHELF_GT.index.str.endswith(' BL')].Z_room_m.iloc[0]) for s in E.SH}
def flat_phase(G, PH):
    """dwell-phase term maxed over z per column (bayes3d --flat-phase), NaN -> min"""
    ph = PH.astype(np.float64); nz = len(np.unique(G[:, 2]))
    if __import__('os').environ.get('RFID_NOFLAT') == '1':    # continuation: lift moves, keep the phase term per height cell
        return np.nan_to_num(ph, nan=np.nanmin(ph) if np.isfinite(ph).any() else 0.0)
    c = np.nanmax(np.where(np.isfinite(ph), ph, -np.inf).reshape(-1, nz), 1); ph = np.repeat(np.where(np.isfinite(c), c, np.nan), nz)
    return np.nan_to_num(ph, nan=np.nanmin(ph) if np.isfinite(ph).any() else 0.0)
import os
MARGIN = float(os.environ.get('RFID_MARGIN', 0.10))                                    # extra box depth for tape-measurement error (user, 2026-10-27)
def map_depths(exclude=(), margin=MARGIN):
    """per-shelf box depth = deepest measured item depth from the front plane (GT depth_m) on that shelf, excluding the tags in
    `exclude` (the scored tag itself and the locked test set), + margin. User-approved shelf-map input."""
    it = SHELF_GT[~SHELF_GT.index.str.startswith('Ref ') & ~SHELF_GT.index.isin(list(exclude))]
    return {s: float(it[it.shelf == s].depth_m.max()) + margin for s in E.SH}
def shelf_of(G, R, t, depth=0.61):
    """-> shelf id per map-frame cell (0 = outside every shelf box); depth: metres, or {shelf: metres}"""
    xy = G[:, :2] @ R.T + t; out = np.zeros(len(G), int)
    for s, g in E.SH.items():
        d = xy - g['a']; al = d @ g['u']; dp = -(d @ g['nrm'])
        ds = depth[s] if isinstance(depth, dict) else depth + .05
        m = (al > -.05) & (al < g['W'] + .05) & (dp > -.05) & (dp < ds) & (G[:, 2] > BL[s] - .10) & (G[:, 2] < BL[s] + g['H'] + .20)
        out[m] = s
    return out
def posterior(lik, eps, inn, n_in):
    pr = eps / len(lik) + (1 - eps) * inn / n_in; p = lik * pr; Z = p.sum()
    return p / Z, float((lik * eps / len(lik)).sum() / Z)
def summarize(name, G, lik, eps, inn, n_in, sh, it, R, t):
    p, plost = posterior(lik, eps, inn, n_in); i = int(p.argmax())
    order = np.argsort(-p); cs = np.cumsum(p[order]); rank = np.empty_like(order); rank[order] = np.arange(len(order))
    n, xy, z = E.truth(name); room = R @ G[i, :2] + t; e = room - xy; ez = G[i, 2] - z
    ms = np.bincount(sh, weights=p, minlength=max(E.SH) + 1); sp = int(np.argmax(ms[1:]) + 1)
    return dict(horiz=np.linalg.norm(e), along=abs(e @ E.SH[n]['u']), zabs=abs(ez), err3=np.hypot(np.linalg.norm(e), ez), hpd=cs[rank[it]],
                p_lost=plost, shelf_pred=sp, p_shelf=ms[sp], shelf_ok=sp == n, x=G[i, 0], y=G[i, 1], z=G[i, 2])
def kde_weight(G, R, t, sh, exclude=(), sd=0.05, sz=0.05):
    """per-cell weight inside shelf boxes from OTHER measured items on the same shelf: KDE over depth into the shelf x KDE over height
    (Gaussian, sd / sz m), each mixed 50/50 with uniform so an unusual depth or board is not ruled out. Cells outside boxes -> 0."""
    xy = G[:, :2] @ R.T + t; w = np.zeros(len(G))
    it = SHELF_GT[~SHELF_GT.index.str.startswith('Ref ') & ~SHELF_GT.index.isin(list(exclude))]
    for s, g in E.SH.items():
        m = sh == s
        if not m.any(): continue
        o = it[it.shelf == s]; od = -((o[['X_room_m', 'Y_room_m']].values - g['a']) @ g['nrm']); oz = o.Z_room_m.values
        dp = -((xy[m] - g['a']) @ g['nrm'])
        kd = np.exp(-0.5 * ((dp[:, None] - od[None]) / sd) ** 2).mean(1) if len(o) else np.ones(m.sum())
        kz = np.exp(-0.5 * ((G[m, 2][:, None] - oz[None]) / sz) ** 2).mean(1) if len(o) else np.ones(m.sum())
        w[m] = (0.5 * kd / max(kd.mean(), 1e-12) + 0.5) * (0.5 * kz / max(kz.mean(), 1e-12) + 0.5)
    return w
def shelf_of_measured(G, R, t, sm, margin=0.05):
    """shelf boxes from a TAPE-MEASURED site map (site_calib.py json: depth_m, top_m per shelf): depth [-5 cm, depth + margin],
    z [BL - 10 cm, top + margin]; along as shelf_of. -> shelf id per cell (0 = outside)"""
    xy = G[:, :2] @ R.T + t; out = np.zeros(len(G), int)
    for s, g in E.SH.items():
        d = xy - g['a']; al = d @ g['u']; dp = -(d @ g['nrm']); dep = sm['depth_m'].get(str(s), sm['depth_m'].get(s))
        top = sm.get('top_m', {}).get(str(s), sm.get('top_m', {}).get(s, BL[s] + g['H'] + 0.20 - margin))
        m = (al > -.05) & (al < g['W'] + .05) & (dp > -.05) & (dp < dep + margin) & (G[:, 2] > BL[s] - .10) & (G[:, 2] < top + margin)
        out[m] = s
    return out
