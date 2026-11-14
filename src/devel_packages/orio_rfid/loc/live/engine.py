#!/usr/bin/env python3
"""
engine.py - live item localiser: where each tag is, from the reads recorded so far, with ONE of the methods F1..F4.

    engine.py --follow <run>/reads_posed_live.csv [--method F4] [--post http://<pc>:8000] [--run LABEL]     robot or PC
    engine.py --runs-dir <receiver's runs folder> --post http://127.0.0.1:8000                               PC: newest run
    engine.py --once <reads_posed.csv> [--method F4] [--out est.csv]                                         a finished run

It is the over-time code of loc/offline/viz_beliefs_v2.py (the run-2 replay; method settings from final_apply.py),
restructured so that it keeps up with a run:
  * the read / no-read model is evaluated once per channel dwell for each (fold model, tag type) and shared by all
    tags, instead of once per tag (same numbers: only the reads of a tag differ from tag to tag)
  * evidence is added dwell by dwell as the reads arrive; nothing is recomputed from the start

Nothing above changes a number. Two shortcuts exist that do, and both are OFF unless asked for (team decision 2026-11-14:
no compromise on accuracy, the computing is done on the PC's GPU):
  * --rm-block N evaluates the read model once per block of N x N cells (5 cm each) instead of once per cell. Default 1.
  * only the chosen method is computed; tags that were just read are refreshed first, the others in turn
  * --cells shelf keeps only the grid cells in or near the shelf boxes (about 1 in 16), for F3 and F4. Default: all
    cells on a GPU, as offline. Without a GPU it is the default for F3 and F4, because all cells cannot keep up on a
    CPU; "not on any shelf" is then not reported. F1 and F2 have no shelf map and always use all cells.
Evidence = b * read/RSSI + k * phase per height + l * end-to-end net [+ f * reference fingerprint]; tags of one object
share evidence (F1-F3); F3 multiplies by the shelf boxes, F4 by the boxes weighted by where the other items sit.

A tag is placed once one pass of it has 30 reads (and 5 dwells), the rule of abuela's pipeline; a tag read less than
that is present but gets no position.

Differences from the offline result that are NOT numerical noise (see loc/README.md):
  * poses are the robot's live map poses, not the smoothed trajectory abuela builds after the run
  * the map -> room tie is fixed (site file, from run 2); offline it is refitted for every run
  * a tag that is not in models/*_folds.csv (a new item) gets a fold from its name, not from its location

Input: the posed read log (RFID/rfid_pose_live.py or rfid_pose_join.py). Output: POST /locations to sync/pc_receiver.py
(room frame, metres) and <run folder>/locations.json. Every round it also tells the receiver that it is alive and what
it is doing (POST /engine); the demo window shows that, and shows in red when it stops coming. Told to stop (SIGINT /
SIGTERM) it exits at once, without a last full pass. Needs torch, numpy, pandas, scipy.
"""
import argparse
import csv
import io
import json
import math
import os
import signal
import sys
import time
import urllib.request
import zlib

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as Fn

HERE = os.path.dirname(os.path.realpath(__file__))
LOC = os.path.dirname(HERE)
C_LIGHT = 299792458.0
METHODS = dict(F1=dict(b=.03, k=.3, l=1.0, f=0.0, pk=1, pr='none', est='lmean'), F2=dict(b=.01, k=.3, l=.3, f=1.0, pk=1, pr='none', est='mode'),
               F3=dict(b=.01, k=.3, l=.3, f=.3, pk=1, pr='box', est='mode'), F4=dict(b=.01, k=.3, l=.3, f=0.0, pk=0, pr='kde', est='mode'))
EPS = dict(box=0.01, kde=0.001)
GATE, MINREADS = 3.0, 5                       # phase: a pass counts within 3 dB of the tag's best; a belief needs 5 read dwells
MIN_PASS_READS = 30                           # ... and one pass with 30 reads. abuela's pipeline only takes such tags in (loc_v1.py:
                                              # visits(min_reads=30)); the scores are for them. On run 2 the 14 tags below it had 9 to 54
                                              # reads in all and were placed 3.9 m off in the median: not placing them is the honest answer
FP_TS, FP_SXY, FP_SZ = 0.02, 0.3, 0.5         # reference fingerprint (fp_term.py)
LMAX, HALF, EGAP, MINREAD, K, NF = 160, 15.0, 8.0, 8, 4, 19      # end-to-end net inputs (e2e.py)


class Net(nn.Module):
    """the end-to-end localiser of e2e.py: dwell tokens of one pass -> mixture of 4 Gaussians in the antenna's frame"""

    def __init__(s, dev, d=96, layers=3):
        super().__init__(); s.dev = dev
        s.inp = nn.Sequential(nn.Linear(NF, d), nn.SiLU(), nn.Linear(d, d))
        s.enc = nn.TransformerEncoder(nn.TransformerEncoderLayer(d, 4, 2 * d, dropout=0.1, batch_first=True, norm_first=True), layers, enable_nested_tensor=False)
        s.q = nn.Parameter(torch.randn(1, 1, d) * 0.1); s.pool = nn.MultiheadAttention(d, 4, batch_first=True)
        s.out = nn.Sequential(nn.Linear(d, d), nn.SiLU(), nn.Linear(d, K * 7))

    def forward(s, X, M):
        h = s.enc(s.inp(X), src_key_padding_mask=M); p, _ = s.pool(s.q.expand(len(X), -1, -1), h, h, key_padding_mask=M); o = s.out(p[:, 0]).view(len(X), K, 7)
        return torch.log_softmax(o[..., 0], -1), o[..., 1:4] + torch.tensor([1.0, 0.0, 0.6], device=s.dev), o[..., 4:7].clamp(-3.5, 1.0)


def rot(v, ang):
    c, s = np.cos(ang), np.sin(ang); return np.stack([c * v[..., 0] - s * v[..., 1], s * v[..., 0] + c * v[..., 1]], -1)


class Tag:
    def __init__(s, name, epc, ttype, fold):
        s.name, s.epc, s.type, s.fold = name, epc, ttype, fold
        s.POS = s.S0 = s.S1 = s.S2 = s.SL = None           # read-model sums over this tag's READ dwells (non-reads are shared)
        s.gid, s.n, s.rssi, s.dr, s.da = [], [], [], [], []      # one entry per global dwell the tag was read in
        s.reads = 0
        s.visits, s.vbest = [], -99.0
        s.e2, s.sig = None, None
        s.dirty, s.est_at, s.est = False, 0.0, None
        s.dirty_since = 0.0        # robot-clock time of the oldest read that is not in the published position yet


class Engine:
    def __init__(s, method='F4', device='auto', cells='auto', site='live_site_v2.json', log=print, threads=4, rm_block=1):
        s.method, s.c, s.log = method, METHODS[method], log
        s.dev = dev = ('cuda' if torch.cuda.is_available() else 'cpu') if device == 'auto' else device
        site = json.load(open(os.path.join(LOC, 'site', site))); s.site = site
        gt_path = os.path.join(LOC, 'site', site['ground_truth'])
        os.environ.update(RFID_SITE=os.path.join(LOC, 'site', site['shelves']), RFID_GT=gt_path, RFID_SHELFGT=gt_path)
        sys.path.insert(0, os.path.join(LOC, 'offline'))
        import relmodel as RM, eval_v1 as EV, shelfprior_lib as SPL      # the validated pieces, unchanged
        import pandas as pd
        s.RM, s.EV, s.SPL = RM, EV, SPL
        torch.set_num_threads(threads)
        s.R, s.tm = np.array(site['map_to_room']['R']), np.array(site['map_to_room']['t'])
        s.OFF = site['antenna_offset']; s.hop = np.array(site['hop_table_mhz']) * 1e6
        g = site['grid']; s.STEP, s.ZSTEP = g['step'], g['zstep']
        xs, ys, zs = np.arange(g['lo'][0], g['hi'][0], s.STEP), np.arange(g['lo'][1], g['hi'][1], s.STEP), np.arange(0.0, g['zmax'] + 1e-4, s.ZSTEP)
        s.shape = (len(xs), len(ys), len(zs)); s.NC = len(xs) * len(ys) * len(zs); s.CELL = s.STEP * s.STEP * s.ZSTEP
        Groom = np.stack(np.meshgrid(xs, ys, zs, indexing='ij'), -1).reshape(-1, 3)
        Gmap = np.c_[(Groom[:, :2] - s.tm) @ s.R, Groom[:, 2]]
        if cells == 'auto':
            cells = 'all' if dev == 'cuda' or s.c['pr'] == 'none' else 'shelf'
        if cells == 'shelf' and s.c['pr'] == 'none':
            raise SystemExit('%s has no shelf map: it cannot run with --cells shelf' % method)
        # every cell any tag's shelf prior can reach: the deepest boxes, 15 cm deeper (about 1 cell in 16)
        boxes = np.flatnonzero(SPL.shelf_of(Gmap, s.R, s.tm, {k: v + 0.15 for k, v in SPL.map_depths(()).items()}) > 0) if s.c['pr'] != 'none' else None
        sel = boxes if cells == 'shelf' else np.arange(s.NC)
        s.cells, s.sel, s.N = cells, sel, len(sel)
        s.cand = None if boxes is None else np.arange(s.N) if cells == 'shelf' else boxes      # the box cells, as indices into the cells in use
        s.cand_t = None if boxes is None else torch.tensor(s.cand, device=dev)
        s.Gmap, s.Groom = Gmap[sel], Groom[sel]
        s.G = torch.tensor(s.Gmap, device=dev, dtype=torch.float32); s.Gr = torch.tensor(s.Groom, device=dev, dtype=torch.float64)
        s.sel_t = torch.tensor(sel, device=dev)
        ix, iy, iz = np.unravel_index(sel, s.shape)             # where each cell in use sits in the full grid
        s.ix, s.iy, s.iz = (torch.tensor(v, device=dev) for v in (ix, iy, iz))
        # read model: one evaluation per block of rm_block x rm_block cells in x, y (same height), at the block's centre
        _, inv = np.unique(np.stack([ix // rm_block, iy // rm_block, iz], 1), axis=0, return_inverse=True); inv = inv.ravel()
        cnt = np.bincount(inv); s.Nc = len(cnt)
        s.Gc = torch.tensor(np.stack([np.bincount(inv, weights=s.Gmap[:, k]) / cnt for k in range(3)], 1), device=dev, dtype=torch.float32)
        s.rm_inv = torch.tensor(inv, device=dev)
        # models
        mdir = os.path.join(LOC, 'models')
        folds = pd.read_csv(os.path.join(mdir, site['read_model'] + '_folds.csv'), index_col=0).iloc[:, 0]
        s.folds = {k: int(v) for k, v in folds.items()}; s.nfold = int(folds.max()) + 1
        s.rel, s.nets = {}, {}
        for k in range(s.nfold):
            m = RM.Rel().to(dev); m.load_state_dict(torch.load(os.path.join(mdir, '%s_fold%d.pt' % (site['read_model'], k)), map_location=dev)); s.rel[k] = m.eval()
            ck = torch.load(os.path.join(mdir, '%s_fold%d.pt' % (site['e2e_model'], k)), map_location=dev)
            n = Net(dev).to(dev); n.load_state_dict(ck['state']); s.nets[k] = (n.eval(), float(ck['tau']))
        s.TM = json.load(open(os.path.join(mdir, 'final_T.json')))[method]
        s.CG = torch.tensor(RM.CGRID, device=dev)
        s.NEG = {(k, t): torch.zeros(len(RM.CGRID), s.Nc, device=dev) for k in range(s.nfold) for t in range(len(RM.TYPES))}
        # ground truth: only for the shelf map (boxes, where the other items sit) and the corner tags' positions
        gt = SPL.SHELF_GT; s.gt = gt
        s.OBJ = gt.object.where(gt.object.notna(), gt.index.to_series())
        s.TEST = set(pd.read_csv(os.path.join(LOC, 'site', site['testset'])).name)
        s.refs = [n for n in gt.index if n.startswith('Ref ')]
        Pr = gt.loc[s.refs, ['X_room_m', 'Y_room_m', 'Z_room_m']].values.astype(float)
        Pm = np.c_[(Pr[:, :2] - s.tm) @ s.R, Pr[:, 2]]
        if s.c['f']:
            Pa = torch.tensor(Pm, device=dev, dtype=torch.float32)
            s.EA = torch.stack([torch.exp(-0.5 * (((s.G[:, :2] - p[:2]) ** 2).sum(1) / FP_SXY ** 2 + (s.G[:, 2] - p[2]) ** 2 / FP_SZ ** 2)) for p in Pa])
        s.priors = {}

        def gk(sig):
            r = int(4 * sig + 0.5); x = torch.arange(-r, r + 1, device=dev, dtype=torch.float64); k = torch.exp(-0.5 * (x / sig) ** 2); return (k / k.sum()), r
        (s.KX, s.RX), (s.KZ, s.RZ) = gk(0.2 / s.STEP), gk(0.2 / s.ZSTEP)
        # run state
        s.clk, s.raw, s.pend, s.open = None, [], [], []
        s.pt, s.px, s.py, s.pyaw, s.plift = [], [], [], [], []       # pose track at the log's host times
        s.Dt, s.DA, s.Dth = [], [], []                               # global dwells: mid time, antenna xyz, base yaw
        s.tags, s.t_max, s.n_reads, s.queue = {}, 0.0, 0, []
        s.t_proc, s.dirty_left = 0.0, 0                              # robot-clock time of the newest read used so far; objects read but not yet re-estimated
        s.waited = []                                                # for the tags just re-estimated: time of their oldest read that was still waiting
        s.cost = {k: 0.0 for k in ('read_model', 'phase', 'e2e', 'fingerprint', 'prior', 'estimate')}    # seconds spent per stage so far
        s._refV = None
        log('localiser: %s on %s, %d of %d cells (%s), read model on %d blocks, region temperature %s' % (method, dev, s.N, s.NC, cells, s.Nc, s.TM))

    # ------------------------------------------------------------------ input
    def ingest(s, r):
        """one row of the posed read log (dict of strings)"""
        try:
            host = float(r['host_time_s']); bx, by, yaw, lift = float(r['base_x']), float(r['base_y']), float(r['base_yaw']), float(r['lift_m'])
        except (ValueError, KeyError):
            return
        if not s.pt or host > s.pt[-1]:
            if s.pyaw:
                yaw = s.pyaw[-1] + (yaw - s.pyaw[-1] + math.pi) % (2 * math.pi) - math.pi      # unwrapped
            s.pt.append(host); s.px.append(bx); s.py.append(by); s.pyaw.append(yaw); s.plift.append(lift)
        s.t_max = max(s.t_max, host)
        name = r.get('name') or ''
        if not name:
            return                                     # an EPC that is not in the tag registry
        try:
            s.raw.append((host, float(r['reader_us']) / 1e6, name, r['epc'], r.get('tag_type') or 'unknown', int(r['channel']),
                          float(r['phase_rad']), float(r['rssi_dbm'])))
        except (ValueError, KeyError):
            pass

    def tag(s, name, epc, ttype):
        t = s.tags.get(name)
        if t is None:
            if ttype not in s.RM.TYPES:                # as adapt_v2.py: unknown -> metal for corner tags, regular for items
                ttype = 'metal' if name.startswith('Ref ') else 'regular'
            fold = s.folds.get(name)
            if fold is None:                           # a tag the models have never seen: any fold is as good, keep it stable
                fold = zlib.crc32(name.encode()) % s.nfold
            t = s.tags[name] = Tag(name, epc, s.RM.TYPES.index(ttype), fold)
            z = lambda: torch.zeros(s.Nc, device=s.dev)
            t.POS, t.S0, t.S1, t.S2, t.SL = z(), z(), z(), z(), z()
        return t

    def antenna(s, t):
        """antenna point and base yaw at robot-clock times t (array), from the pose track"""
        pt = np.asarray(s.pt); th = np.interp(t, pt, s.pyaw); c, sn = np.cos(th), np.sin(th)
        return np.c_[np.interp(t, pt, s.px) + s.OFF[0] * c - s.OFF[1] * sn, np.interp(t, pt, s.py) + s.OFF[0] * sn + s.OFF[1] * c,
                     s.OFF[2] + np.interp(t, pt, s.plift)], th

    # ------------------------------------------------------------------ evidence, dwell by dwell
    @torch.no_grad()
    def step(s, final=False):
        """use every read that is old enough to be complete (0.4 s: the pose join needs up to 0.25 s), or everything when final"""
        if s.clk is None:                    # host clock - reader clock, from the first 1000 reads (about 2 s) and no others: the
            if len(s.raw) < 1000 and not final or not s.raw:     # result must not depend on how long a round happens to be. (With
                return 0                                         # "whatever arrived in the first round" 14 of 132 items moved by up
            s.clk = float(np.median([h - rd for h, rd, *_ in s.raw[:1000]]))       # to 4 cm between 1 s and 2 s rounds.)
        s.pend += [(rd + s.clk, *x) for _, rd, *x in s.raw]; s.raw = []       # (t, name, epc, type, channel, phase, rssi)
        s.pend.sort(key=lambda x: x[0])
        safe = math.inf if final else s.t_max - 0.4
        k = next((i for i, x in enumerate(s.pend) if x[0] > safe), len(s.pend))
        new, s.pend = s.pend[:k], s.pend[k:]
        if not new and not final:
            return 0
        s.n_reads += len(new)
        if new:
            s.t_proc = max(s.t_proc, new[-1][0])
        A, _ = s.antenna(np.array([x[0] for x in new])) if new else (np.zeros((0, 3)), None)
        closed = []                                     # global dwells: new on a channel change or a gap > 0.25 s
        for x, a in zip(new, A):
            if s.open and (x[4] != s.open[-1][0][4] or x[0] - s.open[-1][0][0] > 0.25):
                closed.append(s.open); s.open = []
            s.open.append((x, a))
        if final and s.open:
            closed.append(s.open); s.open = []
        events, g0 = [], len(s.Dt)                      # (dwell index in this batch, tag, max rssi)
        for d, dw in enumerate(closed):
            t0, t1 = dw[0][0][0], dw[-1][0][0]; tm = (t0 + t1) / 2
            Am, th = s.antenna(np.array([tm])); s.Dt.append(tm); s.DA.append(Am[0]); s.Dth.append(float(th[0]))
            kf = 4 * np.pi * s.hop[dw[0][0][4] - 1] / C_LIGHT
            per = {}
            for (t, name, epc, ttype, chan, ph, rssi), a in dw:
                per.setdefault(name, []).append((t, ph, rssi, a, epc, ttype))
            for name, rs in per.items():
                tg = s.tag(name, rs[0][4], rs[0][5])
                php = np.array([r[1] for r in rs])
                dr = float(np.sum(np.angle(np.exp(1j * -(php[1:] - php[:-1]))) / kf)) if len(rs) > 1 else 0.0
                tg.gid.append(g0 + d); tg.n.append(len(rs)); tg.rssi.append(max(r[2] for r in rs)); tg.dr.append(dr); tg.da.append(rs[-1][3] - rs[0][3])
                tg.reads += len(rs)
                if not tg.dirty:
                    tg.dirty, tg.dirty_since = True, rs[0][0]
                events.append((d, tg, tg.rssi[-1]))
                for t, ph, rssi, a, *_ in rs:
                    s._phase_read(tg, t, dw[0][0][4], a, kf, -ph, rssi)
        now_t = new[-1][0] if new else s.t_max
        for tg in s.tags.values():                      # a tag's own dwell is complete 0.25 s after its last read
            v = tg.visits[-1] if tg.visits else None
            if v and v['cur'] and (final or now_t - v['last'] > 0.25):
                s._close_dwell(v)
        t0 = time.time()
        if closed:
            s._read_model(g0, events)
        t1 = time.time(); s._score_phase()
        s.cost['read_model'] += t1 - t0; s.cost['phase'] += time.time() - t1
        return len(closed)

    def _phase_read(s, tg, t, chan, a, kf, ph, rssi):
        v = tg.visits[-1] if tg.visits else None
        if v is None or t - v['last'] > 8.0:            # a new pass of this tag
            if v and v['cur']:
                s._close_dwell(v)
            v = dict(n=0, rmax=-99.0, acc=None, ng=0, cur=[], chan=None, last=t); tg.visits.append(v)
        elif v['cur'] and (chan != v['chan'] or t - v['last'] > 0.25):
            s._close_dwell(v)
        v['cur'].append((a, kf, ph)); v['chan'], v['last'] = chan, t
        v['n'] += 1; v['rmax'] = max(v['rmax'], rssi); tg.vbest = max(tg.vbest, rssi)

    def _close_dwell(s, v):
        if len(v['cur']) >= 2:
            s.queue.append((v, v['cur']))
        v['cur'] = []

    def _read_model(s, g0, events):
        """read / no-read / RSSI model for the new dwells: once per (fold model, tag type), shared by all tags"""
        RM, G = s.RM, s.Gc
        A = torch.tensor(np.c_[np.array(s.DA[g0:]), np.array(s.Dth[g0:])], device=s.dev, dtype=torch.float32)
        ch = max(1, int(0.6e6 // s.Nc))                  # dwells per batch: the model's hidden layers take 256 bytes per block and dwell
        for a0 in range(0, len(A), ch):
            a = A[a0:a0 + ch]; dx = G[None, :, 0] - a[:, None, 0]; dy = G[None, :, 1] - a[:, None, 1]; dz = G[None, :, 2] - a[:, None, 2]
            rh = torch.hypot(dx, dy); bo = torch.atan2(torch.cos(a[:, 3]), -torch.sin(a[:, 3]))[:, None]
            az = torch.remainder(torch.atan2(dy, dx) - bo + np.pi, 2 * np.pi) - np.pi
            ev = {}
            for d, tg, rssi in events:
                if a0 <= d < a0 + len(a):
                    ev.setdefault((tg.fold, tg.type), []).append((d - a0, tg, rssi))
            for tt in range(len(RM.TYPES)):
                X = RM.feats(rh, az, a[:, None, 2].expand_as(rh), torch.tensor(tt, device=s.dev), dz)
                for k in range(s.nfold):
                    lg, mu, sg = s.rel[k](X); neg = s.NEG[(k, tt)]
                    neg -= Fn.softplus(lg[None] + s.CG[:, None, None]).sum(1)          # all five nuisance offsets at once
                    for d, tg, rssi in ev.get((k, tt), ()):
                        w = 1.0 / sg[d] ** 2; res = rssi - mu[d]
                        tg.POS += lg[d]; tg.S0 += w; tg.S1 += w * res; tg.S2 += w * res ** 2; tg.SL += torch.log(sg[d])

    def _score_phase(s):
        """phase inside each completed channel dwell of a tag (sarlib.score_dwell), added to that pass's sum"""
        q, s.queue = s.queue, []
        for i0 in range(0, len(q), 48):
            part = q[i0:i0 + 48]; ng = len(part)
            A = torch.tensor(np.array([r[0] for _, cur in part for r in cur]), device=s.dev, dtype=torch.float32)
            kf = torch.tensor([r[1] for _, cur in part for r in cur], device=s.dev, dtype=torch.float32)
            ph = torch.tensor([r[2] for _, cur in part for r in cur], device=s.dev, dtype=torch.float32)
            gid = torch.tensor([j for j, (_, cur) in enumerate(part) for _ in cur], device=s.dev)
            n = torch.bincount(gid, minlength=ng).float(); S = torch.empty(s.N, ng, device=s.dev)
            for c0 in range(0, s.N, 4096):
                a = ph[None] - kf[None] * torch.cdist(s.G[c0:c0 + 4096], A)
                re = torch.zeros(a.shape[0], ng, device=s.dev).index_add_(1, gid, torch.cos(a))
                im = torch.zeros(a.shape[0], ng, device=s.dev).index_add_(1, gid, torch.sin(a))
                S[c0:c0 + 4096] = (re * re + im * im - n[None]) / n[None]       # mean pair cosine of the dwell x (n - 1)
            for j, (v, _) in enumerate(part):
                v['acc'] = S[:, j].clone() if v['acc'] is None else v['acc'] + S[:, j]
                v['ng'] += 1

    # ------------------------------------------------------------------ belief of one tag
    def _e2e(s, tg):
        """end-to-end net on the tag's passes so far -> log density on the cells, or None (viz_beliefs_v2.e2e_grid)"""
        ND = len(s.Dt)
        gid = np.array(tg.gid)
        if len(gid) < MINREAD:
            return None
        te = np.array(s.Dt); Am = np.array(s.DA); bore = np.array(s.Dth) + np.pi / 2
        a = dict(read=np.zeros(ND, bool), n=np.zeros(ND), rssi=np.zeros(ND), dr=np.zeros(ND), da=np.zeros((ND, 3)))
        a['read'][gid] = True; a['n'][gid] = tg.n; a['rssi'][gid] = tg.rssi; a['dr'][gid] = tg.dr; a['da'][gid] = np.array(tg.da)
        W = []
        for seg in np.split(gid, np.flatnonzero(np.diff(te[gid]) > EGAP) + 1):
            if len(seg) < MINREAD:
                continue
            ref = seg[np.argmax(a['rssi'][seg])]; lo, hi = np.searchsorted(te, te[ref] - HALF), min(np.searchsorted(te, te[ref] + HALF), ND); tok = np.arange(lo, hi)
            if len(tok) > LMAX:
                tok = np.sort(tok[np.argsort(np.abs(te[tok] - te[ref]))[:LMAX]])
            W.append((int(ref), tok))
        sig = (tuple((r, int(tk[0]), int(tk[-1])) for r, tk in W), len(gid))
        if sig == tg.sig:
            return tg.e2
        tg.sig = sig
        if not W:
            tg.e2 = None
            return None
        items = []
        for ref, tok in W:
            b = bore[ref]; rel = rot(Am[tok, :2] - Am[ref, :2], -b); da = a['da'][tok]; rd = a['read'][tok].astype(float); rs = np.where(rd > 0, a['rssi'][tok], 0.0)
            items.append(np.c_[rel / 3, Am[tok, 2] - 1.1, np.cos(bore[tok] - b), np.sin(bore[tok] - b), rot(da[:, :2], -b) * 20, da[:, 2] * 20,
                               a['dr'][tok] * 20, (a['n'][tok] >= 2), rd, np.log1p(a['n'][tok]) / 2, np.where(rd > 0, (rs - a['rssi'][ref]) / 10, 0),
                               np.where(rd > 0, (rs + 60) / 10, 0), (te[tok] - te[ref]) / HALF, tok == ref,
                               np.repeat(np.eye(3)[tg.type][None], len(tok), 0)].astype(np.float32))
        Lm = max(len(x) for x in items); X = np.zeros((len(items), Lm, NF), np.float32); M = np.ones((len(items), Lm), bool)
        for j, x in enumerate(items):
            X[j, :len(x)] = x; M[j, :len(x)] = False
        net, tau = s.nets[tg.fold]; G = s.G
        lw, mu, ls = net(torch.tensor(X, device=s.dev), torch.tensor(M, device=s.dev)); ls = ls + float(np.log(tau)); out = torch.zeros(s.N, device=s.dev)
        floor = torch.tensor(float(np.log(0.05 / 50.0)), device=s.dev)
        for j, (r, _) in enumerate(W):
            c, sn = float(np.cos(-bore[r])), float(np.sin(-bore[r])); dx, dy = G[:, 0] - float(Am[r, 0]), G[:, 1] - float(Am[r, 1])
            rel = torch.stack([c * dx - sn * dy, sn * dx + c * dy, G[:, 2]], -1)
            z = (rel[:, None] - mu[j][None]) / torch.exp(ls[j])[None]; lp = torch.logsumexp(lw[j][None] - 0.5 * (z ** 2).sum(-1) - ls[j].sum(-1)[None] - 2.7568, -1)
            out += torch.logaddexp(lp, floor)
        tg.e2 = out
        return tg.e2

    def _fp(s, tg):
        """reference fingerprint: the corner tags whose RSSI-over-dwells profile looks like this tag's pull it toward them"""
        ND = len(s.Dt); ker = torch.tensor([.25, .5, 1, .5, .25], device=s.dev).view(1, 1, 5)

        def profiles(tags):
            V = torch.zeros(len(tags), ND, device=s.dev)
            for i, t in enumerate(tags):
                if t is not None and t.gid:
                    V[i, torch.tensor(t.gid, device=s.dev)] = torch.tensor(t.rssi, device=s.dev, dtype=torch.float32) + 80
            V = Fn.conv1d(V[:, None], ker, padding=2)[:, 0]
            return V / V.norm(dim=1, keepdim=True).clamp(min=1e-9)
        if s._refV is None or s._refV.shape[1] != ND:       # the corner tags' profiles: the same for every tag until the next dwell
            s._refV = profiles([s.tags.get(n) for n in s.refs])
        sim = (profiles([tg])[0] @ s._refV.T) / FP_TS
        if tg.name in s.refs:
            sim[s.refs.index(tg.name)] = -1e9
        w = torch.softmax(sim, 0)
        return torch.log((w[:, None] * s.EA).sum(0).double() + 1e-300)

    def evidence(s, tg):
        """the weighted sum of the terms for this tag on the cells (max 0), or None while the tag has too few reads to be placed"""
        if len(tg.gid) < MINREADS or max((v['n'] for v in tg.visits), default=0) < MIN_PASS_READS:
            return None
        c, CG = s.c, s.CG
        LD = s.NEG[(tg.fold, tg.type)] + tg.POS[None] + len(tg.gid) * CG[:, None]
        b = (tg.S1 / tg.S0.clamp(min=1e-9)).clamp(-s.RM.BMAX, s.RM.BMAX)
        LP = (LD.max(0).values - 0.5 * (tg.S2 - 2 * b * tg.S1 + b * b * tg.S0) - tg.SL).double(); LP = (LP - LP.max())[s.rm_inv]
        PH = torch.zeros(s.N, device=s.dev)
        for v in tg.visits:
            if v['n'] >= 30 and v['ng'] >= 15 and v['rmax'] >= tg.vbest - GATE:
                PH += v['acc']
        t0 = time.time(); e2 = s._e2e(tg); E2 = (e2 - e2.max()).double() if e2 is not None else 0.0; s.cost['e2e'] += time.time() - t0
        v = c['b'] * LP + c['k'] * PH.double() + c['l'] * E2
        if c['f']:
            t0 = time.time(); v = v + c['f'] * s._fp(tg); s.cost['fingerprint'] += time.time() - t0
        return v - v.max()

    def prior(s, name):
        """shelf boxes / where the other items sit, with this tag's own object and the locked test items left out"""
        key = s.OBJ[name] if name in s.OBJ.index else None
        if key not in s.priors:
            ex = s.TEST | (set(s.OBJ.index[s.OBJ == key]) if key is not None else {name})
            Gc = s.Gmap[s.cand]                     # outside the box cells the prior is the same small constant for every tag
            sh = s.SPL.shelf_of(Gc, s.R, s.tm, s.SPL.map_depths(ex))
            w = (sh > 0).astype(float) if s.c['pr'] == 'box' else s.SPL.kde_weight(Gc, s.R, s.tm, sh, ex)
            eps = EPS[s.c['pr']]
            s.priors[key] = (torch.tensor(sh, device=s.dev), torch.tensor(eps / s.N + (1 - eps) * w / max(w.sum(), 1e-9), device=s.dev, dtype=torch.float64))
        return s.priors[key]

    def _lmean(s, p, centre, r):
        m = (s.Gr - centre).norm(dim=1) < r
        return ((p[m] @ s.Gr[m]) / p[m].sum()).cpu().numpy() if bool(m.any()) and float(p[m].sum()) > 0 else None

    @torch.no_grad()
    def estimate(s, names):
        """position estimates for the tags of ONE object (they share evidence in F1-F3) -> list of dicts"""
        c = s.c; tags = [s.tags[n] for n in names if n in s.tags]; t_in = time.time(); in0 = s.cost['e2e'] + s.cost['fingerprint'] + s.cost['prior']
        ev = {t.name: s.evidence(t) for t in tags}; out = []
        for t in tags:
            v = ev[t.name]; t.est_at = time.time()
            if t.dirty:
                t.dirty = False
                if v is not None:
                    s.waited.append(t.dirty_since)
            if v is None:
                continue
            if c['pk']:
                for m in tags:
                    if m is not t and ev[m.name] is not None:
                        v = v + ev[m.name]
            p = torch.softmax(v, 0); shelf = pshelf = plost = None
            if c['pr'] != 'none':
                t0 = time.time(); sh, PR = s.prior(t.name); s.cost['prior'] += time.time() - t0
                q = p * (EPS[c['pr']] / s.N); q[s.cand_t] = p[s.cand_t] * PR; Z = q.sum()
                plost = float((p * EPS[c['pr']] / s.N).sum() / Z) if s.cells == 'all' else None
                p = q / Z
                ms = torch.zeros(int(sh.max()) + 1, device=s.dev, dtype=torch.float64).index_add_(0, sh, p[s.cand_t])
                shelf = int(ms[1:].argmax()) + 1; pshelf = float(ms[shelf])
            i = int(p.argmax())
            if c['est'] == 'lmean':
                est = s._lmean(p, s.Gr[i], 0.3)
            else:                                           # dominant mode: where most probability is within about 20 cm
                big = p > p.max() * 1e-6                    # the box around everything that matters, one blur radius wider: outside it
                x0, x1 = max(int(s.ix[big].min()) - s.RX, 0), min(int(s.ix[big].max()) + s.RX + 1, s.shape[0])     # there is less than a
                y0, y1 = max(int(s.iy[big].min()) - s.RX, 0), min(int(s.iy[big].max()) + s.RX + 1, s.shape[1])     # millionth of the peak
                inb = torch.nonzero((s.ix >= x0) & (s.ix < x1) & (s.iy >= y0) & (s.iy < y1)).ravel(); bx, by, bz = s.ix[inb] - x0, s.iy[inb] - y0, s.iz[inb]
                q = torch.zeros(1, 1, x1 - x0, y1 - y0, s.shape[2], device=s.dev, dtype=torch.float64); q[0, 0, bx, by, bz] = p[inb]
                q = Fn.conv3d(q, s.KX.view(1, 1, -1, 1, 1), padding=(s.RX, 0, 0)); q = Fn.conv3d(q, s.KX.view(1, 1, 1, -1, 1), padding=(0, s.RX, 0))
                q = Fn.conv3d(q, s.KZ.view(1, 1, 1, 1, -1), padding=(0, 0, s.RZ))
                j = int(inb[q[0, 0, bx, by, bz].argmax()]); est = s._lmean(p, s.Gr[j], 0.2)
            if est is None:
                est = s.Gr[i].cpu().numpy()
            pf = torch.softmax(torch.log(p + 1e-300) / s.TM, 0); sp, _ = torch.sort(pf, descending=True)
            n90 = int(torch.searchsorted(torch.cumsum(sp, 0), torch.tensor(0.9, device=s.dev, dtype=sp.dtype))) + 1
            t.est = dict(rfid_uid=t.epc, item_name=t.name, shelf=shelf, shelf_prob=None if pshelf is None else round(pshelf, 3),
                         x_m=round(float(est[0]), 3), y_m=round(float(est[1]), 3), z_m=round(float(est[2]), 3),
                         region90_m=round((3 * n90 * s.CELL / (4 * math.pi)) ** (1 / 3), 3), n_reads=t.reads, updated_s=round(time.time(), 1),
                         p_lost=None if plost is None else round(plost, 4))
            out.append(t.est)
        s.cost['estimate'] += (time.time() - t_in) - (s.cost['e2e'] + s.cost['fingerprint'] + s.cost['prior'] - in0)
        return out

    def refresh(s, budget=None):
        """re-estimate objects: those just read first, then the longest-waiting; stop when the time budget is used"""
        objs = {}
        for n, t in s.tags.items():
            if not n.startswith('Ref '):
                objs.setdefault(s.OBJ[n] if n in s.OBJ.index else n, []).append(t)
        order = sorted(objs.values(), key=lambda ts: (not any(t.dirty for t in ts), min(t.est_at for t in ts)))
        t0, out, done = time.time(), [], 0
        for ts in order:
            out += s.estimate([t.name for t in ts]); done += 1
            if budget is not None and time.time() - t0 > budget:
                break
        s.dirty_left = sum(1 for ts in order if any(t.dirty for t in ts))
        return out, done, len(order)

    def all_estimates(s):
        return [t.est for t in s.tags.values() if t.est]


# ---------------------------------------------------------------------- running it
def post(url, run, method, items):
    for i in range(0, len(items), 100):
        data = json.dumps({'run': run, 'method': method, 'source': os.uname().nodename, 'items': items[i:i + 100]}).encode()
        req = urllib.request.Request(url.rstrip('/') + '/locations', data=data, headers={'Content-Type': 'application/json'})
        urllib.request.urlopen(req, timeout=5).read()


def beat(url, note, error=False):
    """tell the receiver (and through it the demo window) that the localiser is alive and what it is doing"""
    if not url:
        return
    try:
        data = json.dumps({'note': note, 'error': error, 'source': os.uname().nodename, 'pid': os.getpid()}).encode()
        urllib.request.urlopen(urllib.request.Request(url.rstrip('/') + '/engine', data=data, headers={'Content-Type': 'application/json'}), timeout=2).read()
    except Exception:
        pass


def follow(args, path, method, run, stop, newer=lambda: False):
    """process a log that is still being written, until stopped (or until newer() says another run has started)"""
    sys.path.insert(0, os.path.join(os.path.dirname(LOC), 'RFID'))
    from rfid_pose_live import Tail
    beat(args.post, 'starting on %s, %s' % (run, method))
    eng = Engine(method, args.device, args.cells, args.site, threads=args.threads, rm_block=args.rm_block); tail, head, backlog = Tail(path), None, []
    # one line per round, for measuring how far the positions are behind the reads (loc/live/latency.py reads it)
    rounds = open(os.path.join(os.path.dirname(os.path.abspath(path)), 'engine_rounds.csv'), 'w', buffering=1)
    rounds.write('t_published,newest_read_t,rows,waiting,evidence_s,estimate_s,objects_done,objects_total,objects_left_dirty,placed,'
                 'tags_updated,oldest_waiting_read_median_t,oldest_waiting_read_min_t,' + ','.join('cost_' + k for k in eng.cost) + '\n')
    out_json = os.path.join(os.path.dirname(os.path.abspath(path)), 'locations.json')
    last_rows, idle = time.time(), False

    def publish(items, final=False):
        if not items:
            return
        json.dump({'run': run, 'method': method, 'final': final, 'items': eng.all_estimates()}, open(out_json + '.tmp', 'w')); os.replace(out_json + '.tmp', out_json)
        if args.post:
            try:
                post(args.post, run, method, items)
            except Exception as e:
                print('%s  could not send %d locations to %s (%s); they are in %s' % (time.strftime('%H:%M:%S'), len(items), args.post, e, out_json), flush=True)

    while True:
        t0 = time.time(); n = 0
        backlog += tail.lines()
        take, backlog = backlog[:6000], backlog[6000:]         # catching up with a run that is already long: in rounds of
        for line in take:                                       # 6,000 reads, so every round stays short and reports in
            row = next(csv.reader(io.StringIO(line)))
            if head is None:
                head = row
                continue
            eng.ingest(dict(zip(head, row))); n += 1
        if n:
            last_rows, idle = time.time(), False
        if stop['now']:                      # told to stop: leave now (a full last pass would hold the GPU for seconds more)
            return beat(args.post, 'stopped')
        ending = newer() or (not n and not backlog and not idle and time.time() - last_rows > 15.0)
        nd = eng.step(final=ending); t1 = time.time()
        items, done, total = eng.refresh(None if ending else args.estimate_budget) if (nd or ending) else ([], 0, 0)
        publish(items, final=ending)
        if n or nd or ending:
            w = sorted(eng.waited); eng.waited = []
            rounds.write('%.3f,%.3f,%d,%d,%.3f,%.3f,%d,%d,%d,%d,%d,%.3f,%.3f,%s\n' % (
                time.time(), eng.t_proc, n, len(backlog), t1 - t0, time.time() - t1, done, total, eng.dirty_left,
                sum(1 for t in eng.tags.values() if t.est and not t.name.startswith('Ref ')), len(w), w[len(w) // 2] if w else 0, w[0] if w else 0,
                ','.join('%.2f' % v for v in eng.cost.values())))
        if n or ending:
            print('%s  %d reads, %d dwells, %d tags placed; this round: %d rows, evidence %.1f s, %d of %d objects re-estimated in %.1f s%s%s' % (
                time.strftime('%H:%M:%S'), eng.n_reads, len(eng.Dt), len(eng.all_estimates()), n, t1 - t0, done, total, time.time() - t1,
                '  (catching up: %d reads waiting)' % len(backlog) if backlog else '', '  (FINAL: no rows for 15 s)' if ending else ''), flush=True)
        beat(args.post, '%s, %s on %s: %d reads, %d items placed%s' % (run, method, eng.dev, eng.n_reads, sum(1 for t in eng.tags.values() if t.est and not t.name.startswith('Ref ')),
                                                                    ', catching up' if backlog else ', run over' if idle or ending else ''))
        if ending:
            idle = True                                  # final estimates are out; wake up again if rows resume
            if newer():
                return
        time.sleep(0.05 if backlog else max(0.05, args.every - (time.time() - t0)))


def once(args):
    """a finished run, as fast as it goes, in the same 2 s rounds as live -> final estimates (and the time it took)"""
    rows = list(csv.DictReader(l for l in open(args.once) if not l.startswith('#')))
    eng = Engine(args.method, args.device, args.cells, args.site, threads=args.threads, rm_block=args.rm_block)
    t_start, t0, nxt = time.time(), float(rows[0]['host_time_s']), float(rows[0]['host_time_s']) + args.every
    for r in rows:
        if float(r['host_time_s']) > nxt:
            eng.step(); nxt += args.every
        eng.ingest(r)
    eng.step(final=True); t_ev = time.time() - t_start
    items, _, total = eng.refresh(None); dur = float(rows[-1]['host_time_s']) - t0
    print('%d reads over %.0f s of run: evidence in %.0f s (%.2f x real time), %d objects estimated in %.0f s' % (
        eng.n_reads, dur, t_ev, t_ev / dur, total, time.time() - t_start - t_ev))
    if args.out:
        with open(args.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(items[0].keys())); w.writeheader(); w.writerows(items)
        print('wrote %s (%d tags, method %s)' % (args.out, len(items), args.method))
    if args.post:
        post(args.post, args.run or 'once', args.method, items)
    return items


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--follow', help='posed read log of a run in progress'); src.add_argument('--runs-dir', help="pc_receiver.py's runs folder: follow the newest run")
    src.add_argument('--once', help='posed read log of a finished run')
    ap.add_argument('--method', default='F4', choices=list(METHODS)); ap.add_argument('--run', default=None)
    ap.add_argument('--post', default='', help='address of sync/pc_receiver.py (empty: only locations.json is written)')
    ap.add_argument('--every', type=float, default=1.0, help='seconds between rounds (a round that takes longer simply runs on)')
    ap.add_argument('--estimate-budget', type=float, default=0.7, help='seconds per round for re-estimating the objects that were read; what does '
                    'not fit waits for the next round (those waiting longest go first). Short is better: measured on abuela with all cells, '
                    '0.7 s gave each item a new position every 8 to 11 s, 3 s every 16 s, because long rounds also delay the reads')
    ap.add_argument('--device', default='auto', choices=['auto', 'cpu', 'cuda']); ap.add_argument('--cells', default='auto', choices=['auto', 'all', 'shelf'])
    ap.add_argument('--out', default=None, help='--once: write the estimates to this CSV')
    ap.add_argument('--rm-block', type=int, default=1, help='read model once per block of N x N cells in x, y (1 = every cell, as offline; more is a shortcut)')
    ap.add_argument('--threads', type=int, default=4, help='CPU threads for the maths (on the robot: fewer leaves more for navigation)')
    ap.add_argument('--site', default='live_site_v2.json', help='site file in loc/site (map -> room tie, grid, scene)')
    args = ap.parse_args()
    stop = {'now': False}

    def quit_now(*a):        # told to stop: leave at once, in the middle of a round if need be. The process ending is what
        print('%s  stopped' % time.strftime('%H:%M:%S'), flush=True)       # frees the GPU, and locations.json is always whole
        os._exit(0)
    signal.signal(signal.SIGINT, quit_now); signal.signal(signal.SIGTERM, quit_now)
    try:
        if args.once:
            return once(args)
        if args.follow:
            return follow(args, args.follow, args.method, args.run or os.path.basename(os.path.dirname(os.path.abspath(args.follow))), stop)
        return newest_runs(args, stop)
    except RuntimeError as e:
        if 'out of memory' not in str(e):
            raise
        print('%s  OUT OF GPU MEMORY: %s' % (time.strftime('%H:%M:%S'), str(e).split('. ')[1] if '. ' in str(e) else e), flush=True)
        beat(args.post, 'OUT OF GPU MEMORY (is another program using the GPU?)', error=True)
        print('          is another localiser or another program using the GPU (nvidia-smi)? --cells shelf or --device cpu need far less.', flush=True)
        sys.exit(3)


def newest_runs(args, stop):
    """--runs-dir: follow whichever run the receiver stored last, with the method the robot asked for"""

    def newest():
        runs = [d for d in os.listdir(args.runs_dir) if os.path.exists(os.path.join(args.runs_dir, d, 'run.json'))] if os.path.isdir(args.runs_dir) else []
        return max(runs, key=lambda d: os.path.getmtime(os.path.join(args.runs_dir, d, 'run.json'))) if runs else None

    def being_written(run):
        """rows arrived in the last 30 s (or none yet: the run has only just started)"""
        try:
            return time.time() - os.path.getmtime(os.path.join(args.runs_dir, run, 'reads_posed_live.csv')) < 30.0
        except OSError:
            return True
    # A run that was already over when this program started is left alone: its positions are in the database, and going
    # through it again would look like a run that nobody started (2026-11-13). A run that is still coming in is picked
    # up from its beginning: that is the restart after a stop in the middle of a run.
    done = newest()
    if done is not None and being_written(done):
        done = None
    print('waiting for a run in %s ...%s' % (args.runs_dir, '' if done is None else '  (the last one, %s, is over and is not gone through again)' % done), flush=True)
    while not stop['now']:
        run = newest()
        if run is None or run == done:
            beat(args.post, 'ready, waiting for a run')
            time.sleep(1.5)
            continue
        meta = json.load(open(os.path.join(args.runs_dir, run, 'run.json')))
        method = meta.get('method') if meta.get('method') in METHODS else args.method
        print('%s  run %s, method %s' % (time.strftime('%H:%M:%S'), run, method), flush=True)
        follow(args, os.path.join(args.runs_dir, run, 'reads_posed_live.csv'), method, run, stop, newer=lambda: newest() != run)
        done = run


if __name__ == '__main__':
    main()
