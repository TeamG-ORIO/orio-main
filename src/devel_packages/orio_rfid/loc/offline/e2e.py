"""End-to-end RFID localizer: reads -> tag position. A transformer over the channel dwells of a ~30 s window around one of
the tag's passes (read or not), every dwell expressed relative to the antenna at the tag's strongest read in that window
(no absolute coordinates, no shelf information). It outputs a 4-component Gaussian mixture over the tag position in that
reference frame; a tag's windows are fused in the map frame.
Training per location fold k (same 5 clusters as relmodel): simulated pretraining from the fold-k sensor model
(relmodel_<tag>_fold<k>.pt, never saw fold k) along the real trajectory, then fine-tuning on real windows of tags outside
fold k. Variants: sim+real, real only, sim only. Scored on fold-k tags against room XYZ ground truth.
v3 (2026-10-27): tag type (regular / metal / liquid) is an input (one-hot per token); locked test tags (testset_locked.csv) are
never used for fine-tuning or calibration; predicted positions are saved; each tag's out-of-fold fused log density is evaluated
on its v3 grid (grids3d_<tag>.pkl) for product-of-experts fusion with the grid posterior (e2e_<tag>_gridlp_v3.pkl).
Usage: e2e.py <tag> [steps_pre steps_fine]   (needs e2e_<tag>.pkl, dwells_<tag>.pkl, dwell_meta_<tag>.npz, relmodel folds)"""
import sys, zlib, pickle, time, numpy as np, pandas as pd, torch, torch.nn as nn, relmodel as RM
sys.path.insert(0, '/home/student/rfid_db/loc'); import eval_v1 as E
torch.set_num_threads(4); dev = 'cuda' if torch.cuda.is_available() else 'cpu'; C = 299792458.0
tag = sys.argv[1]; _num = [a for a in sys.argv[2:4] if a.isdigit()]; S_PRE = int(_num[0]) if len(_num) > 0 else 2500; S_FINE = int(_num[1]) if len(_num) > 1 else 1500
LMAX, HALF, GAP, MINREAD, K, NF = 160, 15.0, 8.0, 8, 4, 19
TEST = set(pd.read_csv('/home/student/rfid_db/loc/' + __import__('os').environ.get('RFID_TESTSET', 'testset_locked.csv')).name)
Dd = pickle.load(open(f'/home/student/rfid_db/loc/e2e_{tag}.pkl', 'rb'))
t, A0, A1, Am, bore = Dd['t'], Dd['A0'], Dd['A1'], Dd['Am'], Dd['bore']; ND = len(t); gt = Dd['gt']
meta = np.load(f'/home/student/rfid_db/loc/dwell_meta_{tag}.npz', allow_pickle=True); Rm, tm = meta['R'], meta['t']
TRAIN_ALL = '--train-all' in sys.argv; APPLY = sys.argv[sys.argv.index('--apply') + 1] if '--apply' in sys.argv else None
import os
fp = f'/home/student/rfid_db/loc/relmodel_{tag}_folds.csv'
folds = pd.read_csv(fp, index_col=0).iloc[:, 0] if os.path.exists(fp) else pd.Series(0, index=Dd['gt'].index)   # new runs: no folds needed
T = pd.read_pickle(f'/home/student/rfid_db/loc/dwells_{tag}.pkl'); T['fold'] = T.name.map(folds)
# dense real per-tag arrays over all dwells
REAL = {}
for name, g in Dd['F'].groupby('name'):
    if name not in gt.index: continue
    a = dict(read=np.zeros(ND, bool), n=np.zeros(ND), rssi=np.zeros(ND), dr=np.zeros(ND), da=np.zeros((ND, 3)))
    i = g.gid.values; a['read'][i] = True; a['n'][i] = g.n; a['rssi'][i] = g.rssi; a['dr'][i] = g.dr; a['da'][i] = g[['dax', 'day', 'daz']].values
    a['p'] = gt.loc[name].values.astype(float); a['type'] = RM.TYPES.index(Dd['types'].get(name, 'regular')); REAL[name] = a

THIN = '--thin' in sys.argv; EVAL_KEEP = float(sys.argv[sys.argv.index('--eval-keep') + 1]) if '--eval-keep' in sys.argv else 1.0
NEVAL = int(__import__('os').environ.get('RFID_E2E_NEVAL', 10 ** 9))
WTA = '--wta' in sys.argv   # 2026-11-10 mode-seeking variant: evolving winner-takes-all loss (Makansi et al. 2019) instead of the mixture NLL
AUG = '--aug' in sys.argv; SIMREAL = '--simreal' in sys.argv; FLOOR = -73.0                                      # reader RSSI floor (lowest read in c1)
OSFX = ('_wta' if WTA else '') + ('_thin' if THIN else '') + ('_aug' if AUG else '') + ('_simreal' if SIMREAL else '') + (f'_keep{EVAL_KEEP:g}' if EVAL_KEEP < 1 else '')     # outputs of the variants never overwrite v3
def aug_tag(a, rng):
    """per-window weaker/stronger tag: RSSI offset in [-6, +3] dB; reads that fall below the reader floor are dropped (weakest first)"""
    d = rng.uniform(-6.0, 3.0); rs = a['rssi'] + d; m = a['read'] & (rs >= FLOOR)
    return dict(a, rssi=np.where(m, rs, 0.0), read=m, n=np.where(m, a['n'], 0))
def aug_window(X, y, rng):
    """physically valid window augmentations (handover task 3), label fixed except under the mirror:
    - along-track mirror (= driving the other way; boresight stays left): y-features and target y negated (p = 0.5)
    - smooth trajectory drift: antenna positions shifted by v * (t - t_ref)/HALF, v ~ N(0, 3 cm) per horizontal axis, 1 cm in z
    - within-dwell phase noise: dwell range change + N(0, 3 mm) where a dwell has >= 2 reads"""
    X = X.copy(); y = y.copy()
    if rng.random() < 0.5: X[:, 1] *= -1; X[:, 4] *= -1; X[:, 6] *= -1; y[1] *= -1
    tau = X[:, 14:15]; X[:, 0:2] += tau * rng.normal(0, 0.03, 2) / 3; X[:, 2] += tau[:, 0] * rng.normal(0, 0.01)
    X[:, 8] += (X[:, 9] > 0) * rng.normal(0, 0.003, len(X)) * 20
    return X, y
def thin(a, rng, keep):
    """drop each read dwell with probability 1 - keep (weakest-first is not forced: random)"""
    m = a['read'] & (rng.random(len(a['read'])) < keep); return dict(a, read=m, n=np.where(m, a['n'], 0))
def windows(read, rssi, rng=None):
    idx = np.flatnonzero(read)
    if len(idx) < MINREAD: return []
    out = []
    for seg in np.split(idx, np.flatnonzero(np.diff(t[idx]) > GAP) + 1):
        if len(seg) < MINREAD: continue
        order = seg[np.argsort(-rssi[seg])]
        ref = order[rng.integers(min(3, len(order)))] if rng is not None else order[0]
        half = rng.uniform(3.0, HALF) if (THIN and rng is not None) else HALF
        lo, hi = np.searchsorted(t, t[ref] - half), np.searchsorted(t, t[ref] + half); tok = np.arange(lo, hi)
        if THIN and rng is not None and rng.random() < 0.25: tok = tok[t[tok] <= t[ref]]          # only the part of the pass before its peak
        if len(tok) > LMAX: tok = np.sort(tok[np.argsort(np.abs(t[tok] - t[ref]))[:LMAX]])
        if rng is not None and len(tok) > 20:                                   # token dropout
            keep = rng.random(len(tok)) > 0.1; keep[tok == ref] = True; tok = tok[keep]
        out.append((ref, tok))
    return out

def rot(v, ang):
    c, s = np.cos(ang), np.sin(ang); return np.stack([c * v[..., 0] - s * v[..., 1], s * v[..., 0] + c * v[..., 1]], -1)

def tokens(ref, tok, a):
    """a: dict with read, n, rssi, dr, da arrays (ND) -> (L, 16) features in the reference-antenna frame"""
    b = bore[ref]; rel = rot(Am[tok, :2] - Am[ref, :2], -b); da = a['da'][tok]; rd = a['read'][tok].astype(float)
    rs = np.where(rd > 0, a['rssi'][tok], 0.0)
    return np.c_[rel / 3, Am[tok, 2] - 1.1, np.cos(bore[tok] - b), np.sin(bore[tok] - b), rot(da[:, :2], -b) * 20, da[:, 2] * 20,
                 a['dr'][tok] * 20, (a['n'][tok] >= 2), rd, np.log1p(a['n'][tok]) / 2, np.where(rd > 0, (rs - a['rssi'][ref]) / 10, 0),
                 np.where(rd > 0, (rs + 60) / 10, 0), (t[tok] - t[ref]) / HALF, tok == ref,
                 np.repeat(np.eye(3)[a['type']][None], len(tok), 0)].astype(np.float32)

def target(ref, p):
    return np.r_[rot(p[:2] - Am[ref, :2], -bore[ref]), p[2]].astype(np.float32)

def batch(items):
    """items: list of (tokens (L,F), target (3,) or None) -> padded tensors"""
    L = max(len(x[0]) for x in items); X = np.zeros((len(items), L, NF), np.float32); M = np.ones((len(items), L), bool)
    for j, (x, _) in enumerate(items): X[j, :len(x)] = x; M[j, :len(x)] = False
    Y = np.stack([y for _, y in items]) if items[0][1] is not None else None
    return torch.tensor(X, device=dev), torch.tensor(M, device=dev), (torch.tensor(Y, device=dev) if Y is not None else None)

class Net(nn.Module):
    def __init__(s, d=96, layers=3):
        super().__init__()
        s.inp = nn.Sequential(nn.Linear(NF, d), nn.SiLU(), nn.Linear(d, d))
        s.enc = nn.TransformerEncoder(nn.TransformerEncoderLayer(d, 4, 2 * d, dropout=0.1, batch_first=True, norm_first=True), layers,
                                      enable_nested_tensor=False)
        s.q = nn.Parameter(torch.randn(1, 1, d) * 0.1); s.pool = nn.MultiheadAttention(d, 4, batch_first=True)
        s.out = nn.Sequential(nn.Linear(d, d), nn.SiLU(), nn.Linear(d, K * 7))
    def forward(s, X, M):
        h = s.enc(s.inp(X), src_key_padding_mask=M); p, _ = s.pool(s.q.expand(len(X), -1, -1), h, h, key_padding_mask=M)
        o = s.out(p[:, 0]).view(len(X), K, 7)
        mu = o[..., 1:4] + torch.tensor([1.0, 0.0, 0.6], device=dev)
        return torch.log_softmax(o[..., 0], -1), mu, o[..., 4:7].clamp(-3.5, 1.0)

def mix_logpdf(lw, mu, ls, Y):                      # Y: (B,3) or (B,N,3) -> (B,) or (B,N)
    Y = Y[:, None] if Y.dim() == 2 else Y[:, :, None]                                   # (B,1|N,1,3)
    mu_, ls_, lw_ = (mu[:, None], ls[:, None], lw[:, None]) if Y.dim() == 4 else (mu, ls, lw)
    z = (Y - mu_) / torch.exp(ls_); lp = -0.5 * (z ** 2).sum(-1) - ls_.sum(-1) - 2.7568
    return torch.logsumexp(lw_ + lp, -1).squeeze(1) if Y.dim() == 4 and Y.shape[1] == 1 else torch.logsumexp(lw_ + lp, -1)

def wta_loss(lw, mu, ls, Y, k):
    """evolving winner-takes-all: only the k components that explain the target best are trained on it (k: K -> 2 -> 1 over training), so the
    components settle on distinct modes instead of covering them; the mixture weights learn which component wins (cross-entropy)"""
    z = (Y[:, None] - mu) / torch.exp(ls); lp = -0.5 * (z ** 2).sum(-1) - ls.sum(-1) - 2.7568                     # (B, K)
    top = lp.topk(k, dim=1).values.mean(1); win = lp.argmax(1)
    return -top.mean() + nn.functional.nll_loss(lw, win)

# ---------- simulator (fold sensor model + dwell phase physics) ----------
def make_sim(k, rng):
    mp = f'/home/student/rfid_db/loc/relmodel_{tag}_all.pt' if k == 'all' else f'/home/student/rfid_db/loc/relmodel_{tag}_fold{k}.pt'
    m = RM.Rel().to(dev); m.load_state_dict(torch.load(mp)); m.eval()
    tr = T[(T.fold != k) & (T.read == 1)]; bins = [0, 1, 2, 4, 99]
    nb = [torch.tensor(tr[(tr.rh >= lo) & (tr.rh < hi)].n.values, device=dev, dtype=torch.float32) for lo, hi in zip(bins[:-1], bins[1:])]
    tf = pd.Series([REAL[n]['type'] for n in REAL if folds[n] != k]).value_counts(normalize=True).reindex(range(3), fill_value=0).values
    if SIMREAL:                                    # measured per-tag nuisances by type + real residuals (training-fold tags only)
        ta = T[(T.fold != k) & ~T.name.isin(TEST)]
        with torch.no_grad():
            X_ = RM.feats(*(torch.tensor(ta[c_].values, device=dev, dtype=torch.float32) for c_ in ('rh', 'az', 'ant_z')),
                          torch.tensor(ta.tag_type.map(RM.TYPES.index).values, device=dev), torch.tensor(ta.dz.values, device=dev, dtype=torch.float32))
            lg_, mu_, sg_ = (v.cpu().numpy() for v in m(X_))
        ta = ta.assign(p_=1 / (1 + np.exp(-lg_)), mu_=mu_, sg_=sg_); lgt = lambda q: np.log(np.clip(q, 1e-3, 1 - 1e-3) / (1 - np.clip(q, 1e-3, 1 - 1e-3)))
        per = ta.groupby('name').apply(lambda g: pd.Series(dict(ty=RM.TYPES.index(g.tag_type.iloc[0]), c=lgt(g.read.mean()) - lgt(g.p_.mean()),
                                                                b=(g.rssi - g.mu_)[g.read == 1].mean() if (g.read == 1).any() else 0.0)))
        rd_ = ta[ta.read == 1]; zres = ((rd_.rssi - rd_.mu_ - rd_.name.map(per.b)) / rd_.sg_).values; zres = zres[np.isfinite(zres)]
        CB = {ty: per[per.ty == ty][['c', 'b']].values for ty in range(3)}; CB = {ty: v if len(v) else per[['c', 'b']].values for ty, v in CB.items()}
        Zt = torch.tensor(np.clip(zres, -4, 4), device=dev, dtype=torch.float32)
    At0, At1, Atm = (torch.tensor(a, device=dev, dtype=torch.float32) for a in (A0, A1, Am)); Bt = torch.tensor(bore, device=dev, dtype=torch.float32)
    @torch.no_grad()
    def sim(B):
        i = rng.integers(ND, size=B); r = rng.uniform(0.2, 3.0, B); ang = bore[i] + rng.uniform(-np.pi, np.pi, B)
        p = np.c_[Am[i, 0] + r * np.cos(ang), Am[i, 1] + r * np.sin(ang), rng.uniform(0.0, 1.6, B)]
        ty = rng.choice(3, B, p=tf); P = torch.tensor(p, device=dev, dtype=torch.float32)
        dx, dy, dz = P[:, None, 0] - Atm[None, :, 0], P[:, None, 1] - Atm[None, :, 1], P[:, None, 2] - Atm[None, :, 2]
        rh = torch.hypot(dx, dy); az = torch.remainder(torch.atan2(dy, dx) - Bt[None] + np.pi, 2 * np.pi) - np.pi
        lg, mu, sg = m(RM.feats(rh, az, Atm[None, :, 2].expand_as(rh), torch.tensor(ty, device=dev)[:, None], dz))
        if SIMREAL:
            cb = np.stack([CB[t_][rng.integers(len(CB[t_]))] for t_ in ty]); c = torch.tensor(cb[:, :1], device=dev, dtype=torch.float32) + 0.3 * torch.randn(B, 1, device=dev)
            b = torch.tensor(cb[:, 1:], device=dev, dtype=torch.float32) + 1.0 * torch.randn(B, 1, device=dev)
        else:
            c = torch.randn(B, 1, device=dev); b = 3 * torch.randn(B, 1, device=dev)
        read = torch.rand_like(lg) < torch.sigmoid(lg + c)
        rssi = (mu + b + sg * (Zt[torch.randint(len(Zt), mu.shape, device=dev)] if SIMREAL else torch.randn_like(mu))).clamp(-75, -25)
        n = torch.zeros_like(lg)
        for j, (lo, hi) in enumerate(zip(bins[:-1], bins[1:])):
            sel = (rh >= lo) & (rh < hi); n[sel] = nb[j][torch.randint(len(nb[j]), (int(sel.sum()),), device=dev)]
        n = torch.where(read, n, torch.zeros_like(n))
        umin = 1 - torch.rand_like(n) ** (1 / n.clamp(min=1)); umax = torch.rand_like(n) ** (1 / n.clamp(min=1))
        af = At0[None] + umin[..., None] * (At1 - At0)[None]; al = At0[None] + umax[..., None] * (At1 - At0)[None]
        dr = torch.linalg.norm(P[:, None] - al, dim=-1) - torch.linalg.norm(P[:, None] - af, dim=-1) + 0.005 * torch.randn_like(n)
        two = (n >= 2).float(); da = (al - af) * two[..., None]
        out = []
        R_, N_, S_, DR_, DA_ = (x.cpu().numpy() for x in (read, n, rssi, dr * two, da))
        for j in range(B):
            a = dict(read=R_[j], n=N_[j], rssi=S_[j], dr=DR_[j], da=DA_[j], type=int(ty[j]))
            for ref, tok in windows(a['read'], a['rssi'], rng): out.append((tokens(ref, tok, a), target(ref, p[j])))
        return out
    return sim

def real_items(names, rng, per=1):
    out = []
    for nm in names:
        a = REAL[nm]
        for _ in range(per):
            if THIN: a = thin(REAL[nm], rng, rng.uniform(0.3, 1.0))
            if AUG and rng.random() < 0.5: a = aug_tag(REAL[nm], rng)
            for ref, tok in windows(a['read'], a['rssi'], rng):
                X, y = tokens(ref, tok, a), target(ref, a['p'])
                out.append(aug_window(X, y, rng) if AUG else (X, y))
    return out

def train(net, steps, source, lr, bs=192):
    opt = torch.optim.AdamW(net.parameters(), lr, weight_decay=1e-4); sch = torch.optim.lr_scheduler.OneCycleLR(opt, lr, total_steps=steps)
    net.train(); buf = []; hist = []
    for s in range(steps):
        while len(buf) < bs: buf += source()
        items, buf = buf[:bs], buf[bs:]
        X, M, Y = batch(items); lw, mu, ls = net(X, M)
        loss = wta_loss(lw, mu, ls, Y, K if s < steps / 3 else (2 if s < 2 * steps / 3 else 1)) if WTA else -mix_logpdf(lw, mu, ls, Y).mean()
        opt.zero_grad(); loss.backward(); nn.utils.clip_grad_norm_(net.parameters(), 1.0); opt.step(); sch.step(); hist.append(loss.item())
    net.eval(); return float(np.mean(hist[-100:]))

@torch.no_grad()
def predict(net, name, tau=1.0):
    """fuse a tag's windows in the map frame: candidates = all component means, robust log-sum over windows; plus HPD of truth"""
    a = REAL[name]
    if EVAL_KEEP < 1: a = thin(a, np.random.default_rng(zlib.crc32(name.encode())), EVAL_KEEP)    # fixed thinning of the scored tag
    W = [w for w in windows(a['read'], a['rssi']) if w[0] < NEVAL]      # multi-recording training: score from the first recording's windows only
    if not W: return None
    X, M, _ = batch([(tokens(r, tk, a), None) for r, tk in W]); lw, mu, ls = net(X, M); ls = ls + float(np.log(tau))
    refs = np.array([r for r, _ in W])
    def score(Pm):                                                          # Pm (N,3) map-frame points -> (N,) fused log density
        rel = np.stack([np.c_[rot(Pm[:, :2] - Am[r, :2], -bore[r]), Pm[:, 2]] for r in refs]).astype(np.float32)   # (W,N,3)
        lp = mix_logpdf(lw, mu, ls, torch.tensor(rel, device=dev))                                                     # (W,N)
        return torch.logaddexp(lp, torch.tensor(np.log(0.05 / 50.0), device=dev)).sum(0).cpu().numpy()
    mu_n = mu.cpu().numpy(); cand = np.concatenate([np.c_[Am[r, :2] + rot(m[:, :2], bore[r]), m[:, 2]] for r, m in zip(refs, mu_n)])
    best = cand[np.argmax(score(cand))]
    gx = np.arange(-2.0, 2.0001, 0.05); gz = np.arange(0.0, 1.6001, 0.1)
    G = np.stack(np.meshgrid(best[0] + gx, best[1] + gx, gz, indexing='ij'), -1).reshape(-1, 3)
    sg = score(G); sg -= sg.max(); pg = np.exp(sg); pg /= pg.sum(); i = int(pg.argmax()); est = G[i]
    order = np.argsort(-pg); cs = np.cumsum(pg[order]); rank = np.empty_like(order); rank[order] = np.arange(len(order))
    it = int(np.argmin(np.linalg.norm(G - a['p'], axis=1))); inside = np.linalg.norm(G[it] - a['p']) < 0.1
    return est, (cs[rank[it]] if inside else 1.0), len(W), score

def score_tags(net, names, variant, k, tau=1.0):
    rows = []
    for nm in names:
        r = predict(net, nm, tau)
        if r is None: continue
        est, hpd, nw, _ = r; n, xy, z = E.truth(nm); room = Rm @ est[:2] + tm; e = room - xy
        rows.append(dict(variant=variant, fold=k, name=nm, ref=nm.startswith('Ref '), windows=nw, horiz=np.linalg.norm(e),
                         along=abs(e @ E.SH[n]['u']), zabs=abs(est[2] - z), err3=np.hypot(np.linalg.norm(e), est[2] - z), hpd=hpd,
                         x=est[0], y=est[1], z=est[2], tau=tau, test=nm in TEST))
    return rows

def calibrate(net, names):
    """spread factor tau chosen on TRAINING-fold tags only: 90% regions should hold the truth 90% of the time"""
    best = None
    for tau in (1.0, 1.5, 2.0, 3.0, 4.0, 6.0):
        h = np.array([r[1] for r in (predict(net, nm, tau) for nm in names) if r is not None])
        c = abs(np.mean(h <= 0.9) - 0.9)
        if best is None or c < best[0]: best = (c, tau)
    return best[1]

MIXED = '--mixed' in sys.argv
GR3 = pickle.load(open(f'/home/student/rfid_db/loc/grids3d_{tag}.pkl', 'rb')); GLP = {}
if TRAIN_ALL:                                       # second freeze: one model on ALL c1 tags (the new run is the test), saved with its tau
    rng = np.random.default_rng(0); names = list(REAL); sim = make_sim('all', rng)
    torch.manual_seed(0); net = Net().to(dev); train(net, S_PRE, lambda: sim(256), 1e-3)
    def mixed_all():
        it = real_items(rng.choice(names, 8), rng); sm = sim(64); return it + sm[:max(3 * len(it), 1)]
    train(net, S_FINE, mixed_all, 1e-4); tau = calibrate(net, names)
    torch.save(dict(state=net.state_dict(), tau=tau), f'/home/student/rfid_db/loc/e2e_{tag}_all.pt'); print('saved', f'e2e_{tag}_all.pt', 'tau', tau); sys.exit()
if APPLY:                                           # apply a saved model to a new run: density on that run's grids, no training
    ck = torch.load(APPLY); net = Net().to(dev); net.load_state_dict(ck['state']); net.eval()
    for nm in REAL:
        if nm in GR3:
            r = predict(net, nm, ck['tau'])
            if r is not None:
                G3 = GR3[nm]['G'].astype(np.float64); GLP[nm] = np.concatenate([r[3](G3[i0:i0 + 20000]) for i0 in range(0, len(G3), 20000)]).astype(np.float32)
    pickle.dump(GLP, open(f'/home/student/rfid_db/loc/e2e_{tag}_gridlp_v3.pkl', 'wb')); print('applied', APPLY, 'to', len(GLP), 'tags'); sys.exit()
if '--applyfolds' in sys.argv:                  # apply another tag's per-fold nets to this run: every tag gets the net that never saw its location
    src = sys.argv[sys.argv.index('--applyfolds') + 1]; fs = folds; nets = {}     # folds of THIS tag (make_folds_like.py: same location folds as src)
    for k in sorted(fs.unique()):
        ck = torch.load(f'/home/student/rfid_db/loc/e2e_{src}{OSFX}_fold{k}.pt'); n_ = Net().to(dev); n_.load_state_dict(ck['state']); nets[k] = (n_.eval(), ck['tau'])
    for nm in REAL:
        if nm in GR3 and nm in fs.index:
            r = predict(nets[fs[nm]][0], nm, nets[fs[nm]][1])
            if r is not None:
                G3 = GR3[nm]['G'].astype(np.float64); GLP[nm] = np.concatenate([r[3](G3[i0:i0 + 20000]) for i0 in range(0, len(G3), 20000)]).astype(np.float32)
    pickle.dump(GLP, open(f'/home/student/rfid_db/loc/e2e_{tag}_gridlp_v3{OSFX}.pkl', 'wb')); print('applied fold nets of', src, 'to', len(GLP), 'tags'); sys.exit()
rows = []; t0 = time.time()
for k in sorted(folds.unique()):
    rng = np.random.default_rng(k); train_names = [n for n in REAL if folds[n] != k and n not in TEST]; test_names = [n for n in REAL if folds[n] == k]
    sim = make_sim(k, rng)
    torch.manual_seed(k); net = Net().to(dev); lpre = train(net, S_PRE, lambda: sim(256), 1e-3)
    tau = calibrate(net, train_names); rows += score_tags(net, test_names, 'sim only', k) + score_tags(net, test_names, f'sim only, calibrated', k, tau)
    def mixed():
        it = real_items(rng.choice(train_names, 8), rng); sm = sim(64); return it + sm[:max(3 * len(it), 1)]
    lfine = train(net, S_FINE, mixed, 1e-4)
    tau2 = calibrate(net, train_names)
    torch.save(dict(state=net.state_dict(), tau=tau2), f'/home/student/rfid_db/loc/e2e_{tag}{OSFX}_fold{k}.pt')     # fold nets kept (2026-11-10) for --applyfolds
    rows += score_tags(net, test_names, 'sim + mixed real', k) + score_tags(net, test_names, 'sim + mixed real, calibrated', k, tau2)
    for nm in test_names:                                                     # out-of-fold fused log density on the v3 grid
        if nm in GR3:
            r = predict(net, nm, tau2)
            if r is not None:
                G3 = GR3[nm]['G'].astype(np.float64); GLP[nm] = np.concatenate([r[3](G3[i0:i0 + 20000]) for i0 in range(0, len(G3), 20000)]).astype(np.float32)
    print(f'fold {k}: {len(train_names)} train / {len(test_names)} test tags; NLL pre {lpre:.2f} mixed {lfine:.2f}; tau {tau} / {tau2}; '
          f'{(time.time() - t0) / 60:.1f} min', flush=True)
Rw = pd.DataFrame(rows); Rw.to_csv(f'/home/student/rfid_db/loc/e2e_{tag}_results_v3{OSFX}.csv', index=False)
pickle.dump(GLP, open(f'/home/student/rfid_db/loc/e2e_{tag}_gridlp_v3{OSFX}.pkl', 'wb'))
Rw = Rw[~Rw.test]                                                              # summary on dev tags only (locked test set stays hidden)
for v, g in Rw.groupby('variant'):
    for nm, d in (('items', g[~g.ref]), ('corners', g[g.ref])):
        print(f'{v:29s} {nm:7s} n={len(d)} | 3D {d.err3.median()*100:.0f} cm (p90 {d.err3.quantile(.9)*100:.0f}) | horiz {d.horiz.median()*100:.0f} | '
              f'along {d.along.median()*100:.0f} | height {d.zabs.median()*100:.0f} | 3D<30 cm {np.mean(d.err3 < .3):.2f} <50 cm {np.mean(d.err3 < .5):.2f} | '
              f'cov50 {np.mean(d.hpd <= .5):.2f} cov90 {np.mean(d.hpd <= .9):.2f}')
