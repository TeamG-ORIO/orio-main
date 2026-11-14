"""Robot-relative RFID sensor model + structure-free grid posterior.
Model: features of the tag's position relative to the antenna (horizontal range, azimuth from boresight, antenna height,
tag type) -> P(read in a dwell), RSSI mean and spread. No map, shelf or absolute coordinate is an input.
Training: python relmodel.py <tag>   (dwells_<tag>.pkl from dwell_table.py; 4 location-cluster folds, GPU MLP vs physics logistic)
Inference: posterior() sums per-dwell log-likelihoods (reads AND non-reads) over a map-frame xy grid."""
import sys, numpy as np, pandas as pd, torch, torch.nn as nn
dev = 'cuda' if torch.cuda.is_available() else 'cpu'; TYPES = ['regular', 'metal', 'liquid']
torch.set_num_threads(4)

def feats(rh, az, ant_z, ttype, dz):
    """tag relative to the antenna: horizontal range rh, azimuth az from boresight, height difference dz (tag - antenna);
    ant_z antenna height; ttype int tag type. All tensors broadcastable. -> (..., 11)"""
    r3 = torch.hypot(rh, dz).clamp(min=0.05); el = torch.atan2(dz, rh)
    oh = nn.functional.one_hot(ttype.long(), len(TYPES)).float().expand(*rh.shape, len(TYPES))
    return torch.cat([torch.stack([torch.log(r3), r3 / 6, el, torch.cos(az), torch.sin(az), torch.cos(2 * az),
                                   torch.cos(az) * torch.cos(el), ant_z - 1.1], -1), oh], -1)

class Rel(nn.Module):
    def __init__(s, h=64):
        super().__init__(); s.f = nn.Sequential(nn.Linear(11, h), nn.SiLU(), nn.Linear(h, h), nn.SiLU(), nn.Linear(h, 3))
    def forward(s, x):
        o = s.f(x); return o[..., 0], -60 + 10 * o[..., 1], 1.0 + nn.functional.softplus(o[..., 2]) * 4   # logit, mu, sigma (dBm)

def loglik(model, x, read, rssi):
    lg, mu, sg = model(x)
    lp = -nn.functional.softplus(-lg); lq = -nn.functional.softplus(lg)          # log p, log(1-p)
    lr = -0.5 * ((rssi - mu) / sg) ** 2 - torch.log(sg) - 0.919
    return torch.where(read > 0, lp + torch.nan_to_num(lr), lq)

def fit(T, epochs=12, seed=0):
    torch.manual_seed(seed); m = Rel().to(dev); opt = torch.optim.Adam(m.parameters(), 3e-3, weight_decay=1e-5)
    X = feats(*(torch.tensor(T[c].values, device=dev, dtype=torch.float32) for c in ('rh', 'az', 'ant_z')),
              torch.tensor(T.tag_type.map(TYPES.index).values, device=dev), torch.tensor(T.dz.values, device=dev, dtype=torch.float32))
    y = torch.tensor(T.read.values, device=dev, dtype=torch.float32); r = torch.tensor(T.rssi.fillna(0).values, device=dev, dtype=torch.float32)
    n = len(y)
    for ep in range(epochs):
        for idx in torch.randperm(n, device=dev).split(4096):
            loss = -loglik(m, X[idx], y[idx], r[idx]).mean(); opt.zero_grad(); loss.backward(); opt.step()
    return m.eval()

def clusters(T, k=5, seed=0):
    from sklearn.cluster import KMeans
    P = T.drop_duplicates('name').set_index('name')[['tag_x', 'tag_y']]
    return pd.Series(KMeans(k, n_init=10, random_state=seed).fit_predict(P.values), index=P.index)

CGRID, BMAX = (-2.0, -1.0, 0.0, 1.0, 2.0), 10.0
@torch.no_grad()
def posterior(model, D, ttype, reads, center, half=3.5, step=0.05, zr=(0.0, 1.6), zstep=0.1, beta=1.0, rmax=12.0, nuisance=True):
    """3D log-likelihood over a map-frame grid around center (xy): every dwell (read or not) counts for every cell.
    D: dwell table (gid, ax, ay, az, yaw); reads: dict gid -> max rssi for this tag. -> G (N,3), LP (N,), shape"""
    xs = torch.arange(center[0] - half, center[0] + half + 1e-6, step, device=dev); ys = torch.arange(center[1] - half, center[1] + half + 1e-6, step, device=dev)
    zs = torch.arange(zr[0], zr[1] + 1e-6, zstep, device=dev)
    G = torch.stack(torch.meshgrid(xs, ys, zs, indexing='ij'), -1).reshape(-1, 3); tt = torch.tensor(ttype, device=dev)
    A = torch.tensor(D[['ax', 'ay', 'az', 'yaw']].values, device=dev, dtype=torch.float32)
    rd = torch.tensor([1.0 if g in reads else 0.0 for g in D.gid.values], device=dev)
    rs = torch.tensor([reads.get(g, 0.0) for g in D.gid.values], device=dev, dtype=torch.float32)
    # per-tag nuisance, profiled per cell: detection logit offset c (grid search) and RSSI offset b (closed form),
    # so a weakly-coupled tag (on metal, turned, occluded) is not mistaken for a distant one
    C = torch.tensor(CGRID, device=dev); LD = torch.zeros(len(C), len(G), device=dev)
    S0 = torch.zeros(len(G), device=dev); S1 = torch.zeros_like(S0); S2 = torch.zeros_like(S0); SL = torch.zeros_like(S0)
    ch = max(1, int(2.5e6 // len(G)))
    for i0 in range(0, len(A), ch):
        a = A[i0:i0 + ch]; dx = G[None, :, 0] - a[:, None, 0]; dy = G[None, :, 1] - a[:, None, 1]; dz = G[None, :, 2] - a[:, None, 2]
        rh = torch.hypot(dx, dy); bore = torch.atan2(torch.cos(a[:, 3]), -torch.sin(a[:, 3]))[:, None]
        az = torch.remainder(torch.atan2(dy, dx) - bore + np.pi, 2 * np.pi) - np.pi
        lg, mu, sg = model(feats(rh, az, a[:, None, 2].expand_as(rh), tt, dz))
        y = rd[i0:i0 + ch, None]; ok = (rh < rmax).float()
        if not nuisance:
            lr = -0.5 * ((rs[i0:i0 + ch, None] - mu) / sg) ** 2 - torch.log(sg)
            LD[0] += (ok * torch.where(y > 0, -nn.functional.softplus(-lg) + lr, -nn.functional.softplus(lg))).sum(0); continue
        for j, c in enumerate(C):
            l2 = lg + c; LD[j] += (ok * torch.where(y > 0, -nn.functional.softplus(-l2), -nn.functional.softplus(l2))).sum(0)
        w = ok * y / sg ** 2; res = rs[i0:i0 + ch, None] - mu
        S0 += w.sum(0); S1 += (w * res).sum(0); S2 += (w * res ** 2).sum(0); SL += (ok * y * torch.log(sg)).sum(0)
    if nuisance:
        b = (S1 / S0.clamp(min=1e-9)).clamp(-BMAX, BMAX)          # bounded RSSI offset (dB)
        LP = LD.max(0).values - 0.5 * (S2 - 2 * b * S1 + b * b * S0) - SL
    else:
        LP = LD[0]
    LP = beta * LP.double(); LP -= LP.max()                # raw log-likelihood (float64); caller tempers and normalises
    return G.cpu().numpy(), LP.cpu().numpy(), (len(xs), len(ys), len(zs))

if __name__ == '__main__':
    tag = sys.argv[1]; T = pd.read_pickle(f'/home/student/rfid_db/loc/dwells_{tag}.pkl')
    from sklearn.linear_model import LogisticRegression
    from sklearn.metrics import roc_auc_score, log_loss
    import os
    if os.environ.get('RFID_FOLDS'):       # reuse another tag's location folds (multi-recording training, 2026-11-10)
        cl = pd.read_csv(os.environ['RFID_FOLDS'], index_col=0).iloc[:, 0]; T = T[T.name.isin(cl.index)].copy()
    else: cl = clusters(T)
    T['fold'] = T.name.map(cl); out = []; preds = []
    for k in range(5):
        tr, te = T[T.fold != k], T[T.fold == k]
        m = fit(tr)
        with torch.no_grad():
            X = feats(*(torch.tensor(te[c].values, device=dev, dtype=torch.float32) for c in ('rh', 'az', 'ant_z')),
                      torch.tensor(te.tag_type.map(TYPES.index).values, device=dev), torch.tensor(te.dz.values, device=dev, dtype=torch.float32))
            lg, mu, sg = m(X); p = torch.sigmoid(lg).cpu().numpy(); mu = mu.cpu().numpy(); sg = sg.cpu().numpy()
        def phys(d): return np.c_[np.log(d.r3), np.cos(d.az), np.sin(d.az), d.el]
        lr = LogisticRegression(max_iter=500).fit(phys(tr), tr.read); pb = lr.predict_proba(phys(te))[:, 1]
        rd = te.read.values == 1
        # physics RSSI baseline: linear in log range + cos az, fitted on train reads
        A = np.c_[phys(tr[tr.read == 1]), np.ones((tr.read == 1).sum())]; w = np.linalg.lstsq(A, tr[tr.read == 1].rssi.values, rcond=None)[0]
        rb = np.c_[phys(te[rd]), np.ones(rd.sum())] @ w
        out.append(dict(fold=k, test_tags=te.name.nunique(), rows=len(te), auc_mlp=roc_auc_score(te.read, p), auc_phys=roc_auc_score(te.read, pb),
                        logloss_mlp=log_loss(te.read, np.clip(p, 1e-6, 1 - 1e-6)), logloss_phys=log_loss(te.read, pb),
                        rssi_rmse_mlp=np.sqrt(np.mean((mu[rd] - te.rssi.values[rd]) ** 2)), rssi_rmse_phys=np.sqrt(np.mean((rb - te.rssi.values[rd]) ** 2)),
                        rssi_rmse_const=np.sqrt(np.mean((tr[tr.read == 1].rssi.mean() - te.rssi.values[rd]) ** 2)),
                        z90=np.mean(np.abs(mu[rd] - te.rssi.values[rd]) < 1.645 * sg[rd])))
        torch.save(m.state_dict(), f'/home/student/rfid_db/loc/relmodel_{tag}_fold{k}.pt')
    cl.to_csv(f'/home/student/rfid_db/loc/relmodel_{tag}_folds.csv')
    print(pd.DataFrame(out).round(3).to_string(index=False))
