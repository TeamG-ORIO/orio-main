"""Shelf-agnostic 3D tag localisation: reads + robot poses + lift only (no shelf membership, geometry or corner tags).
Per tag: strong visits (within GATE dB of the tag's best pass), each a dwell-differential SAR score on ONE shared world grid;
visits at different lift heights give rings at different heights, so summing them resolves depth/height.
Estimators: rssi (strongest-read point), sar1 (best single visit), sarF (fused over all strong visits).
Usage: loc_v1.py <pose src: odom | traj file (t x y yaw)> <out tag>   -> loc_v1_<tag>.csv (pose frame, metres)"""
import sys, numpy as np, pandas as pd, torch, sarlib as S
src, tag = sys.argv[1], sys.argv[2]
GATE, HALF, STEP, FINE, ZR = 3.0, 1.5, 0.04, 0.01, (0.0, 2.2)
df = S.load_reads(); traj, lift = S.load_traj(src); df = S.pose_reads(df, traj, lift)
Vs = S.visits(df, keep_q=0.0, maxn=4000, min_reads=30)
best = {}
for V in Vs: best[V['name']] = max(best.get(V['name'], -99), V['g'].rssi_dbm.max())
def grid(c, half, step, zr):
    xs = torch.arange(c[0] - half, c[0] + half + 1e-6, step, device=S.dev); ys = torch.arange(c[1] - half, c[1] + half + 1e-6, step, device=S.dev)
    zs = torch.arange(zr[0], zr[1] + 1e-6, step, device=S.dev)
    return torch.stack(torch.meshgrid(xs, ys, zs, indexing='ij'), -1).reshape(-1, 3)
def vscore(V, P):                     # normalised dwell coherence in [-1, 1]
    n = torch.bincount(V['gid'], minlength=V['ng']).float(); return S.score_dwell(P, V['A'], V['k'], V['ph'], V['gid'], V['ng']) / (n - 1).sum()
rows = []
for name in sorted(best):
    vs = [V for V in Vs if V['name'] == name and V['g'].rssi_dbm.max() >= best[name] - GATE]
    g = pd.concat([V['g'] for V in vs]); top = g[g.rssi_dbm >= g.rssi_dbm.quantile(0.9)]
    c = np.array([(top.ant_x + 0.5 * top.bore_x).mean(), (top.ant_y + 0.5 * top.bore_y).mean()])
    r = dict(name=name, nvis=len(vs), lifts='/'.join(sorted({f'{V["g"].lift_m.median():.2f}' for V in vs})),
             rssi_x=c[0], rssi_y=c[1], rssi_z=top.ant_z.mean())
    pv = [S.prep_dwell(V, -1.0) for V in vs]; pv = [V for V in pv if V['ng'] >= 15]
    if pv:
        P = grid(c, HALF, STEP, ZR); Ss = torch.stack([vscore(V, P) for V in pv])
        i1 = int(torch.argmax(Ss[np.argmax([V['ng'] for V in pv])])); r.update(zip(('sar1_x', 'sar1_y', 'sar1_z'), P[i1].tolist()))
        F = Ss.mean(0); i = int(torch.argmax(F)); p0 = P[i].cpu().numpy()
        P2 = grid(p0, STEP, FINE, (p0[2] - STEP, p0[2] + STEP)); F2 = torch.stack([vscore(V, P2) for V in pv]).mean(0); j = int(torch.argmax(F2))
        r.update(zip(('sarF_x', 'sarF_y', 'sarF_z'), P2[j].tolist())); r.update(sarF_peak=float(F2[j]), sarF_contrast=float(F2[j] / F.mean().clamp(min=1e-6)),
                                                                              nsar=len(pv))
    rows.append(r)
out = pd.DataFrame(rows); out.to_csv(f'/home/student/rfid_db/loc/loc_v1_{tag}.csv', index=False)
print(src, len(out), 'tags,', out.sarF_x.notna().sum(), 'with fused SAR; lift sets:', out.lifts.value_counts().head(6).to_dict())
