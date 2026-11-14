"""Final candidate methods with FIXED settings (chosen on run2 dev items, PLAN_methods_v2.md), applied to any tag that has grids3d / e2e gridlp /
fp files. Evidence = beta*read/RSSI + kappa*phase (per height cell) + lam*e2e [+ phi*reference fingerprint]; tags of one object share evidence.
  F1 no shelf map, no reference tags : beta .03 kappa .3 lam 1            packet fusion, local mean around the peak
  F2 reference tags, no shelf map    : beta .01 kappa .3 lam .3 phi 1     packet fusion, dominant mode (20 cm)
  F3 shelf boxes                     : beta .01 kappa .3 lam .3 phi .3    packet fusion, dominant mode
  F4 shelf boxes x kde of other items: beta .01 kappa .3 lam .3           dominant mode
90% regions: the belief flattened by a temperature (belief^(1/T)); T per method from final_T.json, written by --calib (on the dev items of the
tag given, coverage closest to 0.90). Usage: final_apply.py <tag> [--calib] [--e2e <gridlp file>] -> final_<tag>.csv"""
import sys, json, os, numpy as np, pandas as pd, multiprocessing as mp, mlib as M
tag = sys.argv[1]; CAL = '--calib' in sys.argv; E2F = sys.argv[sys.argv.index('--e2e') + 1] if '--e2e' in sys.argv else None
FUSE = [M.Run(x) for x in sys.argv[sys.argv.index('--fuse') + 1].split(',')] if '--fuse' in sys.argv else []   # other recordings of the same tags: evidence added in the room frame
OT = tag + ('_fuse' + str(len(FUSE) + 1) if FUSE else '') + ('_gd' + os.environ['RFID_GENERIC_DEPTH'] if os.environ.get('RFID_GENERIC_DEPTH') else '')
RUN = M.Run(tag, E2F); TF = M.L + 'final_T.json'; TS = (0.5, 0.7, 1, 1.5, 2, 3, 5, 8)
F = dict(F1=dict(b=.03, k=.3, l=1.0, f=0.0, pk=1, pr='none', est='lmean'), F2=dict(b=.01, k=.3, l=.3, f=1.0, pk=1, pr='none', est='mode'),
         F3=dict(b=.01, k=.3, l=.3, f=.3, pk=1, pr='box', est='mode'), F4=dict(b=.01, k=.3, l=.3, f=0.0, pk=0, pr='kde', est='mode'))
def work(obj):
    names = [n for n in RUN.mates(obj) if n in RUN.names]; T = {n: RUN.terms(n) for n in names}; rows = []
    for fn, c in F.items():
        lp = {n: c['b'] * T[n]['LP'] + c['k'] * T[n]['PH'] + c['l'] * T[n]['E2'] + c['f'] * T[n]['FP'] for n in names}; lp = {n: v - v.max() for n, v in lp.items()}
        for Rb in FUSE:
            for n in names:
                if n not in Rb.names: continue
                Tb = Rb.terms(n); vb = c['b'] * Tb['LP'] + c['k'] * Tb['PH'] + c['l'] * Tb['E2'] + c['f'] * Tb['FP']; vb -= vb.max(); G = T[n]['G']
                Gq = np.c_[(G[:, :2] @ RUN.R.T + RUN.t - Rb.t) @ Rb.R, G[:, 2]]; lp[n] = lp[n] + M.resample(Tb['G'], vb, Tb['shp'], Gq, vb.min())
        for n in names:
            G, shp = T[n]['G'], T[n]['shp']
            v = lp[n] + (sum(M.resample(T[m]['G'], lp[m], T[m]['shp'], G, lp[m].min()) for m in names if m != n) if c['pk'] else 0.0); p = M.norm(v)
            if c['pr'] != 'none':
                sh, box, kde = RUN.priors(n, G); p = M.with_prior(p, box if c['pr'] == 'box' else kde, 0.01 if c['pr'] == 'box' else 0.001)
            P = M.lmean(G, p, int(p.argmax())) if c['est'] == 'lmean' else M.lmean(G, p, M.blur_mode(G, p, shp, 0.2), 0.2)
            it = int(np.argmin(np.linalg.norm(G - RUN.GR[n]['tm'], axis=1))); lg = np.log(p + 1e-300)
            rows.append(dict(name=n, method=fn, x=P[0], y=P[1], z=P[2], **RUN.err(n, P), **{f'hpd_{t}': M.hpd(M.norm(lg / t), it) for t in TS}))
    return rows
if __name__ == '__main__':
    objs = sorted({RUN.obj[n] for n in RUN.items})
    with mp.Pool(3 if FUSE else 5) as pool: A = pd.DataFrame([r for rr in pool.map(work, objs, chunksize=2) for r in rr])
    A = A[~A.name.str.startswith('Ref ')]; A['test'] = A.name.isin(RUN.TEST)
    if CAL:
        d = A[~A.test]; Tm = {fn: min(TS, key=lambda t: abs((g[f'hpd_{t}'] <= .9).mean() - .9)) for fn, g in d.groupby('method')}; json.dump(Tm, open(TF, 'w')); print('region temperatures', Tm)
    Tm = json.load(open(TF)); A['hpd'] = [r[f'hpd_{Tm[r.method]}'] for r in A.itertuples(index=False)] if False else A.apply(lambda r: r[f'hpd_{Tm[r.method]}'], axis=1)
    A.drop(columns=[c for c in A if c.startswith('hpd_')]).to_csv(M.L + f'final_{OT}.csv', index=False); out = []
    for fn in F:
        d = A[A.method == fn]; out += M.report(d, f'{OT} {fn}')
        e = d.err3; print(f'      all items n={len(d)} 3D {e.median()*100:.0f} cm (p90 {e.quantile(.9)*100:.0f}) horiz {d.horiz.median()*100:.0f} height {d.zabs.median()*100:.0f} <0.62 {np.mean(e < .62):.2f} cov90 {np.mean(d.hpd <= .9):.2f}')
    pd.DataFrame(out).to_csv(M.L + f'final_{OT}_summary.csv', index=False)
