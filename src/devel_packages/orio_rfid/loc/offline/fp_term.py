"""Plan step 4: reference-tag fingerprint term. Anchors = the reference tags only (known positions through the map->room tie). For a tag, each
anchor gets weight softmax(cosine similarity of the smoothed RSSI-over-dwells profiles / Ts); the term on the tag's grid is
log sum_a w_a N(cell; anchor a, sxy, sz). Ts, sxy, sz are chosen on the REFERENCE tags only (leave-self-out mean log density at their own
position), never on items. Usage: fp_term.py <tag> -> fp_<tag>.pkl {name: float32 over that tag's grid}"""
import sys, pickle, itertools, numpy as np, pandas as pd, mlib as M
tag = sys.argv[1]; T = pd.read_pickle(M.L + f'dwells_{tag}.pkl'); GR = pickle.load(open(M.L + f'grids3d_{tag}.pkl', 'rb'))
names = sorted(T.name.unique()); ng = T.gid.max() + 1; V = np.zeros((len(names), ng)); ix = {n: i for i, n in enumerate(names)}
r = T[T.read == 1]; V[r.name.map(ix).values, r.gid.values] = r.rssi.values + 80
ker = np.array([.25, .5, 1, .5, .25]); Vs = np.stack([np.convolve(v, ker, 'same') for v in V]); Vn = Vs / np.linalg.norm(Vs, axis=1, keepdims=True).clip(1e-9)
S = Vn @ Vn.T; P = T.drop_duplicates('name').set_index('name').loc[names, ['tag_x', 'tag_y', 'tag_z']].values
refs = np.array([i for i, n in enumerate(names) if n.startswith('Ref ')])
def term(i, X, Ts, sxy, sz):
    a = refs[refs != i]; w = S[i, a] / Ts; w = np.exp(w - w.max()); w /= w.sum()
    d2 = ((X[:, None, :2] - P[a][None, :, :2]) ** 2).sum(-1) / sxy ** 2 + (X[:, None, 2] - P[a][None, :, 2]) ** 2 / sz ** 2
    return np.log((w[None] * np.exp(-0.5 * d2)).sum(1) + 1e-300) - 2 * np.log(sxy) - np.log(sz)
best = max((np.mean([term(i, P[i][None], Ts, sxy, sz)[0] for i in refs]), Ts, sxy, sz)
           for Ts, sxy, sz in itertools.product((0.02, 0.05, 0.1, 0.2), (0.3, 0.5, 0.8, 1.2), (0.2, 0.3, 0.5, 0.8)))
print('picked on refs (leave-self-out): Ts %.2f sxy %.1f sz %.1f, mean log density %.2f' % (best[1], best[2], best[3], best[0]))
out = {n: term(ix[n], GR[n]['G'].astype(np.float64), *best[1:]).astype(np.float32) for n in GR if n in ix}
pickle.dump(out, open(M.L + f'fp_{tag}.pkl', 'wb')); print(len(out), 'tags')
