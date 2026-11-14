"""Location folds of a source CV tag carried to another recording of the same arrangement: same fold for the same tag; a tag the source never
scored gets the fold of the nearest source tag (room frame). Also links the source's per-fold read models. Usage: make_folds_like.py <src tag> <new tag>"""
import sys, os, numpy as np, pandas as pd
L = '/home/student/rfid_db/loc/'; src, new = sys.argv[1], sys.argv[2]
fs = pd.read_csv(L + f'relmodel_{src}_folds.csv', index_col=0).iloc[:, 0]; g = pd.read_csv(os.environ['RFID_GT']).set_index('name')[['X_room_m', 'Y_room_m']]
T = pd.read_pickle(L + f'dwells_{new}.pkl'); names = sorted(T.name.unique()); P = g.loc[fs.index].values; out = {}
for n in names: out[n] = int(fs[n]) if n in fs.index else int(fs.iloc[np.argmin(np.linalg.norm(P - g.loc[n].values, axis=1))])
pd.Series(out).to_csv(L + f'relmodel_{new}_folds.csv'); print(len(names), 'tags;', sum(n not in fs.index for n in names), 'not in source folds')
for k in sorted(fs.unique()):
    d = L + f'relmodel_{new}_fold{k}.pt'
    if os.path.lexists(d): os.remove(d)
    os.symlink(f'relmodel_{src}_fold{k}.pt', d)
