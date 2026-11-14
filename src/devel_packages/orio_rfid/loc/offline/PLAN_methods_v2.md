# Plan: mode-seeking and additional methods on run2 (written 2026-11-10, user-approved; adapt as results come in, no further sign-off needed)
Ground rules: run2, tag r2cv (5 location folds, 104 dev items); selection on dev only; locked 28 test items = second check (already viewed);
final honest test = run1b and auto scored once with the final choice. Rows to beat (3D median dev/test, cm): v6 37/36 (no shelf map),
v6 + shelf map 29/28 and v4-kde 26/32 (per-height phase). Every variant goes to results_methods_r2cv.csv with a variant count; cov90 reported.
0. Harness: one scorer (belief on grid -> point, error, 90% region); per-pass evidence cache (top candidates only; disk 16 GB).
1. Mode-seeking estimators, no training: sharpened point estimate (belief^gamma), dominant mode (most mass), pass voting, packet fusion.
2. Learned fusion trained to rank the true cell first (per-cell MLP on LP, PH, e2e, anchor + antenna-relative geometry; soft CE over cells);
   if it helps -> 3D net on stacked volumes.
3. Mode-seeking e2e: (evolving) winner-takes-all loss; grid-output head; recalibrate; feed back into fusion.
4. Reference tags as anchors: fingerprint term, ordering; 30-min leave-one-out feasibility on refs first (stop if > ~40 cm).
5. Phase-only particle filter: feasibility on refs first.
6. Next-recording spec (lift range over item heights, slower base / faster lift, stationary sweeps).
Log of what was done and changed: see the bottom of this file.

## Log
- Step 0-1 done (mlib.py, step1.py, step1_r2cv.log). Shelf-agnostic: peak 38/37 dev/test; dominant-mode / local-mean estimators 39-40 dev (no gain),
  packet fusion 35 -> 33 dev (small). Conclusion: the evidence, not peak picking, limits the shelf-agnostic row -> steps 2-4 matter.
  With shelf boxes: peak 30/30, local mean or dominant mode 25/23-28. With kde prior + e2e + anchor terms and error-only weight picks: 22/18
  (was 26/32 for v4-kde without e2e); dominant mode (20 cm) cuts the tail: p90 66 -> 45 cm dev. Prior now also leaves out packet mates (stricter than run 1).
  Changed: pass voting deferred (needs per-pass likelihood grids; revisit only if steps 2-4 leave multi-modal failures).
- Step 2 (step2.py): learned per-cell fusion, terms only (T) 40/39 and + geometry (G) 37/46 dev/test without shelf map; with priors same as the
  hand-weighted product. No gain over step 1 -> 3D-volume network dropped.
- Step 4 (step4_feas.py, fp_term.py): reference fingerprint (cosine similarity of RSSI-over-dwells profiles to the 44 reference tags) alone:
  dev items 48 cm (k=3 kNN), as a grid term 56 cm. In the fusion without shelf map: 38 -> 36 (peak) / 31 (local mean) / 28 with packet fusion;
  within 0.62 m 72% -> 88%; p90 89 -> 64 cm. Kept. Phase-profile ordering not built (phase alone is weak, see step 5).
- Step 5 (terms_alone.py): each term alone on dev items: e2e 42, fingerprint 56, read/RSSI 60, phase per height 74 (horiz 62), flat phase 98,
  anchored 116. Phase-only evidence is the weakest position cue here -> a phase-only particle filter is not built. e2e is the strongest single
  source, so e2e improvements (step 3) are the main lever.
- Step 2, feature set C (+ distance to the antenna path): dev 34 / test 44 without shelf map: mixed, not adopted.
- Step 3 (e2e.py --wta, evolving winner-takes-all): calibrated dev 45 / test 48 vs 42 / 39 for the mixture NLL (p90 dev 84 vs 113). Not adopted.
  Changed: grid-output head not built: steps 1-3 all say mode-seeking is not the bottleneck; the remaining e2e lever tried instead is more
  real fine-tuning (tag r2cvL: 2000 fine-tune steps, fold nets saved).
- e2e with 2000 real fine-tune steps (tag r2cvL): e2e alone dev 42 -> 38, test 39 -> 37 (but its own regions are far too small: cov90 0.24);
  fused methods about unchanged on dev. Used for the final candidates because its fold nets are saved.
- Final candidates fixed (final_apply.py, settings chosen on run2 dev): F1 no map/no refs, F2 refs, F3 shelf boxes, F4 boxes x kde.
  run2 (r2cvL) dev/test: F1 35/27, F2 29/29, F3 22/22, F4 20/18; 90% regions calibrated by one temperature per method (final_T.json).
- Honest test: run_apply_folds.sh applies the run2 fold models to run1b and auto (each tag scored by models that never saw its location).
- Step 6, next recording: lift sweep must cover the item heights (antenna 0.2-1.6 m, not 1.01-1.46 m); base <= 0.1 m/s or lift period <= 4 s so a
  tag sees >= 1 full lift cycle per pass (run2: 2.6 m of travel per cycle); one stationary full lift sweep per shelf if time allows; keep 2 laps
  at two standoffs; keep reference tags on every shelf (the fingerprint term uses them); record taped antenna height at two lift positions.
- Honest test done (final_run1bF.csv, final_autoF.csv; all items, 3D median cm): run1b F1 50, F2 43, F3 31, F4 27; auto F1 52, F2 40, F3 32, F4 33;
  cov90 0.90-0.96. Frozen run-1 models on the same runs: v6 63 / 74, v4 57 / 62, v4-kde 40 / 46. Report section "Method study" written.
- Open ideas not tried: train read model + e2e on all three v2 recordings (needs the runs brought into one map frame); multi-recording fusion with
  the final methods; re-score run 1 with F1-F4 (needs fp term + S7 fix + retraining).
- (continued after the user asked why work stopped) Run 1 re-scored with F1-F4 from its existing CV evidence (final_orbfix.csv; settings and region
  temperatures from run2, S7 bug still in): all 81 items F1 55, F2 42, F3 31, F4 27 cm (dev 48/36/29/25, test 79/48/48/45); regions too small (0.70-0.81).
  Run2: 34/29/22/20. The fingerprint term also helps run 1 (dev 36 vs v6 46).
- 3-recording evidence fusion with the final methods (final_apply.py --fuse): 35/31/27/22 all items vs 34/29/22/20 run2 alone; regions overconfident
  (0.55-0.70). Not adopted. F1 within 0.62 m 79 -> 87% is the only gain.
- Error anatomy of run2 F1: |along| 12, |depth| 16, |height| 14 cm, no dependence on read count; per-shelf signed offsets up to 25 cm (S2/S4 depth +16/+25,
  S9 depth -19, S7 height -27 in F2).
- Reference residual-field correction (resid_corr.py): F1 dev 35 -> 30 (test 27 -> 28), F2 and F3 worse; reference tags themselves localize poorly
  (35-50 cm). Not adopted (F2 already uses the references and is as good).
- Running: read model + e2e trained on all three v2 recordings (make_multi.py, run_r3cv.sh, tag r3cv; run2 reads only at scoring).
- Trained on all three recordings (tag r3cv): v3 61 (was 49), e2e 45 (38), F1-F4 dev 34/27/22/22, test 41/35/26/21. No gain; not adopted.
- e2e ensembles (r2cvE: 800 + 2000 steps; r2cvE3: + WTA): F1-F4 dev 35/27-29/22-23/20. No change.
- F3 with boxes from corner tags + nominal depth 0.45 m (RFID_GENERIC_DEPTH): 25/25 dev/test vs 22/22 with measured depths -> no measured shelf map needed.
- Object level (run2, 118 objects): F1 36, F2 30, F3 24, F4 20 cm; multi-tag (metal/liquid) objects' tags ~10 cm better than single-tag dry items.
- Verdict: plateau. Six further variations move the final methods by <= ~2 cm. Remaining error is balanced (along 12 / depth 16 / height 14 cm) with
  per-shelf offsets up to 25 cm -> next gains from the recording and pose accuracy, not the estimator.
