#!/bin/bash
# Honest test of run2-trained fold models on ANOTHER v2 recording: every tag is scored by the read model and e2e net that never saw its location.
# Usage: run_apply_folds.sh <v2 label> <new tag> <src cv tag for read models> <src cv tag for e2e nets>
set -eo pipefail; cd /home/student/rfid_db/loc; source v2env.sh $1; B=$1; TAG=$2; export RFID_TESTSET=testset_locked_v2.csv
TR=../datasets/v2/$B/base_traj_${B}_cf.txt
for f in dwells_$B.pkl dwell_meta_$B.npz loc_v1_$B.csv; do ln -sf $f ${f/$B/$TAG}; done
$PY make_folds_like.py $3 $TAG
echo "== grids $(date +%T)"; $PY bayes3d.py $TR $TAG 2>&1 | grep -v " reads$" | tail -3 > bayes3d_$TAG.log
echo "== e2e apply $(date +%T)"; $PY e2e_data.py $TR $TAG > e2e_data_$TAG.log 2>&1; $PY e2e.py $TAG --applyfolds $4 2>&1 | tail -1
echo "== anchor $(date +%T)"; $PY anchor_term.py $TR $TAG > anchor_$TAG.log 2>&1
echo "== fingerprint $(date +%T)"; $PY fp_term.py $TAG 2>&1 | tail -2
echo "== done $(date +%T)"
