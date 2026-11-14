#!/bin/bash
# rate_matrix.sh - try reader settings one after another (static antenna) and summarise read rates
cd "$(dirname "$0")"; D=30
run() { label=$1; shift; echo "=== $label: $*"; timeout $((D+40)) python3 rfid_log.py scan "mx_$label" --duration $D "$@" 2>&1 | grep -E "overall|channels used|per-tag read rate|Invalid mode|valid mode|rror" | cut -c1-200; }
run mode0_dual    --power 33 --mode 0 --search-mode 2
run mode100_dual  --power 33 --mode 100 --search-mode 2
run mode1000_dual --power 33 --mode 1000 --search-mode 2
run mode1003_dual --power 33 --mode 1003 --search-mode 2
run mode1111_dual --power 33 --mode 1111 --search-mode 2
run mode0_single  --power 33 --mode 0 --search-mode 1
run mode0_s1_dual --power 33 --mode 0 --search-mode 2 --session 1
run mode0_dual_pop16 --power 33 --mode 0 --search-mode 2 --population 16
run mode0_dual_pop256 --power 33 --mode 0 --search-mode 2 --population 256
run mode0_p27 --power 27 --mode 0 --search-mode 2
echo MATRIX_DONE
