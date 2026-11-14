"""
orio_paths.py - where the code and the data live. Python side of orio_env.sh; not a node, import it.

Code is this package. Data (recordings, maps, per-site and per-boot settings) stays outside the repo, under
$ORIO_DATA (default ~/orio), so that a checkout never holds bags and the same data serves every checkout.
"""
import os

PKG_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DATA_DIR = os.environ.get('ORIO_DATA') or os.path.expanduser('~/orio')

DATASETS_DIR = os.path.join(DATA_DIR, 'datasets')       # one folder per recording / run
MAPS_DIR = os.path.join(DATA_DIR, 'maps')               # orbslam/<map>.osa + <map>.live.yaml, logs/
SITE_DIR = os.path.join(DATA_DIR, 'config')             # waypoints_<map>.yaml, truth_<site>.txt, *.env

PAYLOAD_ENV = os.path.join(SITE_DIR, 'payload.env')     # LIFT_EFFORT, written by home_payload.py every boot
