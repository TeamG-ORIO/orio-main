# source v2env.sh <label>   -> environment selecting a v2 recording + the v2 site (ground truth, geometry, leave-packet-out shelf map)
R=/home/student/rfid_db
export RFID_READS=$R/datasets/v2/$1/$1 RFID_BAGNPZ=$R/datasets/v2/$1/bag_$1.npz RFID_GT=$R/ground_truth/ground_truth_v2.csv
export RFID_SITE=$R/ground_truth/site_v2.json RFID_SHELFGT=$R/ground_truth/ground_truth_v2.csv RFID_LOO=1 OMP_NUM_THREADS=4 MKL_NUM_THREADS=4
PY=/home/student/anaconda3/envs/rfid/bin/python
