# Suction grasp returned by the DexNet planner. Mirrors gqcnn/GQCNNGrasp so clients
# need not import gqcnn's Python package, which pulls in TensorFlow.
geometry_msgs/Pose pose
float64 q_value

uint8 PARALLEL_JAW=0
uint8 SUCTION=1
uint8 grasp_type

float64[2] center_px
float64 angle
float64 depth
