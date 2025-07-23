INPUT_FILES = "input_files"
OUTPUT_FILES = "output_files"

# -- For bin_merge.py
INPUT_MERGE_DIR = f"{INPUT_FILES}/merge_bins/"
# --- For bin_to_ros<x>bag<x>
INPUT_ROSBAG_BIN = f"{INPUT_FILES}/ros_bins/"
# --- For bin_to_tum.py
INPUT_TUM_DIR = f"{INPUT_FILES}/tum_bins/"
# --- For tum_cmp.py
INPUT_TRAJ_1 = f"{INPUT_FILES}/tum_cmp/ground_truth.txt" # Ground truth trajectory
# --- For bin_to_tum.py, tum_cmp.py
INPUT_TRAJ_2 = f"{INPUT_FILES}/tum_cmp/traj_lidar.txt" # Estimated trajectory
# --- For bin_diagnose
INPUT_BIN = f"{INPUT_FILES}/bins/"

# --- For bin_merge.py
OUTPUT_MERGE_DIR = f"{OUTPUT_FILES}/combined_bins/"
# --- For bin_to_ros1bag<x>
OUTPUT_ROS1BAGS = f"{OUTPUT_FILES}/ros1_bags/"
# --- For bin_to_ros12bag<x>
OUTPUT_ROS2BAGS = f"{OUTPUT_FILES}/ros2_bags/"
# --- For bin_to_tum.py
OUTPUT_TUM_FILE= f"{OUTPUT_FILES}/tum_files/ground_truth.txt"
# --- For tum_cmp.py
OUTPUT_TRAJ_FILES = f"{OUTPUT_FILES}/traj_files/"

# --- For bin_to_ros1bag.py
GRAVITY_ACCEL = 9.80665  # m/s^2
LIDAR_FRAME_ID = "lidar"
IMU_FRAME_ID = "imu"
LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"

# --- For bin_to_tum.py
POS_CHANNEL_NAME = "duro_gps_llh"
ORIENT_CHANNEL_NAME = "duro_gps_orient_eule"
MAX_GPS_FUSION_AGE_MS = 75
LIDAR_CHANNEL_NAME = "parse_livox_point_packet"
IMU_ACCEL_GYRO_CHANNEL_NAME = "parse_livox_imu_data"
IMU_ORIENTATION_CHANNEL_NAME = "parse_duro_orient_euler"

# --- For bin_diagnose.py
LIVOX_AVIA_POINT_RATE = 240000  # Avia spec: 240k pts/sec (single return mode)
LIVOX_AVIA_FRAME_TIME_MS = 200
LIVOX_AVIA_IMU_RATE_HZ = 200


T_MAX_DIFF = 0.05