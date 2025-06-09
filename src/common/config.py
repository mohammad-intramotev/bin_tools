INPUT_FILES = "input_files"
# -- For bin_merge.py
INPUT_MERGE_BIN = f"{INPUT_FILES}/example.bin" 
# --- For bin_to_rosXbag
INPUT_ROSBAG_BIN = f"{INPUT_FILES}/example.bin"
# --- For bin_to_tum.py
INPUT_TUM_BIN = f"{INPUT_FILES}/log_0855-0999.bin"
# --- For bin_to_tum.py, traj_cmp.py
INPUT_TRAJ_2 = f"{INPUT_FILES}/CBD_Building_01.txt" # Estimated trajectory

OUTPUT_FILES = "output_files"
# --- For bin_merge.py
OUTPUT_MERGE_BIN = f"{OUTPUT_FILES}/combined_example.bin"
# --- For bin_to_rosXbag
OUTPUT_ROSBAGS = f"{OUTPUT_FILES}/ros_bags/"
# --- For bin_to_tum.py
OUTPUT_TUM_FILE = f"{OUTPUT_FILES}/ground_truth.txt"
# --- For traj_cmp.py
INPUT_TRAJ_1 = f"{OUTPUT_FILES}/ground_truth.txt" # Ground truth trajectory
OUTPUT_TRAJ_FILES = f"{OUTPUT_FILES}/traj_files/"

# --- For bin_to_rosXbag
GRAVITY_ACCEL = 9.80665  # m/s^2
MAX_ORIENTATION_AGE_MS = 200  # Max age of orientation data to use
IMU_BUFFER_MAX_LEN = 100
LIDAR_FRAME_ID = "lidar"
IMU_FRAME_ID = "imu"
LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"

# --- For bin_to_tum.py
POS_CHANNEL_NAME = "duro_gps_llh"
ORIENT_CHANNEL_NAME = "duro_gps_orient_eule"
MAX_GPS_FUSION_AGE_MS = 75