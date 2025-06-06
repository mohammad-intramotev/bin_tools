from pyproj import CRS


INPUT_FILES = "input_files/"
INPUT_MERGE_BIN = f"{INPUT_FILES}/example.bin"
INPUT_ROSBAG_BIN = f"{INPUT_FILES}/example.bin"
INPUT_TUM_BIN = f"{INPUT_FILES}/example.bin"
INPUT_TRAJ_1 = f"{INPUT_FILES}/ground_truth.txt"
INPUT_TRAJ_2 = f"{INPUT_FILES}/estimate.txt"

OUTPUT_FILES = "output_files"
OUTPUT_MERGE_BIN = f"{OUTPUT_FILES}/combined_example.bin"
OUTPUT_ROSBAGS = f"{OUTPUT_FILES}/ros_bags/"
OUTPUT_TUM_FILE = f"{OUTPUT_FILES}/ground_truth.txt"
OUTPUT_TRAJ_FILES = f"{OUTPUT_FILES}/traj_files/"

GRAVITY_ACCEL = 9.80665  # m/s^2
MAX_ORIENTATION_AGE_MS = 200  # Max age of orientation data to use
IMU_BUFFER_MAX_LEN = 100

LIDAR_FRAME_ID = "lidar"
IMU_FRAME_ID = "imu"

LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"

GPS_ORIENTATION_CHANNEL = "duro_gps_orient_eule"
GPS_POSITION_CHANNEL = "duro_gps_llh"

# --- GPS Coordinate System ---
IN_PROJ = CRS("EPSG:4326")
OUT_PROJ = CRS("EPSG:32615")

MAX_GPS_FUSION_AGE_MS = 75