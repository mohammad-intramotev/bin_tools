# Save this as sync_and_parse.py
import os
import sys
import glob
import struct
import numpy as np
import pymap3d as pm
from scipy.spatial.transform import Rotation as R

from common import navigation_pb2, orientation_pb2, config

def get_first_timestamp_from_tum(file_path):
    """Reads the first line of a TUM file and returns its timestamp."""
    with open(file_path, 'r') as f:
        first_line = f.readline()
        if not first_line:
            raise ValueError(f"File is empty: {file_path}")
        return float(first_line.split()[0])

def euler_to_quaternion(roll, pitch, yaw):
    """Converts Euler angles (in degrees) to a quaternion (qx, qy, qz, qw)."""
    return R.from_euler('zyx', [yaw, pitch, roll], degrees=True).as_quat()

def lla_to_enu(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    """Converts LLA to a local ENU frame."""
    return pm.geodetic2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref)

def process_log_file(input_bin_file, reference_tum_file, output_tum_file):
    """
    Reads a binary log, synchronizes its timestamps with a reference TUM file,
    and converts the data to a new, aligned TUM trajectory file.
    """
    positions = []
    orientations = []

    print("--- Step 1: Parsing binary log file ---")
    with open(input_bin_file, 'rb') as f:
        while True:
            try:
                timestamp_bytes = f.read(8)
                if not timestamp_bytes: break
                channel_len_bytes = f.read(1)
                channel_len = struct.unpack('>B', channel_len_bytes)[0]
                channel_name_bytes = f.read(channel_len)
                channel_name = channel_name_bytes.decode('utf-8')
                msg_len_bytes = f.read(4)
                msg_len = struct.unpack('>L', msg_len_bytes)[0]
                payload = f.read(msg_len)
                if channel_name == config.POS_CHANNEL_NAME:
                    pos_msg = navigation_pb2.MsgPosLlh()
                    pos_msg.ParseFromString(payload)
                    positions.append(pos_msg)
                elif channel_name == config.ORIENT_CHANNEL_NAME:
                    orient_msg = orientation_pb2.MsgOrientEuler()
                    orient_msg.ParseFromString(payload)
                    orientations.append(orient_msg)
            except (struct.error, IndexError):
                break

    if not positions:
        print(f"\nError: No position data found on channel '{config.POS_CHANNEL_NAME}'. Cannot proceed.", file=sys.stderr)
        sys.exit(1)

    positions.sort(key=lambda p: p.tow)
    orientations.sort(key=lambda o: o.tow)
    
    print(f"Found {len(positions)} position messages and {len(orientations)} orientation messages.")

    print("\n--- Step 2: Synchronizing Timestamps ---")
    # Get the starting timestamps from both sources
    first_gps_tow_sec = positions[0].tow / 1000.0
    slam_start_time_sec = get_first_timestamp_from_tum(reference_tum_file)

    # Calculate the offset
    time_offset = slam_start_time_sec - first_gps_tow_sec
    print(f"SLAM file starts at: {slam_start_time_sec:.4f} s")
    print(f"GPS data starts at:  {first_gps_tow_sec:.4f} s (GPS Time of Week)")
    print(f"Calculated time offset: {time_offset:.4f} s")
    print("Applying this offset to all GPS timestamps.")

    print("\n--- Step 3: Generating Synchronized Ground Truth File ---")
    ref_pos = positions[0]
    lat_ref, lon_ref, alt_ref = ref_pos.lat, ref_pos.lon, ref_pos.height
    orient_tows = np.array([o.tow for o in orientations])

    with open(output_tum_file, 'w') as tum_file:
        for pos_msg in positions:
            time_diffs = np.abs(orient_tows - pos_msg.tow)
            closest_idx = np.argmin(time_diffs)
            if time_diffs[closest_idx] > 50: continue
            orient_msg = orientations[closest_idx]
            
            # Apply the offset to the timestamp
            original_timestamp = pos_msg.tow / 1000.0
            synchronized_timestamp = original_timestamp + time_offset

            tx, ty, tz = lla_to_enu(pos_msg.lat, pos_msg.lon, pos_msg.height, lat_ref, lon_ref, alt_ref)
            
            roll_deg = orient_msg.roll * 1e-6
            pitch_deg = orient_msg.pitch * 1e-6
            yaw_deg = orient_msg.yaw * 1e-6
            qx, qy, qz, qw = euler_to_quaternion(roll_deg, pitch_deg, yaw_deg)

            tum_file.write(f"{synchronized_timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.8f} {qy:.8f} {qz:.8f} {qw:.8f}\n")
            
    print(f"\nDone. Synchronized ground truth saved to: {output_tum_file}")

if __name__ == '__main__':
    bin_files = glob.glob(os.path.join(config.INPUT_TUM_DIR, "*.bin"))
    input_bin_file = bin_files[0]
    print(f"Found binary file: {input_bin_file}")
    process_log_file(input_bin_file, config.INPUT_TRAJ_2, config.OUTPUT_TUM_DIR)