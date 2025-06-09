import sys
import struct
import numpy as np
import pymap3d as pm
from scipy.spatial.transform import Rotation as R

from common import navigation_pb2, orientation_pb2, config


def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles (in degrees) to a quaternion (qx, qy, qz, qw).
    
    Args:
        roll (float): Roll angle in degrees.
        pitch (float): Pitch angle in degrees.
        yaw (float): Yaw angle in degrees.
        
    Returns:
        A numpy array [qx, qy, qz, qw].
    """
    return R.from_euler('zyx', [yaw, pitch, roll], degrees=True).as_quat()

def lla_to_enu(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    """Converts Latitude, Longitude, Altitude (LLA) to a local East, North, Up (ENU) frame."""
    return pm.geodetic2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref)

def process_log_file(input_bin_file, output_tum_file):
    """
    Reads a binary log file with a specific custom format, extracts position and
    orientation data, and converts it to a TUM trajectory file.
    """
    positions = []
    orientations = []
    records_processed = 0

    print(f"Reading data from {input_bin_file}...")
    
    with open(input_bin_file, 'rb') as f:
        while True:
            try:
                # 1. Read Timestamp (8 bytes)
                timestamp_bytes = f.read(8)
                if not timestamp_bytes:
                    break  # End of file
                
                # 2. Read Channel Name Length (1 byte)
                channel_len_bytes = f.read(1)
                channel_len = struct.unpack('>B', channel_len_bytes)[0]
                
                # 3. Read Channel Name (L bytes)
                channel_name_bytes = f.read(channel_len)
                channel_name = channel_name_bytes.decode('utf-8')
                
                # 4. Read Protobuf Message Length (4 bytes)
                msg_len_bytes = f.read(4)
                msg_len = struct.unpack('>L', msg_len_bytes)[0]
                
                # 5. Read Protobuf Message Data (M bytes)
                payload = f.read(msg_len)
                
                records_processed += 1

                # Check if this is a channel we care about
                if channel_name == config.POS_CHANNEL_NAME:
                    pos_msg = navigation_pb2.MsgPosLlh()
                    pos_msg.ParseFromString(payload)
                    positions.append(pos_msg)
                
                elif channel_name == config.ORIENT_CHANNEL_NAME:
                    orient_msg = orientation_pb2.MsgOrientEuler()
                    orient_msg.ParseFromString(payload)
                    orientations.append(orient_msg)

            except (struct.error, IndexError) as e:
                print(f"Warning: Stopped reading due to incomplete record at end of file. Error: {e}")
                break

    print(f"Finished reading. Processed {records_processed} records.")
    print(f"Found {len(positions)} position messages on channel '{config.POS_CHANNEL_NAME}'.")
    print(f"Found {len(orientations)} orientation messages on channel '{config.ORIENT_CHANNEL_NAME}'.")

    if not positions or not orientations:
        print("\nError: No position or orientation data found. Cannot create TUM file.", file=sys.stderr)
        return

    positions.sort(key=lambda p: p.tow)
    orientations.sort(key=lambda o: o.tow)

    ref_pos = positions[0]
    lat_ref, lon_ref, alt_ref = ref_pos.lat, ref_pos.lon, ref_pos.height
    print(f"\nUsing first GPS point as origin: Lat={lat_ref:.6f}, Lon={lon_ref:.6f}, Height={alt_ref:.2f} m")

    orient_tows = np.array([o.tow for o in orientations])

    print(f"Synchronizing data and writing to {output_tum_file}...")
    with open(output_tum_file, 'w') as tum_file:
        for pos_msg in positions:
            time_diffs = np.abs(orient_tows - pos_msg.tow)
            closest_idx = np.argmin(time_diffs)
            
            if time_diffs[closest_idx] > config.MAX_GPS_FUSION_AGE_MS:
                continue

            orient_msg = orientations[closest_idx]
            
            # 1. Timestamp (convert GPS Time of Week from ms to seconds)
            timestamp = pos_msg.tow / 1000.0

            # 2. Position: Convert LLA to local ENU
            tx, ty, tz = lla_to_enu(pos_msg.lat, pos_msg.lon, pos_msg.height, lat_ref, lon_ref, alt_ref)
            
            # 3. Orientation: Convert scaled Euler angles to a quaternion
            # SBP spec for MsgOrientEuler gives roll/pitch/yaw as scaled integers in micro-degrees.
            roll_deg = orient_msg.roll * 1e-6
            pitch_deg = orient_msg.pitch * 1e-6
            yaw_deg = orient_msg.yaw * 1e-6
            
            # Note: The scipy `as_quat()` returns in [x, y, z, w] order, which is what we need.
            qx, qy, qz, qw = euler_to_quaternion(roll_deg, pitch_deg, yaw_deg)

            # Write the formatted line to the file
            tum_file.write(f"{timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.8f} {qy:.8f} {qz:.8f} {qw:.8f}\n")
            
    print("Done. TUM file generated successfully.")

if __name__ == '__main__':
    process_log_file(config.INPUT_TUM_BIN, config.OUTPUT_TUM_FILE)