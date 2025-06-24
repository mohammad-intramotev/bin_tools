#!/usr/bin/env python3

import os
import glob
import math
from collections import defaultdict
from common import livox_pb2, config

def human_readable_size(num_bytes):
    if num_bytes is None: return "N/A"
    for unit in ['B','KB','MB','GB','TB']:
        if num_bytes < 1024.0:
            return f"{num_bytes:3.1f} {unit}"
        num_bytes /= 1024.0
    return f"{num_bytes:.1f} PB"

def validate_point_frame(stats, packet):
    """Validates the internal consistency of a single Point_Packet frame."""
    num_points = packet.num_points
    x_coords_len = len(packet.x_coords)
    
    # 1. Check if point count matches array lengths
    if num_points != x_coords_len:
        stats['warnings'].append(f"Point count mismatch in frame at TS {packet.timestamp_us}. Header: {num_points}, Actual: {x_coords_len}")

    # 2. Check for monotonic hardware timestamp
    if packet.timestamp_us <= stats['last_ts']['avia_points']:
        stats['warnings'].append(f"Non-monotonic LiDAR hardware timestamp detected at {packet.timestamp_us}")
    stats['last_ts']['avia_points'] = packet.timestamp_us

    # 3. Check final timestamp offset against expected frame duration
    if packet.timestamp_offset_us:
        last_offset_us = packet.timestamp_offset_us[-1]
        expected_us = config.LIVOX_AVIA_FRAME_TIME_MS * 1000
        # Allow a 10% tolerance
        if not (0.9 * expected_us < last_offset_us < 1.1 * expected_us):
            stats['warnings'].append(f"Unusual frame duration. Last offset: {last_offset_us}us, Expected: ~{expected_us}us")
            
    # 4. Check for valid reflectivity values (optional, but good)
    if packet.reflectivity and not (0 <= packet.reflectivity[0] <= 255):
        stats['warnings'].append(f"Invalid reflectivity value found: {packet.reflectivity[0]}")
        
    stats['total_lidar_points'] += num_points

def validate_imu_message(stats, msg):
    """Validates the internal consistency of a single Imu_Data message."""
    # 1. Check for monotonic hardware timestamp
    if msg.timestamp_us <= stats['last_ts']['avia_imu']:
        stats['warnings'].append(f"Non-monotonic IMU hardware timestamp detected at {msg.timestamp_us}")
    stats['last_ts']['avia_imu'] = msg.timestamp_us

    # 2. Check for sane floating point values
    for val in [msg.gyro_x, msg.gyro_y, msg.gyro_z, msg.acc_x, msg.acc_y, msg.acc_z]:
        if math.isnan(val) or math.isinf(val):
            stats['warnings'].append(f"Invalid IMU float value (NaN/Inf) at TS {msg.timestamp_us}")
            break

def analyze_bin_file(bin_file_path):
    """Reads a .bin file from start to finish and generates a detailed validation report."""
    print("\n" + "="*25 + " ANALYSIS REPORT " + "="*25)
    print(f"Analyzing file: {bin_file_path}")

    stats = {
        'channel_counts': defaultdict(int),
        'total_lidar_points': 0,
        'first_timestamp_ms': None,
        'last_timestamp_ms': 0,
        'last_ts': defaultdict(int),
        'warnings': [],
        'errors': []
    }

    try:
        with open(bin_file_path, 'rb') as f:
            while True:
                timestamp_bytes = f.read(8)
                if not timestamp_bytes: break

                stats['last_timestamp_ms'] = int.from_bytes(timestamp_bytes, byteorder='big')
                if stats['first_timestamp_ms'] is None:
                    stats['first_timestamp_ms'] = stats['last_timestamp_ms']

                channel_len = int.from_bytes(f.read(1), byteorder='big')
                channel_name = f.read(channel_len).decode('utf-8', errors='replace')
                msg_len = int.from_bytes(f.read(4), byteorder='big')
                raw_msg_data = f.read(msg_len)
                
                if len(raw_msg_data) != msg_len:
                    stats['errors'].append("Incomplete message read at end of file.")
                    break

                stats['channel_counts'][channel_name] += 1

                # --- Dispatch to appropriate validator ---
                try:
                    if channel_name == "avia_points":
                        packet = livox_pb2.Point_Packet()
                        packet.ParseFromString(raw_msg_data)
                        validate_point_frame(stats, packet)
                    elif channel_name == "avia_imu":
                        msg = livox_pb2.Imu_Data()
                        msg.ParseFromString(raw_msg_data)
                        validate_imu_message(stats, msg)
                except Exception as e:
                    stats['errors'].append(f"CRITICAL: Protobuf parsing failed for channel '{channel_name}': {e}")
                    
    except Exception as e:
        stats['errors'].append(f"A critical error occurred during file reading: {e}")

    # --- Final Report Generation ---
    if stats['first_timestamp_ms'] is None:
        print(f"\n{'='*67}\nERROR: File appears to be empty or in an unknown format.\n{'='*67}")
        return

    duration_s = (stats['last_timestamp_ms'] - stats['first_timestamp_ms']) / 1000.0 if stats['first_timestamp_ms'] else 0
    file_size = os.path.getsize(bin_file_path)
    
    # LiDAR calculations
    lidar_msgs = stats['channel_counts'].get("avia_points", 0)
    actual_point_rate = (stats['total_lidar_points'] / duration_s) if duration_s > 0 else 0
    expected_points = duration_s * config.LIVOX_AVIA_POINTS_PER_SECOND
    lidar_capture_rate = (actual_point_rate / config.LIVOX_AVIA_POINTS_PER_SECOND) * 100 if expected_points > 0 else 0
    
    # IMU calculations
    imu_msgs = stats['channel_counts'].get("avia_imu", 0)
    actual_imu_rate = (imu_msgs / duration_s) if duration_s > 0 else 0
    expected_imu_msgs = duration_s * config.LIVOX_AVIA_IMU_RATE_HZ
    imu_capture_rate = (actual_imu_rate / config.LIVOX_AVIA_IMU_RATE_HZ) * 100 if expected_imu_msgs > 0 else 0

    RED, GREEN, YELLOW, RESET = '\033[91m', '\033[92m', '\033[93m', '\033[0m'
    lidar_color = GREEN if lidar_capture_rate > 95 else RED
    imu_color = GREEN if imu_capture_rate > 95 else RED

    print("\n--- Recording Summary ---")
    print(f"  - File Size:               {human_readable_size(file_size)}")
    print(f"  - Duration:                {duration_s:.2f} seconds")

    print("\n--- Message Counts by Channel ---")
    for name, count in sorted(stats['channel_counts'].items()):
        print(f"  - {name:<25}: {count:>8} messages")
        
    print("\n--- LiDAR Point Cloud Analysis ---")
    print(f"  - Frames Received:         {lidar_msgs:,}")
    print(f"  - Total Points Captured:   {stats['total_lidar_points']:,}")
    print(f"  - Actual Point Rate:       {actual_point_rate:,.1f} pts/sec")
    print(f"  - Data Capture Rate:       {lidar_color}{lidar_capture_rate:.2f}%{RESET}")
    
    print("\n--- IMU Analysis ---")
    print(f"  - Messages Received:       {imu_msgs:,}")
    print(f"  - Actual Message Rate:     {actual_imu_rate:.1f} Hz")
    print(f"  - Data Capture Rate:       {imu_color}{imu_capture_rate:.2f}%{RESET}")
    
    print("\n--- CONCLUSION ---")
    if stats['errors'] or stats['warnings'] or lidar_capture_rate < 95 or imu_capture_rate < 95:
        print(f"{YELLOW}Issues were detected during analysis:{RESET}")
        for err in stats['errors']:
            print(f"  - {RED}ERROR:{RESET} {err}")
        for warn in stats['warnings']:
            print(f"  - {YELLOW}WARNING:{RESET} {warn}")
    else:
        print(f"{GREEN}Data integrity checks passed. File appears to be healthy.{RESET}")

    print("="*67 + "\n")

if __name__ == "__main__":
    bin_files = sorted(glob.glob(os.path.join(config.INPUT_BIN, '*.bin')))
    
    if not bin_files:
        print(f"No .bin files found in the directory: {config.INPUT_BIN}")
    else:
        for file_path in bin_files:
            analyze_bin_file(file_path)