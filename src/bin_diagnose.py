#!/usr/bin/env python3

import os
import glob
from collections import Counter
from common import livox_pb2, config

def human_readable_size(num_bytes):
    for unit in ['B','KB','MB','GB','TB']:
        if num_bytes < 1024.0:
            return f"{num_bytes:3.1f} {unit}"
        num_bytes /= 1024.0
    return f"{num_bytes:.1f} PB"

def analyze_bin_file(bin_file_path):
    """
    Reads a .bin file from start to finish and generates a detailed report.
    """
    print("\n" + "="*25 + " ANALYSIS REPORT " + "="*25)
    print(f"Analyzing file: {bin_file_path}")

    # --- Analysis Variables ---
    channel_counts = Counter()
    total_lidar_points = 0
    avia_points_messages = 0
    first_timestamp_ms = None
    last_timestamp_ms = 0

    try:
        with open(bin_file_path, 'rb') as f:
            while True:
                timestamp_bytes = f.read(8)
                if not timestamp_bytes:
                    break  # End of file

                current_timestamp_ms = int.from_bytes(timestamp_bytes, byteorder='big')
                if first_timestamp_ms is None:
                    first_timestamp_ms = current_timestamp_ms
                last_timestamp_ms = current_timestamp_ms

                channel_len = int.from_bytes(f.read(1), byteorder='big')
                channel_name = f.read(channel_len).decode('utf-8', errors='replace')
                msg_len = int.from_bytes(f.read(4), byteorder='big')
                raw_msg_data = f.read(msg_len)
                
                if len(raw_msg_data) != msg_len:
                    print(f"WARN: Incomplete message read for channel '{channel_name}'. Ending analysis.")
                    break

                channel_counts[channel_name] += 1

                if channel_name == "avia_points":
                    avia_points_messages += 1
                    packet = livox_pb2.Point_Packet()
                    packet.ParseFromString(raw_msg_data)
                    
                    if hasattr(packet, 'num_points'):
                        total_lidar_points += packet.num_points

    except Exception as e:
        print(f"An error occurred during analysis: {e}")
        return

    if first_timestamp_ms is None:
        print(f"{'='*67}\nERROR: File appears to be empty or in an unknown format.\n{'='*67}")
        return

    # Duration, expected points, capture %
    duration_seconds = (last_timestamp_ms - first_timestamp_ms) / 1000.0
    expected_points = duration_seconds * config.LIVOX_AVIA_POINTS_PER_SECOND
    data_capture_rate = (total_lidar_points / expected_points) * 100 if expected_points > 0 else 0
    actual_point_rate = (total_lidar_points / duration_seconds) if duration_seconds > 0 else 0
    avg_points_per_packet = (total_lidar_points / avia_points_messages) if avia_points_messages > 0 else 0

    RED, GREEN, RESET = '\033[91m', '\033[92m', '\033[0m'
    color = GREEN if data_capture_rate > 80 else RED

    # --- Final Report ---
    file_size_bytes = os.path.getsize(bin_file_path)

    print("\n--- Recording Summary ---")
    print(f"  - File Size:               {human_readable_size(file_size_bytes)}")
    print(f"  - Duration:                {duration_seconds:.2f} seconds")

    print("\n--- Message Counts by Channel ---")
    for name, count in sorted(channel_counts.items()):
        print(f"  - {name:<25}: {count:>8} messages")
        
    print("\n--- LiDAR Point Cloud Analysis ---")
    print(f"  - avia_points messages:    {avia_points_messages:,}")
    print(f"  - Total LiDAR Points:      {total_lidar_points:,}")
    print(f"  - Avg Points Per Packet:   {avg_points_per_packet:,.1f} points")
    print(f"  - Actual Point Rate:       {actual_point_rate:,.1f} pts/sec")
    print(f"  - Expected Point Rate:     {config.LIVOX_AVIA_POINTS_PER_SECOND:,} pts/sec")
    print(f"  - Data Capture Rate:       {color}{data_capture_rate:.2f}%{RESET}")

    print("\n--- CONCLUSION ---")
    if data_capture_rate < 80:
        print(f"{RED}Significant data loss detected: {100-data_capture_rate:.2f}% missing.{RESET}")
    else:
        print(f"{GREEN}Data capture successful and within expected range.{RESET}")

    print("="*67 + "\n")

if __name__ == "__main__":
    input_directory = config.INPUT_ROSBAG_BIN
    bin_files = sorted(glob.glob(os.path.join(input_directory, '*.bin')))
    
    if not bin_files:
        print(f"No .bin files found in the directory: {input_directory}")
    else:
        for file_path in bin_files:
            analyze_bin_file(file_path)
