#!/usr/bin/env python3

import os
import glob
import math
from collections import defaultdict
from common import livox_pb2, config

def human_readable_size(num_bytes):
    if num_bytes is None:
        return "N/A"
    for unit in ['B','KB','MB','GB','TB']:
        if num_bytes < 1024.0:
            return f"{num_bytes:3.1f} {unit}"
        num_bytes /= 1024.0
    return f"{num_bytes:.1f} PB"

def validate_point_frame(stats, packet):
    """Validates the internal consistency of a single Point_Packet frame."""
    num_points = packet.num_points
    x_coords_len = len(packet.x_coords)

    stats['points_per_packet'][num_points] += 1

    if num_points != x_coords_len:
        stats['warnings'].append(
            f"Point count mismatch in frame at TS {packet.timestamp_us}. "
            f"Header: {num_points}, Actual: {x_coords_len}"
        )

    if packet.timestamp_us <= stats['last_ts']['avia_points']:
        stats['warnings'].append(
            f"Non-monotonic LiDAR hardware timestamp detected at {packet.timestamp_us}"
        )
    stats['last_ts']['avia_points'] = packet.timestamp_us

    if packet.system_timestamp_us <= stats['last_ts']['avia_points_system']:
        stats['warnings'].append(
            f"Non-monotonic LiDAR system timestamp detected at {packet.system_timestamp_us}"
        )
    stats['last_ts']['avia_points_system'] = packet.system_timestamp_us

    if packet.timestamp_offset_us:
        last_offset_us = packet.timestamp_offset_us[-1]
        stats['frame_durations_ms'].append(last_offset_us / 1000.0)
        expected_us = config.LIVOX_AVIA_FRAME_TIME_MS * 1000
        if not (0.95 * expected_us < last_offset_us < 1.05 * expected_us):
            stats['warnings'].append(
                f"Unusual frame duration. Last offset: {last_offset_us}us, "
                f"Expected: ~{expected_us}us"
            )

    if packet.reflectivity and not (0 <= packet.reflectivity[0] <= 255):
        stats['warnings'].append(
            f"Invalid reflectivity value found: {packet.reflectivity[0]}"
        )
    
    if hasattr(packet, 'raw_packet_count') and packet.raw_packet_count > 0:
        stats['total_raw_packets'] += packet.raw_packet_count

    stats['total_lidar_points'] += num_points
    stats['total_lidar_frames'] += 1

def validate_imu_message(stats, msg):
    """Validates the internal consistency of a single Imu_Data message."""
    if stats['last_ts']['avia_imu'] > 0:
        delta_us = msg.timestamp_us - stats['last_ts']['avia_imu']
        stats['imu_delta_t_ms'].append(delta_us / 1000.0)
    
    if msg.timestamp_us <= stats['last_ts']['avia_imu']:
        stats['warnings'].append(
            f"Non-monotonic IMU hardware timestamp detected at {msg.timestamp_us}"
        )
    stats['last_ts']['avia_imu'] = msg.timestamp_us

    if msg.system_timestamp_us <= stats['last_ts']['avia_imu_system']:
        stats['warnings'].append(
            f"Non-monotonic IMU system timestamp detected at {msg.system_timestamp_us}"
        )
    stats['last_ts']['avia_imu_system'] = msg.system_timestamp_us

    for val in [msg.gyro_x, msg.gyro_y, msg.gyro_z, msg.acc_x, msg.acc_y, msg.acc_z]:
        if math.isnan(val) or math.isinf(val):
            stats['warnings'].append(
                f"Invalid IMU float value (NaN/Inf) at TS {msg.timestamp_us}"
            )
            break

def analyze_bin_file(bin_file_path):
    """Reads a .bin file from start to finish and generates a detailed validation report."""
    print("\n" + "="*25 + " ANALYSIS REPORT " + "="*25)
    print(f"Analyzing file: {bin_file_path}")

    stats = {
        'channel_counts': defaultdict(int),
        'total_lidar_points': 0,
        'total_lidar_frames': 0,
        'total_raw_packets': 0,
        'first_timestamp_ms': None,
        'last_timestamp_ms': 0,
        'last_ts': defaultdict(int),
        'warnings': [],
        'errors': [],
        'points_per_packet': defaultdict(int),
        'frame_durations_ms': [],
        'imu_delta_t_ms': []
    }

    try:
        with open(bin_file_path, 'rb') as f:
            while True:
                timestamp_bytes = f.read(8)
                if not timestamp_bytes:
                    break
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
                    stats['errors'].append(
                        f"CRITICAL: Protobuf parsing failed for channel '{channel_name}': {e}"
                    )
    except Exception as e:
        stats['errors'].append(f"A critical error occurred during file reading: {e}")

    # Final Report Generation
    if stats['first_timestamp_ms'] is None:
        print(f"\n{'='*67}\nERROR: File appears to be empty or in an unknown format.\n{'='*67}")
        return

    duration_s = (stats['last_timestamp_ms'] - stats['first_timestamp_ms']) / 1000.0 if stats['last_timestamp_ms'] > stats['first_timestamp_ms'] else 0
    file_size = os.path.getsize(bin_file_path)

    print("\n--- Recording Summary ---")
    print(f" - File Size: {human_readable_size(file_size)}")
    print(f" - Duration: {duration_s:.2f} seconds")

    print("\n--- Message Counts & Refresh Rates by Channel ---")
    for name, count in sorted(stats['channel_counts'].items()):
        rate = (count / duration_s) if duration_s > 0 else 0
        print(f" - {name:<25}: {count:>8} msgs, {rate:>6.2f} Hz")

    print("\n--- LiDAR Point Cloud Analysis (Aggregated Frames) ---")
    print(f" - Frames Published: {stats['total_lidar_frames']:,}")
    print(f" - Total Points Captured: {stats['total_lidar_points']:,}")
    print(f" - Frame Refresh Rate: {stats['total_lidar_frames'] / duration_s if duration_s > 0 else 0:,.2f} Hz")
    print(f" - Actual Point Rate: {stats['total_lidar_points'] / duration_s if duration_s > 0 else 0:,.1f} pts/sec")

    print("\n--- Frame Duration Analysis ---")
    if stats['frame_durations_ms']:
        durations = stats['frame_durations_ms']
        avg_dur = sum(durations) / len(durations)
        min_dur = min(durations)
        max_dur = max(durations)
        print(f" - Target Frame Duration:    {config.LIVOX_AVIA_FRAME_TIME_MS:.1f} ms")
        print(f" - Average Frame Duration:   {avg_dur:.2f} ms")
        print(f" - Minimum Frame Duration:   {min_dur:.2f} ms")
        print(f" - Maximum Frame Duration:   {max_dur:.2f} ms")
    else:
        print(" - No frame duration data to analyze.")

    print("\n--- IMU Timing Analysis ---")
    if stats['imu_delta_t_ms']:
        deltas = stats['imu_delta_t_ms']
        avg_delta = sum(deltas) / len(deltas)
        min_delta = min(deltas)
        max_delta = max(deltas)
        std_dev = math.sqrt(sum((x - avg_delta) ** 2 for x in deltas) / len(deltas))
        expected_delta = 1000.0 / config.LIVOX_AVIA_IMU_RATE_HZ

        print(f" - Target Interval:          {expected_delta:.2f} ms ({config.LIVOX_AVIA_IMU_RATE_HZ} Hz)")
        print(f" - Average Interval:         {avg_delta:.3f} ms")
        print(f" - Minimum Interval:         {min_delta:.3f} ms")
        print(f" - Maximum Interval:         {max_delta:.3f} ms")
        print(f" - Standard Deviation:       {std_dev:.4f} ms")
    else:
        print(" - No IMU data to analyze for timing.")

    print("\n--- Packet Point-Count Distribution (Aggregated Frames) ---")
    for count, freq in sorted(stats['points_per_packet'].items()):
        print(f"  {count:5d} points : {freq:4d} packets")

    print("\n--- CONCLUSION ---")
    if stats['errors'] or stats['warnings']:
        print(f"\033[93mIssues were detected during analysis:\033[0m")
        for err in stats['errors']:
            print(f" - \033[91mERROR:\033[0m {err}")
        for warn in stats['warnings']:
            print(f" - \033[93mWARNING:\033[0m {warn}")
    else:
        print(f"\032[92mData integrity checks passed. File appears to be healthy.\033[0m")

    print("="*67 + "\n")

if __name__ == "__main__":
    bin_files = sorted(glob.glob(os.path.join(config.INPUT_BIN, '*.bin')))
    if not bin_files:
        print(f"No .bin files found in the directory: {config.INPUT_BIN}")
    else:
        for file_path in bin_files:
            analyze_bin_file(file_path)
