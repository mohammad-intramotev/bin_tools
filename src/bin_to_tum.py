#!/usr/bin/env python3

import os
import sys
import math
from collections import deque, Counter
from pyproj import Transformer

from common import config, navigation_pb2, orientation_pb2, transformations as tf_transformations


# --- GPS Coordinate System ---
transformer = Transformer.from_crs(config.IN_PROJ, config.OUT_PROJ, always_xy=True)

# Buffers and state
gps_position_buffer = deque(maxlen=100)
gps_orientation_buffer = deque(maxlen=100)
gps_origin_xyz = None
channel_counter = Counter()
rtk_fix_status_counter = Counter()


def parse_duro_llh(raw_bytes, ts_ms):
    try:
        packet = navigation_pb2.MsgPosLlh()
        packet.ParseFromString(raw_bytes)
        fix_mode = packet.flags & 0b111
        is_rtk_fixed = fix_mode == 2 or (fix_mode == 3)
        rtk_fix_status_counter[fix_mode] += 1
        return {
            'logger_time_ms': ts_ms,
            'lat': packet.lat,
            'lon': packet.lon,
            'alt': packet.height,
            'is_rtk_fixed': is_rtk_fixed
        }
    except Exception:
        return None


def parse_duro_orient_euler(raw_bytes, ts_ms):
    try:
        packet = orientation_pb2.MsgOrientEuler()
        packet.ParseFromString(raw_bytes)
        q = tf_transformations.quaternion_from_euler(
            math.radians(packet.roll / 1e6),
            math.radians(packet.pitch / 1e6),
            math.radians(packet.yaw / 1e6)
        )
        return {'logger_time_ms': ts_ms, 'orientation_q': q}
    except Exception:
        return None


def try_create_ground_truth_point(gt_file_writer):
    global gps_origin_xyz
    if not gps_position_buffer or not gps_orientation_buffer:
        if not hasattr(try_create_ground_truth_point, "buffer_warn_printed"):
            print("DEBUG: Attempting to fuse, but at least one data buffer is empty.")
            try_create_ground_truth_point.buffer_warn_printed = True
        return 0

    pos_data = gps_position_buffer.popleft()
    best_orient_data = None
    min_time_delta = float('inf')

    for orient_data in gps_orientation_buffer:
        time_delta = abs(pos_data['logger_time_ms'] - orient_data['logger_time_ms'])
        if time_delta < min_time_delta:
            min_time_delta = time_delta
        if time_delta <= config.MAX_GPS_FUSION_AGE_MS:
            best_orient_data = orient_data
            break

    if best_orient_data:
        if not pos_data['is_rtk_fixed']:
            if not hasattr(try_create_ground_truth_point, "rtk_warn_printed"):
                print("DEBUG: Found a time-synced GPS pair, but the position data is NOT RTK Fixed. Skipping as it's not ground truth.")
                try_create_ground_truth_point.rtk_warn_printed = True
            return 0

        x, y = transformer.transform(pos_data['lon'], pos_data['lat'])
        z = pos_data['alt']

        if gps_origin_xyz is None:
            gps_origin_xyz = (x, y, z)
            print(f"DEBUG: Set ground truth origin (UTM): {gps_origin_xyz}")

        tx, ty, tz = x - gps_origin_xyz[0], y - gps_origin_xyz[1], z - gps_origin_xyz[2]
        qx, qy, qz, qw = best_orient_data['orientation_q']
        timestamp = pos_data['logger_time_ms'] / 1000.0

        gt_file_writer.write(f"{timestamp:.6f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
        return 1
    else:
        if not hasattr(try_create_ground_truth_point, "time_warn_printed"):
            print(f"DEBUG: Could not find a time-synced orientation message for a position message.")
            print(f"       The smallest time difference found was {min_time_delta} ms, which is > constants.MAX_GPS_FUSION_AGE_MS ({config.MAX_GPS_FUSION_AGE_MS} ms).")
            try_create_ground_truth_point.time_warn_printed = True
    return 0


def main():
    if not os.path.exists(config.INPUT_TUM_BIN):
        print(f"Error: Input .bin file not found at: {config.INPUT_TUM_BIN}")
        sys.exit(1)

    print(f"--- Running Ground Truth Diagnostics ---")
    print(f"Input: {config.INPUT_TUM_BIN}")

    gt_file = open(config.OUTPUT_TUM_FILE, 'w')
    pos_parsed_count = 0
    orient_parsed_count = 0
    total_gt_points = 0

    with open(config.INPUT_TUM_BIN, 'rb') as log_file:
        while True:
            header_bytes = log_file.read(9)
            if not header_bytes:
                break
            ts_ms = int.from_bytes(header_bytes[0:8], byteorder='big')
            channel_len = header_bytes[8]

            channel_name = log_file.read(channel_len).decode('utf-8', errors='replace')
            channel_counter[channel_name] += 1

            msg_len_bytes = log_file.read(4)
            if not msg_len_bytes:
                break
            msg_len = int.from_bytes(msg_len_bytes, byteorder='big')

            raw_msg_data = log_file.read(msg_len)
            if len(raw_msg_data) != msg_len:
                break

            if channel_name == config.GPS_ORIENTATION_CHANNEL:
                parsed = parse_duro_orient_euler(raw_msg_data, ts_ms)
                if parsed:
                    gps_orientation_buffer.append(parsed)
                    orient_parsed_count += 1

            elif channel_name == config.GPS_POSITION_CHANNEL:
                parsed = parse_duro_llh(raw_msg_data, ts_ms)
                if parsed:
                    gps_position_buffer.append(parsed)
                    pos_parsed_count += 1
                    total_gt_points += try_create_ground_truth_point(gt_file)

    gt_file.close()

    print("\n--- Diagnostics Report ---")
    print("\n[1] Channels found in .bin file:")
    for name, count in channel_counter.items():
        print(f"  - '{name}': {count} messages")

    print("\n[2] Parsed GPS Message Counts:")
    print(f"  - Position messages ('{config.GPS_POSITION_CHANNEL}'): {pos_parsed_count} parsed")
    print(f"  - Orientation messages ('{config.GPS_ORIENTATION_CHANNEL}'): {orient_parsed_count} parsed")

    print("\n[3] RTK Fix Status Counts (for parsed position messages):")
    if not rtk_fix_status_counter:
        print("  - No position messages with status flags were parsed.")
    else:
        print(f"  - RTK Fixed (mode=2): {rtk_fix_status_counter.get(2, 0)} points")
        print(f"  - RTK Float (mode=1): {rtk_fix_status_counter.get(1, 0)} points")
        print(f"  - No Solution (mode=0): {rtk_fix_status_counter.get(0, 0)} points")
        for mode, count in rtk_fix_status_counter.items():
            if mode not in [0, 1, 2]:
                print(f"  - Other (mode={mode}): {count} points")

    print("\n--- Final Result ---")
    print(f"Ground truth creation complete! Wrote {total_gt_points} valid RTK points to {config.OUTPUT_TUM_FILE}")


if __name__ == "__main__":
    main()