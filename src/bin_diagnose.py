#!/usr/bin/env python3

import math
from collections import defaultdict
from pathlib import Path

from common import livox_pb2, config

def human_readable_size(num_bytes):
    """Converts bytes to a human-readable format."""
    if num_bytes is None:
        return "N/A"
    for unit in ['B','KB','MB','GB','TB']:
        if num_bytes < 1024.0:
            return f"{num_bytes:3.1f} {unit}"
        num_bytes /= 1024.0
    return f"{num_bytes:.1f} PB"

def validate_point_frame(stats, packet, file_timestamps_ms):
    """Validates the internal consistency of a single Point_Packet frame."""
    num_points = packet.num_points
    x_coords_len = len(packet.x_coords)
    stats['points_per_packet'][num_points] += 1

    relative_time_s = (file_timestamps_ms - stats['first_timestamp_ms']) / 1000
    log_entry = (f" Time: {relative_time_s:8.2f}s | {num_points:5} points in packet")
    stats['point_packets_log'].append(log_entry)

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

def analyze_bin_file(bin_path: Path):
    """Reads a .bin file and writes a detailed validation report to a .txt alongside it."""
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
        'imu_delta_t_ms': [],
        'point_packets_log': []
    }
    lines = []
    lines.append("="*25 + " ANALYSIS REPORT " + "="*25)
    lines.append(f"Analyzing file: {bin_path}")

    try:
        with bin_path.open('rb') as f:
            while True:
                timestamp_bytes = f.read(8)
                if not timestamp_bytes:
                    break
                stats['last_timestamp_ms'] = int.from_bytes(timestamp_bytes, 'big')
                if stats['first_timestamp_ms'] is None:
                    stats['first_timestamp_ms'] = stats['last_timestamp_ms']

                chan_len = int.from_bytes(f.read(1), 'big')
                channel = f.read(chan_len).decode('utf-8', errors='replace')
                msg_len = int.from_bytes(f.read(4), 'big')
                raw = f.read(msg_len)
                if len(raw) != msg_len:
                    stats['errors'].append("Incomplete message read at end of file.")
                    break

                stats['channel_counts'][channel] += 1
                try:
                    if channel == "avia_points":
                        pkt = livox_pb2.Point_Packet()
                        pkt.ParseFromString(raw)
                        validate_point_frame(stats, pkt, stats['last_timestamp_ms'])
                    elif channel == "avia_imu":
                        imu = livox_pb2.Imu_Data()
                        imu.ParseFromString(raw)
                        validate_imu_message(stats, imu)
                except Exception as e:
                    stats['errors'].append(
                        f"CRITICAL: Protobuf parsing failed for '{channel}': {e}"
                    )
    except Exception as e:
        stats['errors'].append(f"Critical error during file read: {e}")

    # If file empty or unreadable:
    if stats['first_timestamp_ms'] is None:
        lines.append("="*67)
        lines.append("ERROR: File appears empty or in unknown format.")
        lines.append("="*67)
    else:
        # Summaries
        dur_s = max(0, (stats['last_timestamp_ms'] - stats['first_timestamp_ms']) / 1000.0)
        size = bin_path.stat().st_size
        lines.append("\n--- Recording Summary ---")
        lines.append(f" - File Size: {human_readable_size(size)}")
        lines.append(f" - Duration: {dur_s:.2f} seconds")

        lines.append("\n--- Message Counts & Refresh Rates by Channel ---")
        for name, cnt in sorted(stats['channel_counts'].items()):
            rate = (cnt / dur_s) if dur_s > 0 else 0
            lines.append(f" - {name:<25}: {cnt:>8} msgs, {rate:>6.2f} Hz")

        # Data loss
        lines.append("\n--- Data Loss & Integrity Analysis ---")
        if dur_s > 0:
            exp_pts = config.LIVOX_AVIA_POINT_RATE * dur_s
            act_pts = stats['total_lidar_points']
            drop = max(0, int(exp_pts - act_pts))
            pct = (drop / exp_pts * 100) if exp_pts>0 else 0
            lines.append(f" - Expected Point Rate:      {config.LIVOX_AVIA_POINT_RATE:,} pts/sec")
            lines.append(f" - Expected Points in Total: {int(exp_pts):,}")
            lines.append(f" - Actual Points Captured:   {act_pts:,}")
            lines.append(f" - Estimated Dropped Points: {drop:,}")
            lines.append(f" - Percentage Dropped:       {pct:.2f}%")
            lines.append("-"*40)
            lines.append(f" - Actual Avg Point Rate:    {act_pts / dur_s:,.1f} pts/sec")
            lines.append(f" - Frame Refresh Rate:       {stats['total_lidar_frames'] / dur_s:,.2f} Hz")
        else:
            lines.append(" - Not enough data for loss analysis.")

        # Frame durations
        lines.append("\n--- Frame Duration Analysis ---")
        if stats['frame_durations_ms']:
            dur = stats['frame_durations_ms']
            lines += [
                f" - Target Frame Duration:    {config.LIVOX_AVIA_FRAME_TIME_MS:.1f} ms",
                f" - Average Frame Duration:   {sum(dur)/len(dur):.2f} ms",
                f" - Minimum Frame Duration:   {min(dur):.2f} ms",
                f" - Maximum Frame Duration:   {max(dur):.2f} ms",
            ]
        else:
            lines.append(" - No frame duration data to analyze.")

        # IMU timing
        lines.append("\n--- IMU Timing Analysis ---")
        if stats['imu_delta_t_ms']:
            d = stats['imu_delta_t_ms']
            avg = sum(d)/len(d)
            std = math.sqrt(sum((x-avg)**2 for x in d)/len(d))
            exp_dt = 1000.0/config.LIVOX_AVIA_IMU_RATE_HZ
            lines += [
                f" - Target Interval:          {exp_dt:.2f} ms ({config.LIVOX_AVIA_IMU_RATE_HZ} Hz)",
                f" - Average Interval:         {avg:.3f} ms",
                f" - Minimum Interval:         {min(d):.3f} ms",
                f" - Maximum Interval:         {max(d):.3f} ms",
                f" - Standard Deviation:       {std:.4f} ms",
            ]
        else:
            lines.append(" - No IMU data to analyze for timing.")
        
        # LiDAR Packet Log
        lines.append("\n--- LiDAR Packet Log ---")
        if stats['point_packets_log']:
            log_entries = stats['point_packets_log']
            lines.append(f"Timestamp and point count for all {len(log_entries)} LiDAR packets:")
            lines.extend(log_entries)
        else:
            lines.append(" - No LiDAR packets were found in the log.")
    
        # Distribution
        lines.append("\n--- Packet Point-Count Distribution ---")
        for c, f in sorted(stats['points_per_packet'].items()):
            lines.append(f"  {c:5d} points : {f:4d} packets")

        # Conclusion
        lines.append("\n--- CONCLUSION ---")
        if stats['errors'] or stats['warnings']:
            lines.append("Issues were detected during analysis:")
            for e in stats['errors']:
                lines.append(f" - ERROR: {e}")
            for w in stats['warnings']:
                lines.append(f" - WARNING: {w}")
        else:
            lines.append("Data integrity checks passed. File appears to be healthy.")

    # Write out to .txt
    out_path = bin_path.with_suffix('.txt')
    out_path.write_text("\n".join(lines))
    # Optional: return the path for further use
    return out_path

if __name__ == "__main__":
    bin_dir = Path(config.INPUT_BIN)
    bin_files = sorted(bin_dir.glob('*.bin'))
    if not bin_files:
        print(f"No .bin files found in directory: {bin_dir}")
    else:
        for bf in bin_files:
            report = analyze_bin_file(bf)
            print(f"Wrote report to: {report}")
