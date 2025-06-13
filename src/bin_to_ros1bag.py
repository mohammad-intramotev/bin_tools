#!/usr/bin/env python3

import glob
import os
import sys
from collections import Counter

import rospy
import rosbag

from livox_ros_driver.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import Imu

from common import config, livox_pb2


def parse_livox_point_packet(raw_bytes, bin_envelope_timestamp_ms):
    """
    Parses a Livox Point_Packet protobuf and uses its internal hardware timestamp.
    """
    try:
        packet = livox_pb2.Point_Packet()
        packet.ParseFromString(raw_bytes)
        points_data_for_ros = []

        try:
            num_points = packet.num_points
            if not (len(packet.x_coords) == num_points and
                    len(packet.y_coords) == num_points and
                    len(packet.z_coords) == num_points and
                    len(packet.reflectivity) == num_points and
                    len(packet.timestamp_offset_us) == num_points):
                proto_ts_debug = getattr(packet, 'system_timestamp_us', 'N/A')
                print(f"DEBUG Points: Point_Packet field length mismatch for proto_ts {proto_ts_debug}. Num_points: {num_points}. Skipping packet.")
                return None

            for i in range(num_points):
                x = packet.x_coords[i] / 1000.0
                y = packet.y_coords[i] / 1000.0
                z = packet.z_coords[i] / 1000.0
                intensity = float(packet.reflectivity[i])
                point_time_offset_sec = packet.timestamp_offset_us[i] / 1_000_000.0
                points_data_for_ros.append([x, y, z, intensity, point_time_offset_sec])

        except AttributeError:
            for point in packet.list_points:
                x = point.x_coord / 1000.0
                y = point.y_coord / 1000.0
                z = point.z_coord / 1000.0
                intensity = float(point.reflectivity)
                point_time_offset_sec = point.timestamp_offset_us / 1_000_000.0
                points_data_for_ros.append([x, y, z, intensity, point_time_offset_sec])
        
        if not hasattr(packet, 'timestamp_us'):
             print("ERROR: Livox Point_Packet missing 'timestamp_us'. Cannot get accurate time.", file=sys.stderr)
             return None
        
        header_time_us = packet.timestamp_us

        header_time_sec = header_time_us / 1_000_000.0

        sec_hdr = int(header_time_sec)
        nanosec_hdr = int((header_time_sec - sec_hdr) * 1_000_000_000)

        return {
            'sec': sec_hdr,
            'nanosec': nanosec_hdr,
            'points_data': points_data_for_ros
        }
    except Exception as e:
        print(f"Error parsing Livox Point_Packet: {e}", file=sys.stderr)
        return None


def create_livox_custom_msg(parsed_data, frame_id):
    """
    Creates a livox_ros_driver/CustomMsg message from parsed point data.
    """
    if not parsed_data or not parsed_data['points_data']:
        return None

    msg = CustomMsg()
    msg.header.stamp = rospy.Time(parsed_data['sec'], parsed_data['nanosec'])
    msg.header.frame_id = frame_id
    msg.timebase = parsed_data['sec'] * 1_000_000_000 + parsed_data['nanosec']
    points_data = parsed_data['points_data']
    msg.point_num = len(points_data)
    msg.lidar_id = 0
    msg.rsvd = [0, 0, 0]

    for point_raw in points_data:
        p = CustomPoint()
        x, y, z, intensity, time_offset_sec = point_raw
        p.x, p.y, p.z = x, y, z
        p.reflectivity = int(intensity)
        p.offset_time = int(time_offset_sec * 1_000_000_000) # offset_time is in nanoseconds
        p.tag, p.line = 0, 0
        msg.points.append(p)

    return msg


def parse_livox_imu_data(raw_bytes, bin_envelope_timestamp_ms):
    """
    Parses Livox Imu_Data protobuf and uses its internal hardware timestamp.
    """
    try:
        packet = livox_pb2.Imu_Data()
        packet.ParseFromString(raw_bytes)

        if not hasattr(packet, 'timestamp_us'):
            print("ERROR: Livox Imu_Data missing 'timestamp_us'. Cannot get accurate time.", file=sys.stderr)
            return None

        header_time_us = packet.timestamp_us

        header_time_sec = header_time_us / 1_000_000.0

        header_sec = int(header_time_sec)
        header_nanosec = int((header_time_sec - header_sec) * 1_000_000_000)

        angular_velocity_flu = [packet.gyro_x, -packet.gyro_y, packet.gyro_z]
        linear_acceleration_flu = [
            packet.acc_x * config.GRAVITY_ACCEL,
            -packet.acc_y * config.GRAVITY_ACCEL,
            packet.acc_z * config.GRAVITY_ACCEL
        ]

        return {
            'header_sec': header_sec,
            'header_nanosec': header_nanosec,
            'angular_velocity': angular_velocity_flu,
            'linear_acceleration': linear_acceleration_flu
        }
    except Exception as e:
        print(f"Error parsing Livox Imu_Data: {e}", file=sys.stderr)
        return None

def create_livox_only_imu_msg(livox_data, frame_id):
    """
    Creates a sensor_msgs/Imu message using only data from the Livox IMU packet.
    """
    if not livox_data:
        return None

    msg = Imu()
    msg.header.stamp = rospy.Time(livox_data['header_sec'], livox_data['header_nanosec'])
    msg.header.frame_id = frame_id
    msg.orientation.w = 1.0
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = livox_data['angular_velocity']
    msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = msg.angular_velocity_covariance[8] = 0.01
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = livox_data['linear_acceleration']
    msg.linear_acceleration_covariance[0] = msg.linear_acceleration_covariance[4] = msg.linear_acceleration_covariance[8] = 0.01

    return msg

CHANNEL_PARSERS = {
    "avia_points": parse_livox_point_packet,
    "avia_imu": parse_livox_imu_data,
}

def create_ros1_bag(bin_file_path, output_bag_path):
    if os.path.exists(output_bag_path):
        print(f"{output_bag_path} already exists. Removing it.")
        try:
            os.remove(output_bag_path)
        except OSError as e:
            print(f"Error removing existing file: {e}. Please check permissions.", file=sys.stderr)
            return

    bag_writer = rosbag.Bag(output_bag_path, 'w')
    channel_counts = Counter()

    last_imu_time_sec = None
    imu_time_gap_threshold_sec = 0.02
    lidar_scan_count = 0
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RESET = '\033[0m'

    print(f"Processing .bin file: {bin_file_path} into {output_bag_path}")
    print("--- Starting Advanced Diagnostics (using timestamp_us field) ---")

    with open(bin_file_path, 'rb') as log_file:
        while True:
            try:
                timestamp_bytes = log_file.read(8)
                if not timestamp_bytes: break
                bin_envelope_timestamp_ms = int.from_bytes(timestamp_bytes, byteorder='big')
                channel_len_byte = log_file.read(1)
                if not channel_len_byte: break
                channel_len = int.from_bytes(channel_len_byte, byteorder='big')
                channel_name_bytes = log_file.read(channel_len)
                if not channel_name_bytes: break
                channel_name = channel_name_bytes.decode('utf-8', errors='replace')
                channel_counts[channel_name] += 1
                msg_len_bytes = log_file.read(4)
                if not msg_len_bytes: break
                msg_len = int.from_bytes(msg_len_bytes, byteorder='big')
                raw_msg_data = log_file.read(msg_len)
                if len(raw_msg_data) != msg_len:
                    print(f"Warning: Truncated message for channel {channel_name}", file=sys.stderr)
                    break
            except (EOFError, IndexError):
                break
            except Exception as e:
                print(f"Error reading from .bin file {bin_file_path}: {e}", file=sys.stderr)
                break

            if channel_name not in CHANNEL_PARSERS:
                continue

            parser_func = CHANNEL_PARSERS[channel_name]
            parsed_data = parser_func(raw_msg_data, bin_envelope_timestamp_ms)

            if not parsed_data:
                continue
            
            if channel_name == "avia_points":
                ros_msg = create_livox_custom_msg(parsed_data, config.LIDAR_FRAME_ID)
                if not ros_msg: continue

                lidar_scan_count += 1
                header_time_sec = ros_msg.header.stamp.to_sec()

                if ros_msg.point_num > 0:
                    offset_times_sec = [p.offset_time / 1e9 for p in ros_msg.points]
                    scan_start_time_sec = header_time_sec + min(offset_times_sec)
                    scan_end_time_sec = header_time_sec + max(offset_times_sec)
                    scan_duration_ms = (scan_end_time_sec - scan_start_time_sec) * 1000
                else:
                    scan_start_time_sec, scan_end_time_sec, scan_duration_ms = header_time_sec, header_time_sec, 0
                
                print(f"\nDIAGNOSTIC [LiDAR Scan #{lidar_scan_count}]:")
                print(f"  - Scan Time Range:  {scan_start_time_sec:.6f} -> {scan_end_time_sec:.6f} (Duration: {scan_duration_ms:.2f} ms)")
                
                imu_status_time = last_imu_time_sec if last_imu_time_sec is not None else 0
                if imu_status_time > 0:
                    coverage_ms = (imu_status_time - scan_end_time_sec) * 1000
                    color = RED if coverage_ms < 0 else GREEN
                    print(f"  - Latest IMU Time:  {imu_status_time:.6f}")
                    print(f"  - {color}Coverage (IMU latest - Scan end): {coverage_ms:+.2f} ms{RESET}")
                    if coverage_ms < 0:
                        print(f"  - {RED}CRITICAL: IMU data does not cover the full LiDAR scan! LIO sync will likely fail.{RESET}")
                else:
                    print(f"  - {YELLOW}WARNING: No IMU messages received yet.{RESET}")

                bag_writer.write(config.LIDAR_TOPIC, ros_msg, ros_msg.header.stamp)

            elif channel_name == "avia_imu":
                imu_msg = create_livox_only_imu_msg(parsed_data, config.IMU_FRAME_ID)
                if not imu_msg: continue

                current_imu_time_sec = imu_msg.header.stamp.to_sec()
                if last_imu_time_sec is not None:
                    time_diff = current_imu_time_sec - last_imu_time_sec
                    if time_diff > imu_time_gap_threshold_sec:
                        print(f"  - {YELLOW}WARNING: Large gap *between* IMU messages detected: {time_diff:.4f} seconds{RESET}")
                
                last_imu_time_sec = current_imu_time_sec
                bag_writer.write(config.IMU_TOPIC, imu_msg, imu_msg.header.stamp)

    print(f"\n--- Diagnostics Complete ---")
    print("Channel distribution:")
    for name, count_val in channel_counts.items():
        print(f"  {name}: {count_val} messages")
    
    bag_writer.close()
    print(f"Finished. Wrote {sum(channel_counts.values())} total ROS messages into {output_bag_path}\n")

def process_bin_path(input_path, output_base_dir):
    bin_files = sorted(glob.glob(os.path.join(input_path, '*.bin')))
    if not bin_files:
        print(f"No .bin files found in {input_path}")
        return
        
    print(f"Found {len(bin_files)} .bin file(s) in directory: {input_path}")
    for bin_file in bin_files:
        base_name = os.path.splitext(os.path.basename(bin_file))[0]
        output_bag_path = os.path.join(output_base_dir, base_name + ".bag")
        create_ros1_bag(bin_file, output_bag_path)

if __name__ == "__main__":
    if not os.path.exists(config.OUTPUT_ROSBAGS):
        os.makedirs(config.OUTPUT_ROSBAGS)

    process_bin_path(config.INPUT_ROSBAG_BIN, config.OUTPUT_ROSBAGS)
    print("Conversion to ROS 1 bag complete.")