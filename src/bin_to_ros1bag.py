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
    Parses a Livox Point_Packet protobuf and also tracks unique line and tag values.
    """
    try:
        packet = livox_pb2.Point_Packet()
        packet.ParseFromString(raw_bytes)
        points_data_for_ros = []
        
        # --- DIAGNOSTICS: Sets to track unique values within this packet ---
        unique_lines_in_packet = set()
        unique_tags_in_packet = set()

        # This block handles the flat-array protobuf format
        if hasattr(packet, 'x_coords'):
            num_points = packet.num_points
            if not (len(packet.x_coords) == num_points and
                    len(packet.y_coords) == num_points and
                    len(packet.z_coords) == num_points and
                    len(packet.reflectivity) == num_points and
                    len(packet.timestamp_offset_us) == num_points):
                proto_ts_debug = getattr(packet, 'system_timestamp_us', 'N/A')
                print(f"DEBUG Points: Point_Packet field length mismatch for proto_ts {proto_ts_debug}. Num_points: {num_points}. Skipping packet.")
                return None

            # Check for tag and line fields, which may or may not exist in the proto definition
            has_tag = hasattr(packet, 'tag') and len(packet.tag) == num_points
            has_line = hasattr(packet, 'line') and len(packet.line) == num_points

            for i in range(num_points):
                x = packet.x_coords[i] / 1000.0
                y = packet.y_coords[i] / 1000.0
                z = packet.z_coords[i] / 1000.0
                intensity = float(packet.reflectivity[i])
                point_time_offset_sec = packet.timestamp_offset_us[i] / 1_000_000.0
                
                tag = packet.tag[i] if has_tag else 0
                line = packet.line[i] if has_line else 0
                
                points_data_for_ros.append([x, y, z, intensity, point_time_offset_sec, tag, line])
                unique_tags_in_packet.add(tag)
                unique_lines_in_packet.add(line)

        # This block handles the older list-of-point-structures format
        elif hasattr(packet, 'list_points'):
            for point in packet.list_points:
                x = point.x_coord / 1000.0
                y = point.y_coord / 1000.0
                z = point.z_coord / 1000.0
                intensity = float(point.reflectivity)
                point_time_offset_sec = point.timestamp_offset_us / 1_000_000.0
                
                tag = point.tag if hasattr(point, 'tag') else 0
                line = point.line if hasattr(point, 'line') else 0

                points_data_for_ros.append([x, y, z, intensity, point_time_offset_sec, tag, line])
                unique_tags_in_packet.add(tag)
                unique_lines_in_packet.add(line)
        
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
            'points_data': points_data_for_ros,
            'unique_lines': unique_lines_in_packet, # Return diagnostic data
            'unique_tags': unique_tags_in_packet   # Return diagnostic data
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
        x, y, z, intensity, time_offset_sec, tag, line = point_raw
        p.x, p.y, p.z = x, y, z
        p.reflectivity = int(intensity)
        p.offset_time = int(time_offset_sec * 1_000_000_000)
        p.tag, p.line = tag, line
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

        angular_velocity_flu = [packet.gyro_x, packet.gyro_y, packet.gyro_z]
        linear_acceleration_flu = [
            packet.acc_x * config.GRAVITY_ACCEL,
            packet.acc_y * config.GRAVITY_ACCEL,
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
    Creates a sensor_msgs/Imu message using Livox IMU data,
    with the coordinate frame correctly transformed to the LiDAR frame
    as per the Livox Avia user manual.
    """
    if not livox_data:
        return None

    msg = Imu()
    msg.header.stamp = rospy.Time(livox_data['header_sec'], livox_data['header_nanosec'])
    msg.header.frame_id = frame_id
    
    # Transformation from IMU frame (FLU) to LiDAR frame (FRD)
    raw_angular_vel = livox_data['angular_velocity']
    msg.angular_velocity.x =  raw_angular_vel[0]
    msg.angular_velocity.y = -raw_angular_vel[1]
    msg.angular_velocity.z = -raw_angular_vel[2]

    raw_linear_accel = livox_data['linear_acceleration']
    msg.linear_acceleration.x =  raw_linear_accel[0]
    msg.linear_acceleration.y = -raw_linear_accel[1]
    msg.linear_acceleration.z = -raw_linear_accel[2]

    msg.orientation.w = 1.0
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = msg.angular_velocity_covariance[8] = 0.01
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

    RED, GREEN, YELLOW, RESET = '\033[91m', '\033[92m', '\033[93m', '\033[0m'
    
    print("--- Pass 1/2: Reading all messages into memory for analysis ---")
    all_messages = []
    channel_counts = Counter()
    
    # --- DIAGNOSTICS: Initialize sets to track all unique values found in the file ---
    all_lines_found = set()
    all_tags_found = set()
    
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
                if len(raw_msg_data) != msg_len: break
            except (EOFError, IndexError):
                break
            
            if channel_name not in CHANNEL_PARSERS: continue
            
            parsed_data = CHANNEL_PARSERS[channel_name](raw_msg_data, bin_envelope_timestamp_ms)
            if not parsed_data: continue

            if channel_name == "avia_points":
                # --- DIAGNOSTICS: Aggregate the unique values from this packet ---
                all_lines_found.update(parsed_data['unique_lines'])
                all_tags_found.update(parsed_data['unique_tags'])
                ros_msg = create_livox_custom_msg(parsed_data, config.LIDAR_FRAME_ID)
                if ros_msg: all_messages.append({'type': 'lidar', 'msg': ros_msg})
            elif channel_name == "avia_imu":
                ros_msg = create_livox_only_imu_msg(parsed_data, config.IMU_FRAME_ID)
                if ros_msg: all_messages.append({'type': 'imu', 'msg': ros_msg})

    print(f"Read {len(all_messages)} total parsable messages.")

    all_messages.sort(key=lambda m: m['msg'].header.stamp.to_sec())

    print("\n--- Pass 2/2: Running diagnostics and writing to bag ---")
    
    bag_writer = rosbag.Bag(output_bag_path, 'w')
    for item in all_messages:
        topic = config.LIDAR_TOPIC if item['type'] == 'lidar' else config.IMU_TOPIC
        bag_writer.write(topic, item['msg'], item['msg'].header.stamp)

    # --- FINAL REPORT SECTION ---
    print("\n" + "="*20 + " FINAL REPORT " + "="*20)
    print("\nChannel distribution:")
    for name, count_val in sorted(channel_counts.items()):
        print(f"  {name}: {count_val} messages")

    print("\n" + "-"*15 + " Data Integrity Report " + "-"*15)
    print(f"Unique 'line' values found: {sorted(list(all_lines_found))}")
    print(f"Unique 'tag' values found: {sorted(list(all_tags_found))}")

    is_data_ok = True
    # Check if line data is valid
    if len(all_lines_found) <= 1 and 0 in all_lines_found:
        print(f"{RED}CRITICAL: All points are on 'line: 0'. The data lacks the geometric structure needed for FAST-LIO feature extraction.{RESET}")
        is_data_ok = False
        
    # Check if tag data is valid
    if len(all_tags_found) <= 1 and 0 in all_tags_found:
        print(f"{YELLOW}WARNING: All points have 'tag: 0'. This indicates metadata may be missing.{RESET}")
        # This is a warning, not a critical failure, but still important.

    if is_data_ok:
        print(f"{GREEN}SUCCESS: The 'line' field contains multiple unique values, which is required for feature extraction.{RESET}")
    
    print("-" * 52)
    bag_writer.close()
    print(f"\nFinished. Wrote {len(all_messages)} total ROS messages into {output_bag_path}\n")


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