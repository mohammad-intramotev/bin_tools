#!/usr/bin/env python3
import glob
import math
import os
import shutil
import sys
from collections import Counter, deque

import numpy as np
import rosbag2_py
import tf_transformations

from rclpy.serialization import serialize_message
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header

from common import livox_pb2, orientation_pb2

# --- Constants ---
GRAVITY_ACCEL = 9.80665 # m/s^2
MAX_ORIENTATION_AGE_MS = 200 # Max age of orientation data to use
IMU_BUFFER_MAX_LEN = 100

LIDAR_FRAME_ID = "lidar"
IMU_FRAME_ID = "imu"

LIDAR_TOPIC = "/os_cloud_node/points"
IMU_TOPIC = "/os_cloud_node/imu"

# Buffers for IMU data components
duro_orientation_buffer = deque(maxlen=IMU_BUFFER_MAX_LEN)


def parse_livox_point_packet(raw_bytes, bin_envelope_timestamp_ms):
    """
    Parses a Livox Point_Packet protobuf.
    """
    try:
        packet = livox_pb2.Point_Packet()
        packet.ParseFromString(raw_bytes)

        num_points = packet.num_points
        if not (len(packet.x_coords) == num_points and
                len(packet.y_coords) == num_points and
                len(packet.z_coords) == num_points and
                len(packet.reflectivity) == num_points and
                len(packet.timestamp_offset_us) == num_points):
            proto_ts_debug = getattr(packet, 'system_timestamp_us', 'N/A') # Safety for debug print
            print(f"DEBUG Points: Point_Packet field length mismatch for proto_ts {proto_ts_debug}. Num_points: {num_points}. Skipping packet.")
            return None

        points_data_for_ros = []
        for i in range(num_points):
            x = packet.x_coords[i] / 1000.0
            y = packet.y_coords[i] / 1000.0
            z = packet.z_coords[i] / 1000.0
            intensity = float(packet.reflectivity[i])
            point_time_offset_sec = packet.timestamp_offset_us[i] / 1_000_000.0
            points_data_for_ros.append([x, y, z, intensity, point_time_offset_sec])

        if hasattr(packet, 'timestamp_us') and packet.timestamp_us > 0:
            header_time_sec = packet.timestamp_us / 1_000_000.0  # Microseconds to seconds
        else:
            print(f"WARNING: Livox Point_Packet missing or zero 'timestamp_us'. Falling back to logger's envelope timestamp {bin_envelope_timestamp_ms}ms for PCL header.")
            header_time_sec = bin_envelope_timestamp_ms / 1000.0

        sec_hdr = int(header_time_sec)
        nanosec_hdr = int((header_time_sec - sec_hdr) * 1_000_000_000)

        return {
            'sec': sec_hdr,
            'nanosec': nanosec_hdr,
            'points_data': points_data_for_ros
        }
    except Exception as e:
        print(f"Error parsing Livox Point_Packet: {e}")
        return None


def create_pointcloud2_msg(parsed_data, frame_id):
    if not parsed_data or not parsed_data['points_data']:
        return None

    header = Header()
    header.stamp.sec = parsed_data['sec']
    header.stamp.nanosec = parsed_data['nanosec']
    header.frame_id = frame_id

    points_np = np.array(parsed_data['points_data'], dtype=np.float32)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = points_np.shape[0] if points_np.ndim > 1 else 0
    if msg.width == 0 and len(parsed_data['points_data']) > 0:
        msg.width = 1
        points_np = points_np.reshape(1, -1)

    msg.is_dense = True
    msg.is_bigendian = False
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1)
    ]
    msg.point_step = 20
    msg.row_step = msg.point_step * msg.width
    msg.data = points_np.tobytes()
    return msg


def parse_livox_imu_data(raw_bytes, bin_envelope_timestamp_ms):
    """
    Parses Livox Imu_Data protobuf.
    Returns original sensor timestamp for header and logger timestamp for age checks.
    """

    try:
        packet = livox_pb2.Imu_Data()
        packet.ParseFromString(raw_bytes)

        if hasattr(packet, 'timestamp_us') and packet.timestamp_us > 0:
            original_sensor_time = packet.timestamp_us / 1_000_000.0
            sensor_time_sec_hdr = int(original_sensor_time)
            sensor_time_nanosec_hdr = int((original_sensor_time - sensor_time_sec_hdr) * 1_000_000_000)
        else:
            print(f"WARNING: Livox Imu_Data missing or zero 'timestamp_us'. "
                  f"Falling back to logger timestamp {bin_envelope_timestamp_ms}ms for header.")
            fallback_time = bin_envelope_timestamp_ms / 1000.0
            sensor_time_sec_hdr = int(fallback_time)
            sensor_time_nanosec_hdr = int((fallback_time - sensor_time_sec_hdr) * 1_000_000_000)

        return {
            'original_sensor_sec': sensor_time_sec_hdr,
            'original_sensor_nanosec': sensor_time_nanosec_hdr,
            'logger_timestamp_ms': bin_envelope_timestamp_ms, # For age comparison
            'angular_velocity': [packet.gyro_x, packet.gyro_y, packet.gyro_z],
            'linear_acceleration': [
                packet.acc_x * GRAVITY_ACCEL,
                packet.acc_y * GRAVITY_ACCEL,
                packet.acc_z * GRAVITY_ACCEL
            ]
        }
    except Exception as e:
        sys_ts_val = getattr(packet, 'system_timestamp_us', 'N/A') if packet else "N/A"
        print(f"Error parsing Livox Imu_Data. SysTS_us: {sys_ts_val}. BinTS_ms: {bin_envelope_timestamp_ms}. Exception: {e}")
        return None

def parse_duro_orient_euler(raw_bytes, bin_envelope_timestamp_ms):
    """
    Parses a Duro orientation Euler protobuf.
    """
    try:
        packet = orientation_pb2.MsgOrientEuler()
        packet.ParseFromString(raw_bytes)

        roll_rad = math.radians(packet.roll / 1_000_000.0)
        pitch_rad = math.radians(packet.pitch / 1_000_000.0)
        yaw_rad = math.radians(packet.yaw / 1_000_000.0)
        q = tf_transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        return {
            'logger_timestamp_ms': bin_envelope_timestamp_ms, # For age comparison
            'orientation_q': [q[0], q[1], q[2], q[3]]
        }
    except Exception as e:
        tow_val = getattr(packet, 'tow', 'N/A') if packet else 'N/A'
        print(f"ERROR Parsing Duro Orient Euler. ProtoData: tow={tow_val}. BinTS_ms: {bin_envelope_timestamp_ms}. Exception: {e}")
        return None


def create_combined_imu_msg(livox_data, duro_data, frame_id):
    """
    Builds a sensor_msgs/Imu message from combined Livox IMU + Duro orientation.
    """
    msg = Imu()
    msg.header.stamp.sec = int(livox_data['original_sensor_sec'])
    msg.header.stamp.nanosec = int(livox_data['original_sensor_nanosec'])
    msg.header.frame_id = frame_id

    # Fill orientation from Duro quaternion
    msg.orientation.x = duro_data['orientation_q'][0]
    msg.orientation.y = duro_data['orientation_q'][1]
    msg.orientation.z = duro_data['orientation_q'][2]
    msg.orientation.w = duro_data['orientation_q'][3]
    msg.orientation_covariance[0] = 0.01
    msg.orientation_covariance[4] = 0.01
    msg.orientation_covariance[8] = 0.01
    for i in [1, 2, 3, 5, 6, 7]:
        msg.orientation_covariance[i] = 0.0 # Initialize others

    # Fill angular velocity from Livox IMU
    msg.angular_velocity.x = livox_data['angular_velocity'][0]
    msg.angular_velocity.y = livox_data['angular_velocity'][1]
    msg.angular_velocity.z = livox_data['angular_velocity'][2]
    msg.angular_velocity_covariance[0] = 0.01
    msg.angular_velocity_covariance[4] = 0.01
    msg.angular_velocity_covariance[8] = 0.01
    for i in [1, 2, 3, 5, 6, 7]:
        msg.angular_velocity_covariance[i] = 0.0 # Initialize others

    # Fill linear acceleration from Livox IMU
    msg.linear_acceleration.x = livox_data['linear_acceleration'][0]
    msg.linear_acceleration.y = livox_data['linear_acceleration'][1]
    msg.linear_acceleration.z = livox_data['linear_acceleration'][2]
    msg.linear_acceleration_covariance[0] = 0.01
    msg.linear_acceleration_covariance[4] = 0.01
    msg.linear_acceleration_covariance[8] = 0.01
    for i in [1, 2, 3, 5, 6, 7]:
        msg.linear_acceleration_covariance[i] = 0.0 # Initialize others

    return msg


def try_combine_imu_data_new(current_livox_imu_data, bag_writer):
    """
    Called when a new Livox IMU (gyro/accel) packet is parsed.
    Tries to find the most recent suitable Duro orientation using logger timestamps.
    """
    if not duro_orientation_buffer:
        return 0

    best_duro_data = None
    # Age comparison uses logger timestamps for pragmatic synchronization checking
    # current_livox_imu_data['logger_timestamp_ms'] is used here
    # d_data['logger_timestamp_ms'] is used from duro_orientation_buffer
    for d_data in reversed(duro_orientation_buffer):
        # Use logger timestamps (both are in milliseconds)
        if d_data['logger_timestamp_ms'] <= current_livox_imu_data['logger_timestamp_ms']:
            if (current_livox_imu_data['logger_timestamp_ms'] - d_data['logger_timestamp_ms']) <= MAX_ORIENTATION_AGE_MS:
                best_duro_data = d_data
                break
            else:
                # print(f"DEBUG IMU Combine: Stale Duro data found...")
                break

    if best_duro_data:
        imu_msg = create_combined_imu_msg(current_livox_imu_data, best_duro_data, IMU_FRAME_ID)
        if imu_msg:
            ros_timestamp_ns = (imu_msg.header.stamp.sec * 1_000_000_000) + imu_msg.header.stamp.nanosec
            bag_writer.write(IMU_TOPIC, serialize_message(imu_msg), ros_timestamp_ns)
            return 1
    # else:
        # print(f"DEBUG IMU Combine: No suitable Duro orientation found...")
    return 0


CHANNEL_PARSERS = {
    "avia_points": parse_livox_point_packet,
    "avia_imu": parse_livox_imu_data,
    "duro_gps_orient_eule": parse_duro_orient_euler
}


def create_ros2_bag(bin_file_path, output_bag_dir):
    # If the .bag already exists, remove it
    if os.path.exists(output_bag_dir):
        print(f"Output directory {output_bag_dir} already exists. Removing it.")
        try:
            shutil.rmtree(output_bag_dir)
        except OSError as e:
            print(f"Error removing existing directory {output_bag_dir}: {e}. Please check permissions or remove manually.")
            return

    # Open a ROS2 bag for writing
    bag_writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=output_bag_dir, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    bag_writer.open(storage_options, converter_options)

    topic_metadata_map = {
        LIDAR_TOPIC: rosbag2_py.TopicMetadata(name=LIDAR_TOPIC, type='sensor_msgs/msg/PointCloud2', serialization_format='cdr'),
        IMU_TOPIC: rosbag2_py.TopicMetadata(name=IMU_TOPIC, type='sensor_msgs/msg/Imu', serialization_format='cdr')
    }
    bag_writer.create_topic(topic_metadata_map[LIDAR_TOPIC])
    bag_writer.create_topic(topic_metadata_map[IMU_TOPIC])
    print(f"Registered topics: {LIDAR_TOPIC}, {IMU_TOPIC} for bag {output_bag_dir}")

    message_count_total = 0
    imu_combined_count = 0
    channel_counts = Counter()
    print(f"Processing .bin file: {bin_file_path} into {output_bag_dir}")

    duro_orientation_buffer.clear() # Clear for this bag file

    bin_envelope_timestamp_ms = 0

    with open(bin_file_path, 'rb') as log_file:
        while True:
            try:
                # Read the next 8-byte envelope timestamp (big-endian)
                timestamp_bytes = log_file.read(8)
                if not timestamp_bytes:
                    break
                bin_envelope_timestamp_ms = int.from_bytes(timestamp_bytes, byteorder='big')

                # Next: 1-byte channel name length, then that many bytes of channel name
                channel_len_byte = log_file.read(1)
                if not channel_len_byte:
                    break
                channel_len = int.from_bytes(channel_len_byte, byteorder='big')

                channel_name_bytes = log_file.read(channel_len)
                if not channel_name_bytes:
                    break
                channel_name = channel_name_bytes.decode('utf-8', errors='replace')
                channel_counts[channel_name] += 1

                # Next: 4-byte message length, then the raw protobuf payload
                msg_len_bytes = log_file.read(4)
                if not msg_len_bytes:
                    break
                msg_len = int.from_bytes(msg_len_bytes, byteorder='big')

                raw_msg_data = log_file.read(msg_len)
                if len(raw_msg_data) != msg_len:
                    print(f"Warning: Truncated message for channel {channel_name} at end of file. "
                          f"Expected {msg_len} bytes, got {len(raw_msg_data)}. Aborting.")
                    break

            except EOFError:
                break
            except Exception as e:
                print(f"Error reading from .bin file {bin_file_path}: {e}")
                break

            if channel_name not in CHANNEL_PARSERS:
                continue

            parser_func = CHANNEL_PARSERS[channel_name]
            parsed_data = parser_func(raw_msg_data, bin_envelope_timestamp_ms)

            if not parsed_data:
                continue

            if channel_name == "avia_points":
                ros_msg = create_pointcloud2_msg(parsed_data, LIDAR_FRAME_ID)
                if ros_msg:
                    ros_timestamp_ns = (parsed_data['sec'] * 1_000_000_000) + parsed_data['nanosec']
                    bag_writer.write(LIDAR_TOPIC, serialize_message(ros_msg), ros_timestamp_ns)
                    message_count_total += 1

            elif channel_name == "duro_gps_orient_eule":
                duro_orientation_buffer.append(parsed_data)

            elif channel_name == "avia_imu":
                combined_count = try_combine_imu_data_new(parsed_data, bag_writer)
                if combined_count > 0:
                    imu_combined_count += combined_count
                    message_count_total += combined_count

            if message_count_total > 0 and (message_count_total % 200) == 0:
                print(f"[{os.path.basename(output_bag_dir)}] Wrote {message_count_total} ROS msgs "
                    f"(IMU combined: {imu_combined_count})...")

        print(f"[{os.path.basename(output_bag_dir)}] Completed reading {os.path.basename(bin_file_path)}. "
              f"Channel distribution:")
        for name, count_val in channel_counts.items():
            print(f"  {name}: {count_val} messages")
        print(f"[{os.path.basename(output_bag_dir)}] Duro orientation buffer size: {len(duro_orientation_buffer)}")

    bag_writer.close()
    print(f"Finished processing {bin_file_path}. Wrote {message_count_total} ROS messages "
          f"(Total IMU combined: {imu_combined_count}) into {output_bag_dir}")


def process_bin_path(input_path, output_base_dir):
    try:
        print(f"Converting .bin files in directory: {input_path}")
        bin_files = sorted(glob.glob(os.path.join(input_path, '*.bin')))
        if not bin_files:
            print(f"No .bin files found in {input_path}")
            return
        for bin_file in bin_files:
            base_name = os.path.splitext(os.path.basename(bin_file))[0]
            output_bag_dir = os.path.join(output_base_dir, base_name + "_bag")
            create_ros2_bag(bin_file, output_bag_dir)
    except:
        print(f"Invalid path: {input_path}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python your_script_name.py <input_bin_file_or_dir> [output_base_directory]")
        sys.exit(1)

    input_path_arg = sys.argv[1]
    output_base_dir_arg = sys.argv[2] if len(sys.argv) > 2 else "ros_bags_output/ros2_bags"

    if not os.path.exists(output_base_dir_arg):
        os.makedirs(output_base_dir_arg)

    process_bin_path(input_path_arg, output_base_dir_arg)
    print("Conversion to ROS 2 bag complete.")
