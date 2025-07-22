#!/usr/bin/env python3

import glob
import os
import math
import struct
import shutil

from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from sensor_msgs.msg import Imu, PointCloud2, PointField
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from common import config, livox_pb2, duro_gps_pb2

TOPICS = {
    'avia_points': ('lidar', livox_pb2.Point_Packet),
    'avia_imu':    ('imu',   livox_pb2.Imu_Data),
    'duro_gps_utc_time': ('gps', duro_gps_pb2.gps_data)
}

def process_file(file_obj, writer: SequentialWriter) -> None:
    """
    Read binary frames from file_obj, parse protobuf messages,
    and write corresponding ROS 2 messages into the bag.
    """
    file_obj.seek(0, os.SEEK_END)
    total_size = file_obj.tell()
    file_obj.seek(0)
    print(f"DEBUG: Starting to process file. Total size: {total_size / 1e9:.2f} GB")

    lidar_to_host_offsets = []
    host_to_utc_offsets = []

    while True:
        current_pos = file_obj.tell()
        header = file_obj.read(9)
        if len(header) < 9:
            print(f"DEBUG: Breaking loop. Header read was {len(header)} bytes.")
            print(f"DEBUG: Stopped at position {current_pos} out of {total_size} bytes.")
            break

        logger_timestamp_ns, ch_len = struct.unpack('>QB', header)
        channel = file_obj.read(ch_len).decode()

        length_bytes = file_obj.read(4)
        if len(length_bytes) < 4:
            break
        payload_len = struct.unpack('>I', length_bytes)[0]

        payload = file_obj.read(payload_len)
        topic_type, ProtoClass = TOPICS.get(channel, (None, None))
        if not ProtoClass:
            continue

        proto_msg = ProtoClass()
        try:
            proto_msg.ParseFromString(payload)
        except Exception as e:
            print(f"INFO: Failed to parse protobuf message. Error: {e}")
            break

        if topic_type in ['lidar', 'imu']:
            if not hasattr(proto_msg, 'timestamp_us'):
                print(f"INFO: Skipping message in '{channel}' because it has no 'timestamp_us' field.") # <-- ADD THIS
                continue

            stamp_ns = proto_msg.timestamp_us * 1000

            offset = logger_timestamp_ns - stamp_ns
            lidar_to_host_offsets.append(offset)
        
            if topic_type == 'lidar':
                write_lidar_msg_pointcloud2(proto_msg, writer, stamp_ns)
            else:
                write_imu_msg(proto_msg, writer, stamp_ns)
    
        elif topic_type == 'gps':
            host_time_ns = proto_msg.system_timestamp_ms * 1_000_000
            utc_time_ns = proto_msg.sensor_timestamp_ms * 1_000_000
            
            # Calculate and store the Host-to-UTC offset
            offset = utc_time_ns - host_time_ns
            host_to_utc_offsets.append(offset)

    if lidar_to_host_offsets:
        avg_lidar_to_host_offset = sum(lidar_to_host_offsets) / len(lidar_to_host_offsets)
        print(f"\n✅ Calculated LiDAR-to-Host offset: {avg_lidar_to_host_offset} ns")
    else:
        avg_lidar_to_host_offset = 0
        print("\n⚠️ No LiDAR/IMU messages found to calculate LiDAR-to-Host offset.")

    if host_to_utc_offsets:
        avg_host_to_utc_offset = sum(host_to_utc_offsets) / len(host_to_utc_offsets)
        print(f"✅ Calculated Host-to-UTC offset:    {avg_host_to_utc_offset} ns")
    else:
        avg_host_to_utc_offset = 0
        print("⚠️ No GPS messages found to calculate Host-to-UTC offset.")

    total_offset = avg_lidar_to_host_offset + avg_host_to_utc_offset
    print(f"✅ TOTAL OFFSET (LiDAR Clock to UTC): {total_offset} ns")


def write_lidar_msg_pointcloud2(proto_msg, writer: SequentialWriter, stamp_ns: int) -> None:
    """
    Build a sensor_msgs/PointCloud2 from proto_msg and write it to the ROS 2 bag.
    This format is compatible with SLAM frameworks like GLIM.
    """
    secs, nanosecs = divmod(stamp_ns, 1_000_000_000)
    header = Header(stamp=Time(sec=secs, nanosec=nanosecs), frame_id=config.LIDAR_FRAME_ID)
    fields = [
        PointField(name='x',           offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',           offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',           offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='reflectivity',offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='t',           offset=16, datatype=PointField.UINT32,  count=1),
    ]
    
    point_step = 20
    num_points = proto_msg.num_points

    point_data = []
    packer = struct.Struct('<fff fI')

    for i in range(num_points):
        x = proto_msg.x_coords[i] / 1000.0
        y = proto_msg.y_coords[i] / 1000.0
        z = proto_msg.z_coords[i] / 1000.0
        reflectivity = float(proto_msg.reflectivity[i])
        offset_time_ns = proto_msg.timestamp_offset_us[i] * 1000

        point_data.append(packer.pack(x, y, z, reflectivity, offset_time_ns))

    msg = PointCloud2(
        header=header,
        height=1,
        width=num_points,
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=point_step,
        row_step=point_step * num_points,
        data=b''.join(point_data)
    )
    serialized_msg = serialize_message(msg)
    writer.write(config.LIDAR_TOPIC, serialized_msg, stamp_ns)


def write_imu_msg(proto_msg, writer: SequentialWriter, stamp_ns: int) -> None:
    """
    Build a sensor_msgs/Imu message from proto_msg and write it to the ROS 2 bag.
    """
    secs, nanosecs = divmod(stamp_ns, 1_000_000_000)
    
    imu_msg = Imu(
        header=Header(stamp=Time(sec=secs, nanosec=nanosecs), frame_id=config.IMU_FRAME_ID)
    )

    imu_msg.linear_acceleration.x = proto_msg.acc_x
    imu_msg.linear_acceleration.y = proto_msg.acc_y
    imu_msg.linear_acceleration.z = proto_msg.acc_z

    imu_msg.angular_velocity.x = math.radians(proto_msg.gyro_x)
    imu_msg.angular_velocity.y = math.radians(proto_msg.gyro_y)
    imu_msg.angular_velocity.z = math.radians(proto_msg.gyro_z)

    serialized_msg = serialize_message(imu_msg)
    writer.write(config.IMU_TOPIC, serialized_msg, stamp_ns)


def main():
    os.makedirs(config.OUTPUT_ROS2BAGS, exist_ok=True)
    pattern = os.path.join(config.INPUT_ROSBAG_BIN, '*.bin')

    for bin_path in sorted(glob.glob(pattern)):
        bag_name = os.path.basename(bin_path).replace('.bin', '')
        out_path = os.path.join(config.OUTPUT_ROS2BAGS, bag_name)

        if os.path.exists(out_path):
            print(f"Overwriting existing bag: {out_path}")
            shutil.rmtree(out_path)
        
        print(f"Processing {bin_path} -> {out_path}")
        writer = SequentialWriter()
        storage_options = StorageOptions(uri=out_path, storage_id='sqlite3')
        converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        writer.open(storage_options, converter_options)

        imu_topic = TopicMetadata(
            name=config.IMU_TOPIC,
            type='sensor_msgs/msg/Imu',
            serialization_format='cdr'
        )
        writer.create_topic(imu_topic)

        lidar_topic = TopicMetadata(
            name=config.LIDAR_TOPIC,
            type='sensor_msgs/msg/PointCloud2',
            serialization_format='cdr'
        )
        writer.create_topic(lidar_topic)

        with open(bin_path, 'rb') as f:
            process_file(f, writer)

    print("\nConversion to ROS 2 bag with standard messages complete")


if __name__ == '__main__':
    main()