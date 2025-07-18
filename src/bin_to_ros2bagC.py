#!/usr/bin/env python3

import glob
import os
import struct
import shutil

from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from livox_custom_msgs_ros2.msg import CustomMsg, CustomPoint
from common import config, livox_pb2


TOPICS = {
    'avia_points': ('lidar', livox_pb2.Point_Packet),
    'avia_imu':    ('imu',   livox_pb2.Imu_Data),
}


def process_file(file_obj, writer: SequentialWriter) -> None:
    """
    Read binary frames from file_obj, parse protobuf messages,
    and write corresponding ROS 2 messages into the bag.
    """
    while True:
        header = file_obj.read(9)
        if len(header) < 9:
            break

        _, ch_len = struct.unpack('>QB', header)
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
        except Exception:
            continue

        if not hasattr(proto_msg, 'timestamp_us'):
            continue

        stamp_ns = proto_msg.timestamp_us * 1000
        if topic_type == 'lidar':
            write_lidar_msg(proto_msg, writer, stamp_ns)
        else:
            write_imu_msg(proto_msg, writer, stamp_ns)


def write_lidar_msg(proto_msg, writer: SequentialWriter, stamp_ns: int) -> None:
    """
    Build a CustomMsg from proto_msg and write it to the ROS 2 bag.
    """
    secs, nanosecs = divmod(stamp_ns, 1_000_000_000)
    msg = CustomMsg(
        header=Header(stamp=Time(sec=secs, nanosec=nanosecs), frame_id=config.LIDAR_FRAME_ID),
        point_num=proto_msg.num_points,
        lidar_id=0,
        points=[
            CustomPoint(
                x=x / 1000.0,
                y=y / 1000.0,
                z=z / 1000.0,
                reflectivity=int(r),
                offset_time=int(off * 1000),
                tag=0,
                line=0,
            )
            for x, y, z, r, off in zip(
                proto_msg.x_coords,
                proto_msg.y_coords,
                proto_msg.z_coords,
                proto_msg.reflectivity,
                proto_msg.timestamp_offset_us,
            )
        ],
    )
    serialized_msg = serialize_message(msg)
    writer.write(config.LIDAR_TOPIC, serialized_msg, stamp_ns)


def write_imu_msg(proto_msg, writer: SequentialWriter, stamp_ns: int) -> None:
    """
    Build an Imu message from proto_msg and write it to the ROS 2 bag.
    """
    secs, nanosecs = divmod(stamp_ns, 1_000_000_000)
    
    imu_msg = Imu(
        header=Header(stamp=Time(sec=secs, nanosec=nanosecs), frame_id=config.IMU_FRAME_ID)
    )

    imu_msg.angular_velocity.x = proto_msg.gyro_x
    imu_msg.angular_velocity.y = proto_msg.gyro_y
    imu_msg.angular_velocity.z = proto_msg.gyro_z

    g = config.GRAVITY_ACCEL
    imu_msg.linear_acceleration.x = proto_msg.acc_x * g
    imu_msg.linear_acceleration.y = proto_msg.acc_y * g
    imu_msg.linear_acceleration.z = proto_msg.acc_z * g

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
            type='livox_custom_msgs_ros2/msg/CustomMsg',
            serialization_format='cdr'
        )
        writer.create_topic(lidar_topic)

        with open(bin_path, 'rb') as f:
            process_file(f, writer)

    print("\nConversion to ROS 2 bag complete with custom messages complete.")


if __name__ == '__main__':
    main()