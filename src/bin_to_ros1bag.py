#!/usr/bin/env python3

import glob
import os
import struct

import rospy
import rosbag
from sensor_msgs.msg import Imu

from common import config, livox_pb2
from livox_ros_driver.msg import CustomMsg, CustomPoint


TOPICS = {
    'avia_points': ('lidar', livox_pb2.Point_Packet),
    'avia_imu':    ('imu',   livox_pb2.Imu_Data),
}


def to_stamp(timestamp_us: int) -> rospy.Time:
    """
    Convert a timestamp in microseconds to a rospy.Time object.
    """
    secs, micros = divmod(timestamp_us, 1_000_000)
    return rospy.Time(secs, micros * 1000)


def process_file(file_obj, bag: rosbag.Bag) -> None:
    """
    Read binary frames from file_obj, parse protobuf messages,
    and write corresponding ROS messages into bag.
    """
    while True:
        header = file_obj.read(9)
        if len(header) < 9:
            break

        ts_us, ch_len = struct.unpack('>QB', header)
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

        stamp = to_stamp(proto_msg.timestamp_us)
        if topic_type == 'lidar':
            write_lidar_msg(proto_msg, bag, stamp)
        else:
            write_imu_msg(proto_msg, bag, stamp)


def write_lidar_msg(proto_msg, bag: rosbag.Bag, stamp: rospy.Time) -> None:
    """
    Build a CustomMsg from proto_msg and write it to bag.
    """
    msg = CustomMsg(
        header=rospy.Header(stamp=stamp, frame_id=config.LIDAR_FRAME_ID),
        point_num=proto_msg.num_points,
        lidar_id=0,
        points=[
            CustomPoint(
                x=x / 1000.0,
                y=y / 1000.0,
                z=z / 1000.0,
                reflectivity=int(r),
                offset_time=int(off * 1000),
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
    bag.write(config.LIDAR_TOPIC, msg, stamp)


def write_imu_msg(proto_msg, bag: rosbag.Bag, stamp: rospy.Time) -> None:
    """
    Build an Imu message from proto_msg and write it to bag.
    """
    imu = Imu()
    imu.header.stamp = stamp
    imu.header.frame_id = config.IMU_FRAME_ID

    imu.angular_velocity.x = proto_msg.gyro_x
    imu.angular_velocity.y = proto_msg.gyro_y
    imu.angular_velocity.z = proto_msg.gyro_z

    g = config.GRAVITY_ACCEL
    imu.linear_acceleration.x = proto_msg.acc_x * g
    imu.linear_acceleration.y = proto_msg.acc_y * g
    imu.linear_acceleration.z = proto_msg.acc_z * g

    bag.write(config.IMU_TOPIC, imu, stamp)


def main():
    os.makedirs(config.OUTPUT_ROSBAGS, exist_ok=True)
    pattern = os.path.join(config.INPUT_ROSBAG_BIN, '*.bin')

    for bin_path in sorted(glob.glob(pattern)):
        bag_name = os.path.basename(bin_path).replace('.bin', '.bag')
        out_path = os.path.join(config.OUTPUT_ROSBAGS, bag_name)

        with open(bin_path, 'rb') as f, rosbag.Bag(out_path, 'w') as bag:
            process_file(f, bag)

    print("Conversion to ROS 1 bag complete.")

if __name__ == '__main__':
    main()
