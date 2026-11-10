#!/usr/bin/env python3
"""Convert a ROS 1 bag with Livox scans (livox_ros_driver/CustomMsg, e.g. KILVO) into a ROS 2 bag
that FAST-LIO for ROS 2 can read (livox_ros_driver2/msg/CustomMsg). Both messages have the same
fields, so the scans are copied as they are; every other topic is converted as usual.

    pip install rosbags
    python livox_bag_to_ros2.py outdoor_h01_lidar_imu.bag outdoor_h01_ros2

A plain `rosbags-convert` keeps the ROS 1 type name, which ROS 2 does not know, and `ros2 bag play`
then skips the scans ("package 'livox_ros_driver' not found").
"""
import sys
from pathlib import Path

from rosbags.rosbag1 import Reader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

POINT = """uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""
MSG = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
livox_ros_driver2/CustomPoint[] points
"""


def main(src: str, dst: str) -> int:
    store = get_typestore(Stores.ROS2_HUMBLE)  # also converts ROS 1 bytes (headers with seq) to CDR
    for pkg in ("livox_ros_driver", "livox_ros_driver2"):
        types = get_types_from_msg(POINT, f"{pkg}/msg/CustomPoint")
        types.update(get_types_from_msg(MSG.replace("livox_ros_driver2", pkg), f"{pkg}/msg/CustomMsg"))
        store.register(types)
    rename = {"livox_ros_driver/msg/CustomMsg": "livox_ros_driver2/msg/CustomMsg"}
    with Reader(Path(src)) as r, Writer(Path(dst), version=8) as w:  # metadata v8: readable by Humble
        out = {}
        for c in r.connections:
            if c.topic not in out:
                out[c.topic] = w.add_connection(c.topic, rename.get(c.msgtype, c.msgtype), typestore=store)
        n = 0
        for c, t, raw in r.messages():
            # same fields: the Livox scan keeps its bytes under the ROS 2 package name
            w.write(out[c.topic], t, store.ros1_to_cdr(raw, c.msgtype))
            n += 1
    print(f"{dst}: {n} messages, Livox scans as livox_ros_driver2/msg/CustomMsg")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1], sys.argv[2]))
