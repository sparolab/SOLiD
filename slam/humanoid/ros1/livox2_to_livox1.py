"""Copy a ROS 1 bag keeping /livox/lidar + /livox/imu and retype livox_ros_driver2/CustomMsg as
livox_ros_driver/CustomMsg (same wire layout), so ROS 1 FAST-LIO can read it (e.g. KILVO bags).

    python livox2_to_livox1.py in.bag out.bag
"""
import sys
from pathlib import Path
from rosbags.rosbag1 import Reader, Writer
src, dst = Path(sys.argv[1]), Path(sys.argv[2])
MD5_V1 = "e4d6829bdfe657cb6c21a746c86b21a6"
with Reader(src) as r, Writer(dst) as w:
    out = {}
    for c in r.connections:
        if c.topic == "/livox/lidar":
            msgdef = c.msgdef.data.replace("livox_ros_driver2", "livox_ros_driver")
            out[c.id] = w.add_connection(c.topic, "livox_ros_driver/msg/CustomMsg", msgdef=msgdef, md5sum=MD5_V1)
        elif c.topic == "/livox/imu":
            out[c.id] = w.add_connection(c.topic, c.msgtype, msgdef=c.msgdef.data, md5sum=c.digest)
    conns = [c for c in r.connections if c.id in out]
    n = 0
    for c, t, raw in r.messages(connections=conns):
        w.write(out[c.id], t, raw); n += 1
print(dst, n, "messages")
