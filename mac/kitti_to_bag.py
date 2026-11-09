#!/usr/bin/env python3
"""Convert a KITTI odometry sequence into a ROS 1 or ROS 2 bag (no ROS needed).

Writes /cloud (sensor_msgs/PointCloud2: x y z intensity, LiDAR frame) and
/odom (nav_msgs/Odometry: ground truth, or --poses, in the LiDAR frame, frame `odom`),
so the same data can drive the ROS nodes and the Mac tool.

    python kitti_to_bag.py /data/kitti/sequences/00 kitti00.bag        # ROS 1
    python kitti_to_bag.py /data/kitti/sequences/00 kitti00_ros2        # ROS 2 (.db3)
    python kitti_to_bag.py ... --stride 2 --voxel 0.4                   # smaller bag
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from rosbags.typesys import Stores, get_typestore

from solid_loops import read_kitti, rot_to_quat


def voxel_downsample(pts: np.ndarray, leaf: float) -> np.ndarray:
    if leaf <= 0:
        return pts
    keys = np.floor(pts[:, :3] / leaf).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("seq", help="KITTI sequence folder (velodyne/, poses.txt, calib.txt)")
    ap.add_argument("out", help="output: *.bag -> ROS 1, anything else -> ROS 2 folder")
    ap.add_argument("--stride", type=int, default=1, help="keep every N-th scan")
    ap.add_argument("--voxel", type=float, default=0.0, help="voxel leaf (m) to shrink the bag")
    ap.add_argument("--max-frames", type=int, default=0, help="stop after N scans (0 = all)")
    ap.add_argument("--poses", default="", help="odometry poses (KITTI format, LiDAR frame) for /odom "
                                               "instead of the ground truth, e.g. from KISS-ICP")
    args = ap.parse_args()

    out = Path(args.out)
    ros1 = out.suffix == ".bag"
    store = get_typestore(Stores.ROS1_NOETIC if ros1 else Stores.ROS2_HUMBLE)
    PointCloud2 = store.types["sensor_msgs/msg/PointCloud2"]
    PointField = store.types["sensor_msgs/msg/PointField"]
    Header = store.types["std_msgs/msg/Header"]
    Time = store.types["builtin_interfaces/msg/Time"]
    Odometry = store.types["nav_msgs/msg/Odometry"]
    PoseWithCov = store.types["geometry_msgs/msg/PoseWithCovariance"]
    TwistWithCov = store.types["geometry_msgs/msg/TwistWithCovariance"]
    Pose = store.types["geometry_msgs/msg/Pose"]
    Point = store.types["geometry_msgs/msg/Point"]
    Quat = store.types["geometry_msgs/msg/Quaternion"]
    Twist = store.types["geometry_msgs/msg/Twist"]
    Vec3 = store.types["geometry_msgs/msg/Vector3"]

    fields = [PointField(name=n, offset=4 * i, datatype=7, count=1)
              for i, n in enumerate(("x", "y", "z", "intensity"))]

    def header(stamp: float, frame: str, seq: int):
        t = Time(sec=int(stamp), nanosec=int(round((stamp - int(stamp)) * 1e9)) % 1_000_000_000)
        return Header(seq=seq, stamp=t, frame_id=frame) if ros1 else Header(stamp=t, frame_id=frame)

    if ros1:
        from rosbags.rosbag1 import Writer
        writer = Writer(out)
        serialize = store.serialize_ros1
    else:
        from rosbags.rosbag2 import Writer
        writer = Writer(out, version=8)  # version 8 is readable by Humble and newer
        serialize = store.serialize_cdr

    t0 = 1_000_000_000.0  # KITTI times start at 0; use a plausible epoch offset
    n = 0
    with writer:
        c_cloud = writer.add_connection("/cloud", PointCloud2.__msgtype__, typestore=store)
        c_odom = writer.add_connection("/odom", Odometry.__msgtype__, typestore=store)
        for stamp, T, xyz, _, inten in read_kitti(args.seq, args.stride, args.poses):
            s = t0 + stamp
            ns = int(round(s * 1e9))
            pts = voxel_downsample(np.column_stack([xyz, inten]).astype(np.float32), args.voxel)
            cloud = PointCloud2(header=header(s, "velodyne", n), height=1, width=len(pts),
                                fields=fields, is_bigendian=False, point_step=16,
                                row_step=16 * len(pts), data=np.frombuffer(pts.tobytes(), np.uint8),
                                is_dense=True)
            q = rot_to_quat(T[:3, :3])
            odom = Odometry(
                header=header(s, "odom", n), child_frame_id="velodyne",
                pose=PoseWithCov(pose=Pose(position=Point(x=T[0, 3], y=T[1, 3], z=T[2, 3]),
                                           orientation=Quat(x=q[0], y=q[1], z=q[2], w=q[3])),
                                 covariance=np.zeros(36)),
                twist=TwistWithCov(twist=Twist(linear=Vec3(x=0.0, y=0.0, z=0.0),
                                               angular=Vec3(x=0.0, y=0.0, z=0.0)),
                                   covariance=np.zeros(36)))
            writer.write(c_cloud, ns, serialize(cloud, PointCloud2.__msgtype__))
            writer.write(c_odom, ns, serialize(odom, Odometry.__msgtype__))
            n += 1
            if n % 500 == 0:
                print(f"{n} scans")
            if args.max_frames and n >= args.max_frames:
                break
    print(f"wrote {n} scans to {out} ({'ROS 1' if ros1 else 'ROS 2'})")


if __name__ == "__main__":
    main()
