#!/usr/bin/env python3
"""Offline SOLiD loop detection without ROS (macOS / Linux).

Reads a ROS 1 bag, a ROS 2 bag (.db3 / .mcap) or a KITTI sequence, picks
keyframes along the odometry, and reports SOLiD loop candidates. Uses the same
keyframe / loop logic and the same slam/config/*.yaml files as the ROS nodes.

    # ROS 1 or ROS 2 bag (cloud + odometry topics)
    python solid_loops.py --bag run.bag --cloud /cloud --odom /odom \
        --config ../config/ouster64.yaml --out results

    # KITTI sequence (velodyne/*.bin + poses.txt + calib.txt)
    python solid_loops.py --kitti /data/kitti/sequences/00 \
        --config ../config/kitti.yaml --out results

Outputs in --out: keyframes.txt (TUM), loops.csv and loops.png.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from collections import deque
from typing import Iterator, List, Optional, Tuple

import numpy as np
import yaml

try:
    import solid
except ImportError:
    sys.exit("The `solid` package is missing. From the SOLiD repo root run: pip install .")


# --------------------------------------------------------------------------- config
def load_params(path: str) -> dict:
    with open(path) as f:
        raw = yaml.safe_load(f) or {}
    cfg = solid.Config()
    for key, value in (raw.get("solid") or {}).items():
        if not hasattr(cfg, key):
            raise KeyError(f"{path}: unknown solid key '{key}'")
        setattr(cfg, key, type(getattr(cfg, key))(value))
    kf, lp, inp = raw.get("keyframe") or {}, raw.get("loop") or {}, raw.get("input") or {}
    return {
        "solid": cfg,
        "keyframe_dist": float(kf.get("dist", 1.0)),
        "keyframe_angle_deg": float(kf.get("angle_deg", 10.0)),
        "exclude_recent": int(lp.get("exclude_recent", 50)),
        "min_score": float(lp.get("min_score", 0.99)),
        "max_candidates": int(lp.get("max_candidates", 1)),
        "cloud_in_world": bool(inp.get("cloud_in_world", False)),
        "weight_field": str(inp.get("weight_field", "") or ""),
    }


# --------------------------------------------------------------------------- detector
class LoopDetector:
    """Python twin of slam/common/include/solid_slam/loop_detector.hpp."""

    def __init__(self, p: dict):
        self.p = p
        self.extractor = solid.Extractor(p["solid"])
        self.database = solid.Database(p["solid"])
        self.stamps: List[float] = []
        self.poses: List[np.ndarray] = []
        self.loops: List[Tuple[int, int, float, float]] = []
        self._pending: deque = deque()

    def is_keyframe(self, pose: np.ndarray) -> bool:
        if not self.poses:
            return True
        d = np.linalg.inv(self.poses[-1]) @ pose
        cos = np.clip((np.trace(d[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)
        return (np.linalg.norm(d[:3, 3]) >= self.p["keyframe_dist"]
                or np.degrees(np.arccos(cos)) >= self.p["keyframe_angle_deg"])

    def add_keyframe(self, stamp: float, pose: np.ndarray, pts: np.ndarray,
                     weights: Optional[np.ndarray] = None):
        kid = len(self.poses)
        self.stamps.append(stamp)
        self.poses.append(pose)
        pts = np.ascontiguousarray(pts, dtype=np.float32)
        if self.p["solid"].use_weight and weights is not None:
            desc = self.extractor.extract(pts, np.ascontiguousarray(weights, dtype=np.float32))
        else:
            desc = self.extractor.extract(pts)
        n = np.linalg.norm(desc.rsolid)
        if n > 0:
            desc.rsolid = desc.rsolid / n

        out = []
        if not self.database.empty():
            for c in self.database.query(desc):
                if c.score < self.p["min_score"]:
                    break
                out.append((kid, int(c.id), float(c.score), float(c.yaw_rad)))
                if len(out) >= self.p["max_candidates"]:
                    break
        self.loops.extend(out)

        self._pending.append((kid, desc))
        while len(self._pending) > self.p["exclude_recent"]:
            self.database.add(*self._pending.popleft())
        return out

    def save(self, out_dir: str) -> None:
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "keyframes.txt"), "w") as f:
            for s, T in zip(self.stamps, self.poses):
                q = rot_to_quat(T[:3, :3])
                f.write(f"{s:.9f} {T[0,3]:.9f} {T[1,3]:.9f} {T[2,3]:.9f} "
                        f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
        with open(os.path.join(out_dir, "loops.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["query_id", "match_id", "query_stamp", "match_stamp", "score", "yaw_rad"])
            for q, m, s, y in self.loops:
                w.writerow([q, m, f"{self.stamps[q]:.6f}", f"{self.stamps[m]:.6f}",
                            f"{s:.6f}", f"{y:.6f}"])


# --------------------------------------------------------------------------- geometry
def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def quat_to_rot(x, y, z, w) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# --------------------------------------------------------------------------- readers
Frame = Tuple[float, np.ndarray, np.ndarray, Optional[np.ndarray]]  # stamp, pose, pts, weights


def read_kitti(seq_dir: str, stride: int) -> Iterator[Frame]:
    """KITTI odometry: velodyne/*.bin with poses.txt (camera frame) + calib.txt."""
    poses = np.loadtxt(os.path.join(seq_dir, "poses.txt")).reshape(-1, 3, 4)
    tr = np.eye(4)
    calib = os.path.join(seq_dir, "calib.txt")
    if os.path.isfile(calib):
        for line in open(calib):
            if line.startswith("Tr:"):
                tr[:3, :4] = np.array(line.split()[1:], dtype=float).reshape(3, 4)
    tr_inv = np.linalg.inv(tr)
    times_file = os.path.join(seq_dir, "times.txt")
    times = np.loadtxt(times_file) if os.path.isfile(times_file) else None
    for i in range(0, len(poses), stride):
        T = np.eye(4)
        T[:3, :4] = poses[i]
        T_lidar = tr_inv @ T @ tr  # camera-frame poses -> LiDAR frame
        scan = np.fromfile(os.path.join(seq_dir, "velodyne", f"{i:06d}.bin"),
                           dtype=np.float32).reshape(-1, 4)
        stamp = float(times[i]) if times is not None else i * 0.1
        yield stamp, T_lidar, scan[:, :3], scan[:, 3]


_PF_DTYPES = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}


def cloud_to_numpy(msg, weight_field: str):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.datatype in _PF_DTYPES and f.count == 1:
            names.append(f.name)
            formats.append(_PF_DTYPES[f.datatype].replace("<", ">" if msg.is_bigendian else "<"))
            offsets.append(f.offset)
    dt = np.dtype({"names": names, "formats": formats, "offsets": offsets,
                   "itemsize": msg.point_step})
    n = msg.width * msg.height
    arr = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
    pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)
    w = None
    if weight_field:
        if weight_field not in arr.dtype.names:
            raise KeyError(f"weight field '{weight_field}' not in cloud fields {arr.dtype.names}")
        w = arr[weight_field].astype(np.float32)
    ok = np.isfinite(pts).all(axis=1)
    return pts[ok], (w[ok] if w is not None else None)


def odom_to_matrix(msg) -> np.ndarray:
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def read_bag(path: str, cloud_topic: str, odom_topic: str, weight_field: str,
             cloud_in_world: bool, max_dt: float = 0.05) -> Iterator[Frame]:
    """ROS 1 .bag or ROS 2 bag folder; each cloud is paired with the nearest odometry."""
    from pathlib import Path
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    def stamp_of(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    bag = Path(path)
    store = get_typestore(Stores.ROS1_NOETIC if bag.suffix == ".bag" else Stores.ROS2_HUMBLE)
    with AnyReader([bag], default_typestore=store) as reader:
        topics = {c.topic for c in reader.connections}
        for t in (cloud_topic, odom_topic):
            if t not in topics:
                raise KeyError(f"topic '{t}' not in bag. Topics: {sorted(topics)}")
        # Odometry is small: load it first so each cloud can find its pose.
        odom_conns = [c for c in reader.connections if c.topic == odom_topic]
        o_stamps, o_poses = [], []
        for conn, _, raw in reader.messages(connections=odom_conns):
            msg = reader.deserialize(raw, conn.msgtype)
            o_stamps.append(stamp_of(msg))
            o_poses.append(odom_to_matrix(msg))
        if not o_stamps:
            raise RuntimeError(f"no messages on {odom_topic}")
        o_stamps = np.asarray(o_stamps)
        order = np.argsort(o_stamps)
        o_stamps = o_stamps[order]
        o_poses = [o_poses[i] for i in order]

        cloud_conns = [c for c in reader.connections if c.topic == cloud_topic]
        for conn, _, raw in reader.messages(connections=cloud_conns):
            msg = reader.deserialize(raw, conn.msgtype)
            s = stamp_of(msg)
            j = int(np.searchsorted(o_stamps, s))
            cands = [k for k in (j - 1, j) if 0 <= k < len(o_stamps)]
            k = min(cands, key=lambda k: abs(o_stamps[k] - s))
            if abs(o_stamps[k] - s) > max_dt:
                continue
            pts, w = cloud_to_numpy(msg, weight_field)
            pose = o_poses[k]
            if cloud_in_world:
                inv = np.linalg.inv(pose)
                pts = (pts @ inv[:3, :3].T + inv[:3, 3]).astype(np.float32)
            yield s, pose, pts, w


# --------------------------------------------------------------------------- output
def plot(det: LoopDetector, path: str, show: bool) -> None:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xy = np.array([T[:2, 3] for T in det.poses]) if det.poses else np.zeros((0, 2))
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(xy[:, 0], xy[:, 1], color="#888888", lw=1.0, label="keyframes")
    for i, (q, m, _, _) in enumerate(det.loops):
        ax.plot([xy[q, 0], xy[m, 0]], [xy[q, 1], xy[m, 1]], color="#d62728", lw=0.8,
                label="SOLiD loops" if i == 0 else None)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"{len(det.poses)} keyframes, {len(det.loops)} loop candidates")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    if show:
        plt.show()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag", help="ROS 1 .bag file or ROS 2 bag folder")
    src.add_argument("--kitti", help="KITTI sequence folder (velodyne/, poses.txt, calib.txt)")
    ap.add_argument("--cloud", default="/cloud", help="PointCloud2 topic (bag)")
    ap.add_argument("--odom", default="/odom", help="nav_msgs/Odometry topic (bag)")
    ap.add_argument("--config", required=True, help="slam/config/*.yaml")
    ap.add_argument("--out", default="solid_results", help="output folder")
    ap.add_argument("--stride", type=int, default=1, help="use every N-th KITTI scan")
    ap.add_argument("--gt-radius", type=float, default=0.0,
                    help="if > 0, count a loop as correct when its keyframes are this close "
                         "(meaningful when the poses are ground truth, e.g. KITTI)")
    ap.add_argument("--show", action="store_true", help="open the plot window")
    args = ap.parse_args()

    p = load_params(args.config)
    det = LoopDetector(p)
    if args.kitti:
        frames = read_kitti(args.kitti, args.stride)
    else:
        frames = read_bag(args.bag, args.cloud, args.odom, p["weight_field"], p["cloud_in_world"])

    print(f"[solid] backend={solid.backend} config={args.config}")
    t0, n_scans, t_desc = time.time(), 0, 0.0
    for stamp, pose, pts, w in frames:
        n_scans += 1
        if not det.is_keyframe(pose):
            continue
        t = time.perf_counter()
        for q, m, s, y in det.add_keyframe(stamp, pose, pts, w):
            print(f"[loop] kf {q:5d} -> {m:5d}  score {s:.3f}  yaw {np.degrees(y):7.1f} deg")
        t_desc += time.perf_counter() - t

    det.save(args.out)
    plot(det, os.path.join(args.out, "loops.png"), args.show)
    n_kf = len(det.poses)
    print(f"[solid] {n_scans} scans, {n_kf} keyframes, {len(det.loops)} loop candidates "
          f"({time.time() - t0:.1f} s total, {1e3 * t_desc / max(n_kf, 1):.2f} ms per keyframe)")
    if args.gt_radius > 0 and det.loops:
        ok = sum(np.linalg.norm(det.poses[q][:3, 3] - det.poses[m][:3, 3]) <= args.gt_radius
                 for q, m, _, _ in det.loops)
        print(f"[solid] {ok}/{len(det.loops)} candidates within {args.gt_radius} m "
              f"(precision {ok / len(det.loops):.3f})")
    print(f"[solid] wrote {args.out}/keyframes.txt, loops.csv, loops.png")


if __name__ == "__main__":
    main()
