# Multi-session SLAM

Every SOLiD front-end (ROS 1 node, ROS 2 node, `slam/mac/solid_loops.py`) saves its run as an
[LT-mapper](https://github.com/gisbi-kim/lt-mapper) session when `save_dir` / `--out` is set:

```
<session>/
├── Scans/000000.pcd               keyframe cloud, sensor frame (x y z intensity)
├── SCDs/000000.scd                Scan Context (20 x 60), read by LT-SLAM
├── SOLiDs/000000.txt              SOLiD descriptor (R-SOLiD, A-SOLiD)
├── singlesession_posegraph.g2o    keyframe poses + odometry edges
├── odom_poses.txt, times.txt      KITTI poses, keyframe stamps
└── keyframes.txt, loops.csv       TUM poses, SOLiD loop candidates
```

## Align two sessions with LT-SLAM (ROS 1, Docker)

```bash
docker pull dongjae0107/lt-mapper:latest
./ros1/run_ltslam.sh <sessions_dir> <central_session> <query_session> [out_dir]
```

The aligned poses of the query session, in the central session's frame, are written to
`<out_dir>/<query>_central_aft_intersession_loops.txt` (KITTI format).

### Example: KITTI 00 cut into two sessions

```bash
cd slam/mac
python solid_loops.py --bag kitti00.bag --config ../config/kitti.yaml --out sessions/00a --range 0:2000 --rebase
python solid_loops.py --bag kitti00.bag --config ../config/kitti.yaml --out sessions/00b --range 2000:4541 --rebase
../../multi_session_slam/ros1/run_ltslam.sh sessions 00a 00b
```

| Inter-session loops on KITTI 00 (00a: scans 0-1999, 00b: 2000-4540) | Pairs | Within 5 m |
| --- | --- | --- |
| Scan Context (LT-SLAM default) | 542 | 74.5 % |
| SOLiD (`min_score` 0.99) | 350 | 100 % |

LT-SLAM accepts the wrong Scan Context pairs through its ICP check, so 00b ends up 30 m (ATE) from
the ground truth. The SOLiD descriptors are already saved in every session (`SOLiDs/`) for a
SOLiD-based inter-session search.
