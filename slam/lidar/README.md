# slam/lidar — SOLiD × spark-fast-lio2 (ROS2)

**Target:** [spark-fast-lio2](https://github.com/MIT-SPARK/spark-fast-lio2) — LiDAR-inertial odometry, **ROS2 (Jazzy), native**.

**Role:** decoupled loop-candidate module. Subscribes to spark-fast-lio2's
`(registered cloud, odometry)`, runs `solid_core` (Extractor + Database) per
keyframe, and publishes loop candidates (keyframe i↔j + yaw + score). Downstream
registration/PGO is handled by the SLAM (or KISS-Matcher-SAM).

**Flavor:** ROS2 ament package, links `solid_core` (`../../cpp`).

**Profile:** `lidar_ouster` / `lidar_velodyne` (see cpp `Config`).

Status: **planned** (blocked on a ROS2 Jazzy environment on this machine).

Expected topics (remap in launch):
- in:  `~input/cloud` ← `/cloud_registered`, `~input/odom` ← `/Odometry`
- out: `~output/loop_candidates`
