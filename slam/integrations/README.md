# Integrations

Launch files that run an odometry together with the SOLiD loop candidate node. They were tested on Kimera-Multi `10_14_sparkal1` (VLP-16 + IMU); change the topics and the odometry config for your robot. Results are in the [configuration guide](../config/GUIDE.md#tuning-min_score).

| File | Odometry | ROS |
| --- | --- | --- |
| `ros1/fastlio_solid.launch` | [FAST-LIO](https://github.com/hku-mars/FAST_LIO) (`fast_lio/kimera_sparkal1_ros1.yaml`) | Noetic |
| `ros1/kiss_solid.launch` | [KISS-ICP](https://github.com/PRBonn/kiss-icp) `v0.3.0` | Noetic |
| `ros2/fastlio_solid.launch.py` | [sparo-fast-lio](https://github.com/sparolab/sparo-fast-lio) | Humble |
| `ros2/kiss_solid.launch.py` | [KISS-ICP](https://github.com/PRBonn/kiss-icp) `v1.3` | Humble |

```bash
roslaunch slam/integrations/ros1/fastlio_solid.launch save_dir:=$HOME/solid_results    # then: rosbag play --clock <bag>
ros2 launch slam/integrations/ros2/kiss_solid.launch.py topic:=/sparkal1/lidar_points save_dir:=$HOME/solid_results
```

Notes from testing:
- FAST-LIO's body frame is the IMU frame. If the IMU is not z-up (e.g. a camera's optical IMU), give SOLiD the raw LiDAR scan, not `/cloud_registered_body`.
- KISS-ICP (ROS 1) keeps a queue of one scan: play the bag at 1x, or it drops scans and drifts.
- The Kimera-Multi VLP-16 clouds have no per-point time: run KISS-ICP with `deskew:=false`.
