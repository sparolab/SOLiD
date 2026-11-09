# Integrations

SOLiD on top of real odometry systems, grouped by platform. Each folder has launch files (ROS 1 / ROS 2), the SOLiD config for that setup, and helper tools; the Docker images are in [`docker/integrations`](../docker/integrations). Every setup runs online loop closing (`loop_closing.enable`).

| Folder | Odometry | ROS | Tested on | Docker |
| --- | --- | --- | --- | --- |
| [`lidar/`](lidar) | [FAST-LIO](https://github.com/hku-mars/FAST_LIO) | 1: `ros1/fastlio_solid.launch`, 2: `ros2/fastlio_solid.launch.py` | Kimera-Multi `sparkal1` (VLP-16) | `fastlio.noetic`, `fastlio.humble` |
| [`lidar/`](lidar) | [KISS-ICP](https://github.com/PRBonn/kiss-icp) | 1 (`v0.3.0`): `ros1/kiss_solid.launch`, 2: `ros2/kiss_solid.launch.py` | Kimera-Multi, KITTI 00 | `kiss.noetic`, `kiss.humble` |
| [`radar/`](radar) | [GeoRIO](https://github.com/wooseongY/GeoRIO) | 2: `ros2/georio_solid.launch.py` | SNAIL-Radar (ARS548) | `georio.humble` |
| [`legged/`](legged) | [GaRLILEO](https://github.com/ChiyunNoh/GaRLILEO) | 2: `ros2/garlileo_solid.launch.py` | GaRLILEO dataset (Spot, TI IWR1843) | `garlileo.humble` |
| [`humanoid/`](humanoid) | [FAST-LIO](https://github.com/hku-mars/FAST_LIO) with a Livox Mid-360 | 1: `ros1/fastlio_mid360_solid.launch`, 2: `ros2/fastlio_mid360_solid.launch.py` | KILVO (Unitree G1) | `fastlio.noetic`, `fastlio.humble` |

```bash
# build: the base image first, then the integration
docker build -f docker/Dockerfile.humble -t solid:humble .
docker build -f docker/integrations/Dockerfile.georio.humble -t solid-georio:humble .

# run, e.g. GeoRIO on SNAIL-Radar
ros2 launch integrations/radar/ros2/georio_solid.launch.py save_dir:=/out
ros2 bag play <SNAIL-Radar ROS 2 bag>
```

Notes from testing:
- **Frames.** SOLiD needs the cloud in a z-up sensor frame and the pose of that sensor. FAST-LIO's `/Odometry` is the IMU pose: give SOLiD the raw LiDAR scan plus `input.extrinsic` (LiDAR in IMU), as in `lidar/config/kimera_sparkal1.yaml`. GeoRIO's `/current_scan` is in the odometry frame: `input.cloud_in_world: true`.
- **GaRLILEO** stamps `/Odometry` relative to its first IMU message; `legged/tools/garlileo_odom_relay.py` (started by the launch file) republishes it with absolute stamps. Its TI radar gives ~30 points per scan, so `garlileo.yaml` merges 50 scans per keyframe.
- **KILVO** bags carry `livox_ros_driver2` messages; ROS 1 FAST-LIO reads `livox_ros_driver`, so convert them first with `humanoid/tools/livox2_to_livox1.py`. Its Mid-360 is mounted upside down; SOLiD's elevation bins are symmetric, so only the FOV matters.
- **KISS-ICP (ROS 1)** keeps a queue of one scan: play bags at 1x. Kimera-Multi VLP-16 clouds have no per-point time: run it with `deskew:=false`.
- **GaRLILEO Docker**: GaRLILEO builds its own PCL, so its image is built on GaRLILEO's own Dockerfile (`docker build -t garlileo-deps https://github.com/ChiyunNoh/GaRLILEO.git#main:docker`) and skips `align_node`.
