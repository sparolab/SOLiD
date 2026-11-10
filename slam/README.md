# SLAM with SOLiD

```
slam/
├── common/            the SOLiD front-ends every platform uses
│   ├── ros1/          ROS 1 package solid_ros: solid_node (loop closing)
│   │   └── launch/    run.launch (odometry + SOLiD), solid.launch (SOLiD only)
│   ├── ros2/          ROS 2 package solid_ros: the same nodes
│   │   └── launch/    run.launch.py, solid.launch.py
│   └── mac/           ROS-free tools: solid_loops.py (front-end), kitti_to_bag.py
├── config/            one YAML per sensor (GUIDE.md)
├── lidar/             cars and UGVs with a LiDAR
├── radar/             4D imaging radar
├── legged/            legged robots
└── humanoid/          humanoids
```

The odometries are git submodules in [`thirdparty/`](../thirdparty) (pinned to the tested commits; `git clone --recursive`). One entry point picks the odometry and runs it live with SOLiD:

```bash
roslaunch solid_ros run.launch odometry:=fast_lio save_dir:=/out           # ROS 1: fast_lio, kiss_icp, legkilo, fast_lio_mid360
ros2 launch solid_ros run.launch.py odometry:=georio save_dir:=/out        # ROS 2: + georio, garlileo
```

Each platform folder has the same layout: `ros1/` and `ros2/` (launch files that run an odometry together with `solid_node`), `mac/` (scripts that run SOLiD offline on a bag where that odometry was recorded) and `config/` (the SOLiD config for that setup). Every setup closes loops online (`loop_closing.enable`): the odometry runs live and `solid_node` optimizes on its output as it arrives. The results below are from such live runs (details in the [configuration guide](config/GUIDE.md#loop-closing-drift-before-and-after)). Docker images: [`docker/slam`](../docker/slam).

| Platform | Odometry | ROS 1 | ROS 2 | macOS | Tested on | Result (live) |
| --- | --- | --- | --- | --- | --- | --- |
| `lidar` | [KISS-ICP](https://github.com/PRBonn/kiss-icp) | `kiss_solid.launch` (`v0.3.0`) | `kiss_solid.launch.py` | `kiss_icp.sh` | KITTI 00, Kimera-Multi | ATE 3.70 → 1.13 m |
| `lidar` | [FAST-LIO](https://github.com/hku-mars/FAST_LIO) | `fastlio_solid.launch` | `fastlio_solid.launch.py` | `fast_lio.sh` | Kimera-Multi `sparkal1` (VLP-16) | ATE 1.30 → 1.29 m |
| `radar` | [GeoRIO](https://github.com/wooseongY/GeoRIO) | – | `georio_solid.launch.py` | `georio.sh` | SNAIL-Radar (ARS548) | ATE 0.47 → 0.19 m |
| `legged` | [GaRLILEO](https://github.com/ChiyunNoh/GaRLILEO) | – | `garlileo_solid.launch.py` | `garlileo.sh` | GaRLILEO (Spot, TI IWR1843) | ATE 0.37 → 0.30 m |
| `legged` | [Leg-KILO](https://github.com/ouguangjun/Leg-KILO) | `legkilo_solid.launch` | `legkilo_solid.launch.py` | `legkilo.sh` | Leg-KILO `corridor` (Go1, VLP-16) | ATE 0.22 m (already accurate) |
| `humanoid` | [FAST-LIO](https://github.com/hku-mars/FAST_LIO) on a Livox Mid-360 | `fastlio_mid360_solid.launch` | `fastlio_mid360_solid.launch.py` | `fast_lio_mid360.sh` | [KILVO](https://github.com/JixinGao/KILVO) (Unitree G1) | 19 loops kept, 9 rejected |

GeoRIO and GaRLILEO are ROS 2 only.

```bash
# the odometry image, on the published base ghcr.io/sparolab/solid:humble (clone with --recursive)
docker build -f docker/slam/Dockerfile.georio.humble -t solid-georio:humble .

# online: odometry + SOLiD
ros2 launch solid_ros run.launch.py odometry:=georio save_dir:=/out
ros2 bag play <SNAIL-Radar ROS 2 bag>

# offline on macOS: record the odometry topics once, then
slam/radar/mac/georio.sh georio_out_bag /out
```

Notes from testing:
- **Frames.** SOLiD needs the cloud in a z-up sensor frame and the pose of that sensor. FAST-LIO's `/Odometry` is the IMU pose: give SOLiD the raw LiDAR scan and `input.extrinsic` (LiDAR in IMU), as in `lidar/config/kimera_sparkal1.yaml`. GeoRIO's `/current_scan` and Leg-KILO's `/cloud_registered` are in the odometry frame: `input.cloud_in_world: true`.
- **GaRLILEO** stamps `/Odometry` relative to its first IMU message; `legged/ros2/garlileo_odom_relay.py` (started by the launch file) republishes it with absolute stamps. Its TI radar gives ~30 points per scan, so `garlileo.yaml` merges 50 scans per keyframe. Its image is built on GaRLILEO's own Dockerfile (`docker build -t garlileo-deps https://github.com/ChiyunNoh/GaRLILEO.git#main:docker`) and builds only `solid_ros`.
- **Leg-KILO** always opens its own viewer; without a display run it under `xvfb-run -a` (installed in its image).
- **[KILVO](https://github.com/JixinGao/KILVO)** bags carry `livox_ros_driver2` messages; ROS 1 FAST-LIO reads `livox_ros_driver`, so convert them with `humanoid/ros1/livox2_to_livox1.py`. Its Mid-360 is mounted upside down; SOLiD's elevation bins are symmetric, so only the FOV matters.
- **KISS-ICP (ROS 1)** keeps a queue of one scan: play bags at 1x. Kimera-Multi VLP-16 clouds have no per-point time: run it with `deskew:=false`.
