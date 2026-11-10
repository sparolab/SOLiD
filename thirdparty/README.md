# Odometry submodules

The odometry systems SOLiD was tested with, pinned to the tested commits. Get them with

```bash
git clone --recursive https://github.com/sparolab/SOLiD.git     # or: git submodule update --init --recursive
```

| Folder | Odometry | ROS | Commit |
| --- | --- | --- | --- |
| `FAST_LIO` | [FAST-LIO](https://github.com/hku-mars/FAST_LIO) (main) | 1 | `7cc4175` |
| `FAST_LIO_ROS2` | FAST-LIO (`ROS2` branch) | 2 | `cc3c9e8` |
| `kiss-icp_ros1` | [KISS-ICP](https://github.com/PRBonn/kiss-icp) `v0.3.0` (last release with ROS 1) | 1 | `460c6f2` |
| `kiss-icp` | KISS-ICP `v1.3.0` | 2 | `b168352` |
| `GeoRIO` | [GeoRIO](https://github.com/wooseongY/GeoRIO) | 2 | `f1f1e2b` |
| `GaRLILEO`, `SPOT_ego_Velocity` | [GaRLILEO](https://github.com/ChiyunNoh/GaRLILEO) and its leg velocity | 2 | `4c4d3b1`, `c2d61bd` |
| `Leg-KILO` | [Leg-KILO](https://github.com/ouguangjun/Leg-KILO) | 1, 2 | `f3a5bd2` |
| `livox_ros_driver`, `livox_ros_driver2` | Livox messages for FAST-LIO | 1, 2 | `3d240d5`, `2144554` |

`CATKIN_IGNORE` / `COLCON_IGNORE` keep a plain workspace build from picking up all of them (FAST-LIO and KISS-ICP each appear twice). Build the ones you need explicitly, e.g. `colcon build --base-paths src/SOLiD/slam/common/ros2 src/SOLiD/thirdparty/GeoRIO`, or link a ROS 1 package into your catkin workspace (`ln -s $PWD/thirdparty/FAST_LIO ~/catkin_ws/src/`). The Docker images in [`docker/slam`](../docker/slam) do this for you.
