# Loop detection guide

The SOLiD loop candidate front-ends: a ROS 1 node, a ROS 2 node and a ROS-free tool for macOS / Linux. All three run the same C++ code (`solid::slam` in [`cpp/`](../cpp)) and read the same [configs](config/GUIDE.md).

## ROS 1 / ROS 2 node

The node takes a point cloud and the odometry that goes with it, from any LiDAR or radar odometry:

```bash
# ROS 1
roslaunch solid_ros solid.launch config:=kitti cloud_topic:=/cloud odom_topic:=/odom save_dir:=$HOME/solid_results
# ROS 2
ros2 launch solid_ros solid.launch.py config:=kitti cloud_topic:=/cloud odom_topic:=/odom save_dir:=$HOME/solid_results
```

| Argument | Default | |
| --- | --- | --- |
| `config` | `kitti` | a name in [`slam/config`](config) (`kitti`, `ouster64`, `vlp16`, `radar_4d`) or a path to a YAML file |
| `cloud_topic` | `/cloud` | `sensor_msgs/PointCloud2` in the sensor frame (or set `input.cloud_in_world`) |
| `odom_topic` | `/odom` | `nav_msgs/Odometry`, the pose of the cloud's frame |
| `save_dir` | empty | where to save the session on exit (see below) |
| `rviz` | `true` | open RViz |
| `use_sim_time` (ROS 2) | `true` | |

| Output topic | Type | |
| --- | --- | --- |
| `/solid/loop_candidates` | `solid_ros/LoopCandidate` | query and match keyframe: ids, stamps, poses, R-SOLiD score, yaw |
| `/solid/markers` | `visualization_msgs/MarkerArray` | keyframe path and loop lines |

The node only proposes loops. Verify them (e.g. ICP) and add them to your pose graph in the SLAM back-end.

### With an odometry

| Odometry | Sensor | `cloud_topic` | `odom_topic` | Notes |
| --- | --- | --- | --- | --- |
| [FAST-LIO](https://github.com/hku-mars/FAST_LIO) (ROS 1) | LiDAR | the raw LiDAR topic | `/Odometry` | FAST-LIO's body frame is the IMU frame: do not use `/cloud_registered_body` if the IMU is not aligned with the LiDAR |
| [FAST-LIO](https://github.com/sparolab/sparo-fast-lio) (ROS 2) | LiDAR | `/<robot>/cloud_registered_lidar` | `/<robot>/odometry` | set `visualization_frame: lidar` |
| [KISS-ICP](https://github.com/PRBonn/kiss-icp) | LiDAR | `/kiss/frame` | `/kiss/odometry` | ROS 1: `v0.3.0`, the last release with ROS 1 |
| [GeoRIO](https://github.com/wooseongY/GeoRIO) (ROS 2) | 4D radar | `/current_scan` | `/odometry_georio` | `config:=slam/integrations/config/georio.yaml` (the scan is in the odometry frame) |
| [GaRLILEO](https://github.com/ChiyunNoh/GaRLILEO) (ROS 2) | 4D radar | | | `config:=radar_4d` (not tested yet) |

Ready-made launch files for these are in [`integrations/`](integrations).

## macOS / Linux without ROS

[`mac/solid_loops.py`](mac/solid_loops.py) reads a ROS 1 bag, a ROS 2 bag or a KITTI sequence directly:

```bash
cd slam/mac
python solid_loops.py --bag kitti00.bag --cloud /cloud --odom /odom --config ../config/kitti.yaml --out results
python solid_loops.py --kitti /data/kitti/sequences/00 --config ../config/kitti.yaml --out results --gt-radius 5
```

| Option | |
| --- | --- |
| `--bag` / `--kitti` | input: ROS 1 `.bag` or ROS 2 bag folder, or a KITTI sequence (`velodyne/`, `poses.txt`, `calib.txt`) |
| `--cloud`, `--odom` | topics in the bag |
| `--config` | a file in `slam/config` |
| `--out` | output folder (the session below, plus `loops.png`) |
| `--range START:END`, `--rebase` | use only some scans, with poses relative to the first one: cuts one sequence into sessions |
| `--gt-radius R` | count a loop as correct when its keyframes are within R m (when the poses are ground truth) |
| `--show` | open the plot |

[`mac/kitti_to_bag.py`](mac/kitti_to_bag.py) turns a KITTI sequence into a ROS 1 or ROS 2 bag (`/cloud`, `/odom`).

## Output: a session

With `save_dir` (ROS) or `--out` (Mac), every front-end writes the run as an [LT-mapper](https://github.com/gisbi-kim/lt-mapper) session, ready for [multi-session SLAM](../multi_session_slam/GUIDE.md):

```
<session>/
├── Scans/000000.pcd               keyframe cloud, sensor frame (x y z intensity)
├── SCDs/000000.scd                Scan Context (20 x 60), for LT-SLAM
├── SOLiDs/000000.txt              SOLiD descriptor (R-SOLiD, A-SOLiD)
├── singlesession_posegraph.g2o    keyframe poses + odometry edges
├── odom_poses.txt, times.txt      KITTI poses, keyframe stamps
└── keyframes.txt, loops.csv       TUM poses, SOLiD loop candidates
```

## Python / C++ API

```python
import solid
cfg = solid.load_config("lidar_velodyne")      # or a YAML path
ext, db = solid.Extractor(cfg), solid.Database(cfg)
db.add(0, ext.extract(points_0))                # points: (N, 3) float32
for c in db.query(ext.extract(points_1)):       # sorted by cosine similarity
    print(c.id, c.score, c.yaw_rad)

det = solid.slam.LoopDetector(solid.slam.Params())   # the same detector as the ROS nodes
```

```cmake
find_package(solid REQUIRED COMPONENTS slam)
target_link_libraries(my_slam solid::slam)
```

See [`cpp/README.md`](../cpp/README.md) for the library targets.
