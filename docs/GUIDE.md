# Run guide

Everything below runs the same C++ library ([`cpp/`](../cpp)): the ROS 1 and ROS 2 nodes are thin wrappers, and the macOS tools call the library directly. Parameters come from one YAML per sensor in [`slam/config/`](../slam/config) (see the [configuration guide](../slam/config/GUIDE.md)).

## ROS 1 / ROS 2: online loop closing

With one of the bundled odometries (git submodules in [`thirdparty/`](../thirdparty)):

```bash
roslaunch solid_ros run.launch odometry:=fast_lio save_dir:=$HOME/solid_results       # fast_lio, kiss_icp, legkilo, fast_lio_mid360 (rviz:=false to skip RViz)
ros2 launch solid_ros run.launch.py odometry:=georio save_dir:=$HOME/solid_results    # + georio, garlileo
```

With any other odometry, start `solid_node` on its output:

`solid_node` takes a point cloud and the odometry that goes with it, from any LiDAR or radar odometry. Per keyframe it builds a SOLiD descriptor, finds loop candidates, checks them with GICP between keyframe submaps and adds the accepted loops to an iSAM2 pose graph, online.

```bash
# ROS 1
roslaunch solid_ros solid.launch config:=kitti cloud_topic:=/cloud odom_topic:=/odom save_dir:=$HOME/solid_results
# ROS 2
ros2 launch solid_ros solid.launch.py config:=kitti cloud_topic:=/cloud odom_topic:=/odom save_dir:=$HOME/solid_results
```

| Argument | Default | |
| --- | --- | --- |
| `config` | `kitti` | a name in [`slam/config/`](../slam/config) (`kitti`, `ouster64`, `vlp16`, `livox_mid360`, `radar_4d`) or a path to a YAML file |
| `cloud_topic` | `/cloud` | `sensor_msgs/PointCloud2` in the sensor frame (or set `input.cloud_in_world`) |
| `odom_topic` | `/odom` | `nav_msgs/Odometry` of the body; `input.extrinsic` gives the sensor in the body frame |
| `save_dir` | empty | where to save the session on exit |
| `rviz` | `true` | open RViz: loop-closed map, trajectory, loops and pose (`run.launch` too) |
| `map_voxel`, `map_period` (node params) | `0.5`, `2.0` | leaf (m) of the published map per keyframe, seconds between map updates |
| `use_sim_time` (ROS 2) | `true` | |

| Output (ROS 1: `~`, ROS 2: `solid/`) | Type | |
| --- | --- | --- |
| `odometry` | `nav_msgs/Odometry` | the input odometry corrected by the pose graph, for every scan (frame `map`) |
| `path` | `nav_msgs/Path` | corrected keyframe poses |
| `loop_candidates` | `solid_ros/LoopCandidate` | accepted loops: keyframe ids, stamps, poses, SOLiD score and yaw, GICP relative pose |
| `markers` | `visualization_msgs/MarkerArray` | loop-closed keyframe trajectory and accepted loops (frame `map`) |
| `map` | `sensor_msgs/PointCloud2` | the keyframe clouds at their loop-closed poses (frame `map`, updated every `map_period` s and after every loop; only while someone subscribes) |
| TF `map` → odometry frame | | the loop-closing correction, so the odometry's own clouds line up with the map in RViz (`publish_tf`) |

With `loop_closing.enable: false` in the config, the node only proposes candidates (all of them, `verified: false`) for your own back-end.

Live results (odometry and `solid_node` running together on the played bag): KITTI 00 with KISS-ICP 3.70 → 1.13 m, SNAIL-Radar with GeoRIO 0.47 → 0.19 m, GaRLILEO BridgeLoop 0.37 → 0.30 m; all runs in the [configuration guide](../slam/config/GUIDE.md#loop-closing-drift-before-and-after).

### With an odometry

Ready-made launch files, configs, macOS scripts and Docker images are per platform in [`slam/`](../slam):

| Platform | Odometry | ROS | Cloud / odometry | SOLiD config |
| --- | --- | --- | --- | --- |
| Car, LiDAR | [KISS-ICP](https://github.com/PRBonn/kiss-icp) | 1, 2 | `/kiss/frame`, `/kiss/odometry` | `vlp16` (Kimera-Multi) |
| Car / UGV, LiDAR | [FAST-LIO](https://github.com/hku-mars/FAST_LIO) | 1, 2 | raw LiDAR, `/Odometry` (IMU pose) | `slam/lidar/config/kimera_sparkal1.yaml` (with the LiDAR extrinsic) |
| 4D radar | [GeoRIO](https://github.com/wooseongY/GeoRIO) | 2 | `/current_scan` (odometry frame), `/odometry_georio` | `slam/radar/config/georio.yaml` |
| Legged, LiDAR | [Leg-KILO](https://github.com/ouguangjun/Leg-KILO) | 1, 2 | `/cloud_registered` (odometry frame), `/Odometry` | `slam/legged/config/legkilo.yaml` |
| Legged, 4D radar | [GaRLILEO](https://github.com/ChiyunNoh/GaRLILEO) | 2 | `/ti_mmwave/radar_scan_pcl_0`, `/garlileo/odometry` (time relay) | `slam/legged/config/garlileo.yaml` |
| Humanoid, Livox Mid-360 | [FAST-LIO](https://github.com/hku-mars/FAST_LIO) | 1, 2 | `/cloud_registered_body`, `/Odometry` | `livox_mid360` |

GeoRIO and GaRLILEO are ROS 2 only.

## macOS / Linux without ROS

[`slam/common/mac/solid_loops.py`](../slam/common/mac/solid_loops.py) is the front-end (descriptors, keyframes, candidates) on a ROS 1 bag, a ROS 2 bag or a KITTI sequence; `--close-loops` then runs the C++ back-end tool `solid_close_loops` on the same session.

```bash
cd slam/common/mac
python solid_loops.py --bag kitti00.bag --cloud /cloud --odom /odom --config ../../config/kitti.yaml --out results --close-loops
python solid_loops.py --kitti /data/kitti/sequences/00 --poses kiss_icp_poses.txt --config ../../config/kitti.yaml --out results --close-loops
```

| Option | |
| --- | --- |
| `--bag` / `--kitti` | input: ROS 1 `.bag` or ROS 2 bag folder, or a KITTI sequence (`velodyne/`, `poses.txt`, `calib.txt`) |
| `--cloud`, `--odom` | topics in the bag |
| `--poses` | KITTI: odometry poses (KITTI format, LiDAR frame) instead of the ground truth |
| `--odom-time-offset` | seconds added to odometry stamps (GaRLILEO stamps relative to its start) |
| `--config` | a file in `slam/config/` |
| `--out` | output folder: the session below, plus `loops.png` |
| `--close-loops` | run `solid_close_loops` (build `cpp/` with `-DSOLID_BUILD_BACKEND=ON`, put it on `PATH` or set `$SOLID_TOOLS`) |
| `--range START:END`, `--rebase`, `--offset=X,Y,YAW` | cut one sequence into sessions, express it in its own frame, or move it (multi-session tests) |
| `--gt-radius R` | count a candidate as correct when its keyframes are within R m (when the poses are ground truth) |

The C++ tools also run on their own: `solid_close_loops <session> <config.yaml>` and `solid_align_sessions <config.yaml> <out> <central> <session> ...`. [`slam/common/mac/kitti_to_bag.py`](../slam/common/mac/kitti_to_bag.py) turns a KITTI sequence into a ROS 1 or ROS 2 bag (`/cloud`, `/odom`; `--poses` for an odometry instead of the ground truth).

## Output: a session

With `save_dir` (ROS) or `--out` (macOS), every run is saved as an [LT-mapper](https://github.com/gisbi-kim/lt-mapper) session, ready for [multi-session SLAM](../tools/multi-session-slam):

```
<session>/
├── Scans/000000.pcd               keyframe cloud (the merged submap), sensor frame (x y z intensity)
├── SCDs/000000.scd                Scan Context (20 x 60), for LT-SLAM
├── SOLiDs/000000.txt              SOLiD descriptor (R-SOLiD, A-SOLiD)
├── singlesession_posegraph.g2o    keyframe poses (optimized if loop closing ran) + odometry and loop edges
├── odom_poses.txt, times.txt      odometry poses (KITTI), keyframe stamps
├── optimized_poses.txt            poses after loop closing (KITTI)
└── keyframes.txt, loops.csv       TUM poses, SOLiD candidates
```

## Python / C++ API

```python
import solid
cfg = solid.load_config("lidar_velodyne")      # or a YAML path
ext, db = solid.Extractor(cfg), solid.Database(cfg)
db.add(0, ext.extract(points_0))                # points: (N, 3) float32
for c in db.query(ext.extract(points_1)):       # sorted by cosine similarity
    print(c.id, c.score, c.yaw_rad)

det = solid.slam.LoopDetector(solid.slam.Params())   # the same front-end as the ROS nodes
```

```cmake
find_package(solid REQUIRED COMPONENTS slam backend)
target_link_libraries(my_slam solid::backend)   # solid::core, solid::slam, solid::backend
```

See [`cpp/README.md`](../cpp/README.md) for the library targets.
