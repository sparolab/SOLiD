<div align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="fig/logo_dark.svg" />
    <img src="fig/logo_light.svg" alt="SOLiD: Spatially Organized and Lightweight Global Descriptor" width="560" />
  </picture>
  <br />
  <a href="https://github.com/sparolab/SOLiD/tree/main/cpp"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
  <a href="https://github.com/sparolab/SOLiD/tree/main/python"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
  <a href="#ros-1--ros-2"><img src="https://img.shields.io/badge/ROS-Noetic-blue" /></a>
  <a href="#ros-1--ros-2"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
  <a href="#macos-no-ros"><img src="https://img.shields.io/badge/macOS-000000?logo=apple&logoColor=white" /></a>
  <a href="#docker"><img src="https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white" /></a>
  <br />
  <a href="https://sparolab.github.io/research/solid/"><img src="https://img.shields.io/badge/Project-Page-blue" /></a>
  <a href="https://ieeexplore.ieee.org/abstract/document/10629042"><img src="https://img.shields.io/badge/Paper-IEEE%20RA--L-yellow" /></a>
  <a href="https://arxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
  <a href="https://www.youtube.com/watch?v=4sAWWfZTwLs"><img src="https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white" /></a>
  <br />
  <img src="fig/kitti05_solid.gif" alt="SOLiD loop closures on KITTI 05" width="95%" />
  <p>
    <strong>[IEEE RA-L 2024] Narrowing your FOV with SOLiD: Spatially Organized and Lightweight<br />
    Global Descriptor for FOV-constrained LiDAR Place Recognition</strong>
  </p>
</div>

______________________________________________________________________

SOLiD is a lightweight global descriptor for LiDAR and 4D radar place recognition, built for sensors whose field of view is narrow or partly blocked (solid-state LiDAR, sensor fusion rigs, robot bodies in the way):

- :dart: **FOV-aware**: points are binned by range, elevation and azimuth inside the sensor's real vertical FOV, so a narrow FOV still gives a dense descriptor.
- :compass: **Two signatures**: R-SOLiD (range) is rotation invariant and drives retrieval; A-SOLiD (azimuth) recovers the relative yaw of a loop.
- :zap: **Lightweight**: a few milliseconds per keyframe on a CPU, with no GPU or learned weights.
- :satellite: **Any sensor**: LiDAR uses point occupancy, and 4D radar can weight each point by its return power.

<p align="center"><img src="fig/method.png" alt="SOLiD descriptor" width="85%" /></p>

## :package: What is in this repository

| Folder | What it is | Runs on |
| --- | --- | --- |
| [`cpp/`](cpp) | SOLiD core library (Eigen only, PCL optional) | Linux, macOS |
| [`python/`](python) | `pip install` package: C++ binding, plus a pure-NumPy fallback | Linux, macOS |
| [`slam/ros1/`](slam/ros1) | Loop candidate node for ROS 1 Noetic | Ubuntu 20.04 |
| [`slam/ros2/`](slam/ros2) | Loop candidate node for ROS 2 Humble | Ubuntu 22.04 |
| [`slam/mac/`](slam/mac) | Offline loop detection on rosbags or KITTI, no ROS needed | macOS, Linux |

The ROS nodes and the Mac tool share one keyframe and loop logic ([`slam/common`](slam/common)) and one set of sensor configs ([`slam/config`](slam/config): `kitti`, `ouster64`, `radar_4d`). They find loop candidates only. Registration and pose-graph optimization stay in your SLAM.

______________________________________________________________________

## :wrench: Installation

```bash
git clone https://github.com/sparolab/SOLiD.git
cd SOLiD
```

### Python (Linux / macOS)

```bash
python3 -m venv .venv && source .venv/bin/activate   # or: uv venv && source .venv/bin/activate
pip install .                                        # builds the C++ core (needs CMake + Eigen)
pip install rosbags matplotlib                       # only for slam/mac
```

On macOS, install the build tools first: `xcode-select --install` and `brew install cmake eigen`.

### Docker

```bash
# ROS 1
docker build -f docker/Dockerfile.noetic -t solid:noetic .
# ROS 2
docker build -f docker/Dockerfile.humble -t solid:humble .

xhost +local:docker
docker run -it --rm --net=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v /path/to/bags:/data solid:humble
```

### From source

<details open>
  <summary><strong>ROS 1 (Noetic)</strong></summary>

```bash
sudo apt install libeigen3-dev libyaml-cpp-dev
mkdir -p ~/catkin_ws/src && ln -s "$PWD" ~/catkin_ws/src/SOLiD
cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

</details>

<details open>
  <summary><strong>ROS 2 (Humble)</strong></summary>

```bash
sudo apt install libeigen3-dev libyaml-cpp-dev
mkdir -p ~/ros2_ws/src && ln -s "$PWD" ~/ros2_ws/src/SOLiD
cd ~/ros2_ws && colcon build --base-paths src/SOLiD/slam/ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`--base-paths` keeps colcon from also picking up the ROS 1 package.

</details>

______________________________________________________________________

## :rocket: Run

### ROS 1 / ROS 2

The node takes a point cloud and the odometry that goes with it, from any LiDAR or radar odometry (FAST-LIO2, LIO-SAM, KISS-ICP, GaRLILEO, ...):

```bash
# ROS 1
roslaunch solid_ros solid.launch config:=kitti cloud_topic:=/cloud odom_topic:=/odom save_dir:=$HOME/solid_results
rosbag play --clock kitti00.bag

# ROS 2
ros2 launch solid_ros solid.launch.py config:=kitti cloud_topic:=/cloud odom_topic:=/odom save_dir:=$HOME/solid_results
ros2 bag play kitti00_ros2 --clock
```

| Topic | Type | |
| --- | --- | --- |
| `cloud_topic` (in) | `sensor_msgs/PointCloud2` | in the sensor frame, or set `input.cloud_in_world: true` |
| `odom_topic` (in) | `nav_msgs/Odometry` | pose of the cloud's frame |
| `/solid/loop_candidates` (out) | `solid_ros/LoopCandidate` | query and match keyframe: ids, stamps, poses, score, yaw |
| `/solid/markers` (out) | `visualization_msgs/MarkerArray` | keyframe path and loop lines for RViz |

With `save_dir` set, the node writes `keyframes.txt` (TUM) and `loops.csv` on exit.

### macOS (no ROS)

[`slam/mac/solid_loops.py`](slam/mac/solid_loops.py) reads a ROS 1 bag, a ROS 2 bag or a KITTI sequence directly, with no ROS installed:

```bash
cd slam/mac
# ROS 1 .bag or ROS 2 bag folder
python solid_loops.py --bag kitti00.bag --cloud /cloud --odom /odom --config ../config/kitti.yaml --out results
# KITTI sequence (velodyne/, poses.txt, calib.txt); --gt-radius checks loops against the poses
python solid_loops.py --kitti /data/kitti/sequences/00 --config ../config/kitti.yaml --out results --gt-radius 5
```

It writes `keyframes.txt`, `loops.csv` and `loops.png` (trajectory with loop lines) to `--out`. To try the ROS nodes on KITTI, [`kitti_to_bag.py`](slam/mac/kitti_to_bag.py) turns a sequence into a ROS 1 or ROS 2 bag.

### Python / C++ API

```python
import numpy as np
import solid

cfg = solid.load_config("lidar_velodyne")      # or lidar_ouster, radar_4d, or a YAML path
ext, db = solid.Extractor(cfg), solid.Database(cfg)
db.add(0, ext.extract(points_0))                # points: (N, 3) float32
for c in db.query(ext.extract(points_1)):       # sorted by cosine similarity
    print(c.id, c.score, c.yaw_rad)
```

The C++ API is the same (`solid::Extractor`, `solid::Database`); see [`cpp/README.md`](cpp/README.md).

______________________________________________________________________

## :robot: Run with your agent

Open [Claude Code](https://claude.com/claude-code) or [Codex](https://openai.com/codex/) in an empty folder and paste:

```text
Clone https://github.com/sparolab/SOLiD, read its AGENTS.md, and follow it to build SOLiD and run the loop detector on KITTI 00 on this machine. Ask me what you need along the way.
```

The agent follows [`AGENTS.md`](AGENTS.md): it picks ROS 1, ROS 2 or the Mac tool, builds it, runs it on KITTI 00, and checks the loops.

______________________________________________________________________

## :gear: Configuration

One YAML file per sensor in [`slam/config`](slam/config), read by all three front-ends:

| Section | Keys | |
| --- | --- | --- |
| `solid` | `fov_up`, `fov_down`, `num_height`, `num_range`, `num_angle`, `min_range`, `max_range`, `voxel_size`, `use_weight` | descriptor; set the FOV to your sensor |
| `keyframe` | `dist`, `angle_deg` | new keyframe after this much motion |
| `loop` | `exclude_recent`, `min_score`, `max_candidates` | skip the latest keyframes; minimum R-SOLiD cosine similarity |
| `input` | `cloud_in_world`, `weight_field` | cloud frame; per-point weight field (radar power) |

On KITTI 00, `min_score: 0.99` gives a precision of 1.00 and a recall of 0.80 (a revisit counts within 5 m). Retune `min_score` for a new sensor: run the Mac tool with `--gt-radius` on a sequence with ground truth.

______________________________________________________________________

## :link: SOLiD in other projects

- **Multi-robot SLAM**: [SKiD-SLAM](https://github.com/sparolab/SKiD-SLAM), [Distributed-SOLiD-SLAM](https://github.com/sparolab/Distributed-SOLiD-SLAM)
- **Single-robot SLAM**: [SOLiD-A-LOAM](https://github.com/sparolab/SOLiD-A-LOAM), [SOLiD-PyICP-SLAM](https://github.com/sparolab/SOLiD-PyICP-SLAM)
- **Benchmarks**: [HeLiPR Place Recognition Toolbox](https://github.com/minwoo0611/HeLiPR-Place-Recognition), [awesome-lidar-place-recognition](https://github.com/hogyun2/awesome-lidar-place-recognition)

## Citation

If you use SOLiD in your research, please cite our [paper](https://ieeexplore.ieee.org/abstract/document/10629042).

<details>
  <summary><strong>See bibtex</strong></summary>

```bibtex
@article{kim2024narrowing,
  title     = {Narrowing your FOV with SOLiD: Spatially Organized and Lightweight Global Descriptor for FOV-constrained LiDAR Place Recognition},
  author    = {Kim, Hogyun and Choi, Jiwon and Sim, Taehu and Kim, Giseop and Cho, Younggun},
  journal   = {IEEE Robotics and Automation Letters},
  year      = {2024},
  publisher = {IEEE}
}
```

</details>

## Contributing

We would love to see SOLiD grow with the community. Open an issue or a Pull Request, and your face joins the list below!

<a href="https://github.com/sparolab/SOLiD/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=sparolab/SOLiD" />
</a>

## Contact

[Hogyun Kim](https://scholar.google.com/citations?user=t5UEbooAAAAJ&hl=ko), [Spatial AI and Robotics Lab (SPARO)](https://sites.google.com/view/sparo), Inha University. For questions, open an issue or comment on [alphaXiv](https://www.alphaxiv.org/abs/2408.07330).
