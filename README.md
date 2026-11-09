<div align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="fig/logo_dark.svg" />
    <img src="fig/logo_light.svg" alt="SOLiD: Spatially Organized and Lightweight Global Descriptor" width="560" />
  </picture>
  <br />
  <a href="https://github.com/sparolab/SOLiD/tree/main/cpp"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
  <a href="https://github.com/sparolab/SOLiD/tree/main/python"><img src="https://img.shields.io/badge/Python-3670A0?logo=python&logoColor=ffdd54" /></a>
  <a href="https://github.com/sparolab/SOLiD/tree/main/slam/ros1"><img src="https://img.shields.io/badge/ROS-Noetic-blue" /></a>
  <a href="https://github.com/sparolab/SOLiD/tree/main/slam/ros2"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
  <a href="https://github.com/sparolab/SOLiD/tree/main/slam/mac"><img src="https://img.shields.io/badge/macOS-000000?logo=apple&logoColor=white" /></a>
  <a href="#docker"><img src="https://img.shields.io/badge/Docker-2496ED?logo=docker&logoColor=white" /></a>
  <br />
  <a href="https://sparolab.github.io/research/solid/"><img src="https://img.shields.io/badge/Project-Page-blue" /></a>
  <a href="https://ieeexplore.ieee.org/abstract/document/10629042"><img src="https://img.shields.io/badge/Paper-IEEE%20RA--L-yellow" /></a>
  <a href="https://arxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
  <a href="https://www.dropbox.com/scl/fo/qgsj4cj03vziul4e0xnbr/AD1KSBVdeMMk6rUpFaxD8kE?rlkey=7ecyaowj1lhtjfatnei4pd3ye&dl=0"><img src="https://img.shields.io/badge/Dataset-Dropbox-0061FF?logo=dropbox&logoColor=white" /></a>
  <a href="https://www.youtube.com/watch?v=4sAWWfZTwLs"><img src="https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white" /></a>
  <br />
  <img src="fig/solid_demo.gif" alt="SOLiD on KITTI 00: the map builds up, the trajectory rises with time, and each loop SOLiD finds links the current pose down to the place it recognized" width="95%" />
  <p>
    <strong>[IEEE RA-L 2024] Narrowing your FOV with SOLiD: Spatially Organized and Lightweight<br />
    Global Descriptor for FOV-constrained LiDAR Place Recognition</strong>
  </p>
</div>

______________________________________________________________________

A robot recognizes where it has been before, even when its sensor sees only a narrow or partly blocked slice of the world:

- :dart: **FOV-aware**: points are binned by range, elevation and azimuth inside the sensor's real vertical field of view.
- :compass: **Two signatures**: R-SOLiD (range) finds the place, A-SOLiD (azimuth) recovers the yaw between the two visits.
- :zap: **Lightweight**: about 1.5 ms per keyframe on a CPU, with no GPU and no training.
- :satellite: **Any range sensor**: LiDAR counts points; 4D radar can weight each point by its return power.
- :world_map: **From one run to many**: loop candidates for your SLAM, and [multi-session SLAM](multi_session_slam/GUIDE.md) that aligns separate runs.

<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="fig/features_dark.svg" />
    <img src="fig/features_light.svg" alt="FOV-aware bins, two signatures (range for retrieval, azimuth for yaw), milliseconds on a CPU, LiDAR or 4D radar" width="95%" />
  </picture>
</p>

## :floppy_disk: Datasets

KITTI 00 as ROS bags (`/cloud`, `/odom` with ground-truth poses), ready for every example below, is on **[Dropbox](https://www.dropbox.com/scl/fo/qgsj4cj03vziul4e0xnbr/AD1KSBVdeMMk6rUpFaxD8kE?rlkey=7ecyaowj1lhtjfatnei4pd3ye&dl=0)**: `kitti00.bag` (ROS 1, 730 MB), `kitti00_ros2/` (ROS 2, 735 MB) and `kitti00_loop.bag` (a 112 MB cut that returns to the start). They are derived from the [KITTI odometry benchmark](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) (CC BY-NC-SA 3.0).

______________________________________________________________________

## :package: Installation

```bash
git clone https://github.com/sparolab/SOLiD.git
cd SOLiD
```

Everything is built on one C++ library, [`cpp/`](cpp): the ROS 1 node, the ROS 2 node and the macOS tool all run the same code.

### Docker

```bash
xhost +local:docker

# ROS 1
docker build -f docker/Dockerfile.noetic -t solid:noetic .
docker run -it --rm --net=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v /path/to/bags:/data solid:noetic

# ROS 2
docker build -f docker/Dockerfile.humble -t solid:humble .
docker run -it --rm --net=host -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix -v /path/to/bags:/data solid:humble
```

### From source

<details open>
  <summary><strong>ROS 1 (Noetic)</strong></summary>

```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt install libeigen3-dev libyaml-cpp-dev libgtsam-dev libgtsam-unstable-dev

# clone into ~/catkin_ws/src
cd ~/catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
```

</details>

<details open>
  <summary><strong>ROS 2 (Humble)</strong></summary>

```bash
sudo apt install libeigen3-dev libyaml-cpp-dev ros-humble-gtsam

# clone into ~/ros2_ws/src
cd ~/ros2_ws && colcon build --base-paths src/SOLiD/slam/ros2 src/SOLiD/multi_session_slam/ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

</details>

<details open>
  <summary><strong>macOS (no ROS)</strong></summary>

```bash
xcode-select --install && brew install cmake eigen
python3 -m venv .venv && source .venv/bin/activate     # or: uv venv && source .venv/bin/activate
pip install . rosbags matplotlib
```

</details>

______________________________________________________________________

## :rocket: Run

```bash
# ROS 1
roslaunch solid_ros solid.launch config:=kitti save_dir:=$HOME/solid_results
rosbag play --clock kitti00.bag

# ROS 2
ros2 launch solid_ros solid.launch.py config:=kitti save_dir:=$HOME/solid_results
ros2 bag play kitti00_ros2 --clock

# macOS
cd slam/mac && python solid_loops.py --bag kitti00.bag --config ../config/kitti.yaml --out $HOME/solid_results
```

Each run is saved as a session, and sessions from different days can be aligned into one map:

```bash
# ROS 1 / ROS 2
roslaunch solid_multi_session align.launch sessions:="$HOME/day1 $HOME/day2" save_dir:=$HOME/aligned
ros2 launch solid_multi_session align.launch.py sessions:="$HOME/day1 $HOME/day2" save_dir:=$HOME/aligned
```

See the guides for all arguments, outputs and odometry integrations (FAST-LIO, KISS-ICP, GeoRIO, GaRLILEO): [Loop detection](slam/GUIDE.md) · [Multi-session SLAM](multi_session_slam/GUIDE.md).

______________________________________________________________________

## :robot: Run with your agent

Open [Claude Code](https://claude.com/claude-code) or [Codex](https://openai.com/codex/) in an empty folder and paste:

```text
Clone https://github.com/sparolab/SOLiD, read its AGENTS.md, and follow it to build SOLiD and run it on KITTI 00 on this machine. Ask me what you need along the way.
```

The agent follows [`AGENTS.md`](AGENTS.md): it builds SOLiD for ROS 1, ROS 2 or macOS, gets the data, runs it, and checks the loops.

______________________________________________________________________

## :gear: Configuration

All parameters are read at runtime from one YAML file per sensor (`kitti`, `ouster64`, `vlp16`, `radar_4d`). See the configuration guides: [Loop detection](slam/config/GUIDE.md) · [Multi-session SLAM](multi_session_slam/GUIDE.md#configuration).

______________________________________________________________________

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

We envision SOLiD as a community-driven project and love to see it grow through contributions from the community. We would love to see your face in the list below, just open a Pull Request!

<a href="https://github.com/sparolab/SOLiD/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=sparolab/SOLiD" />
</a>

## Acknowledgements

SOLiD runs with [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [KISS-ICP](https://github.com/PRBonn/kiss-icp), [GeoRIO](https://github.com/wooseongY/GeoRIO) and [GaRLILEO](https://github.com/ChiyunNoh/GaRLILEO), and builds on [nanoflann](https://github.com/jlblancoc/nanoflann), [GTSAM](https://github.com/borglab/gtsam) and [PCL](https://github.com/PointCloudLibrary/pcl). We thank the authors for open-sourcing their work.

## Special Thanks

We sincerely thank **[Giseop Kim](https://gisbi-kim.github.io/)** for open-sourcing [LT-mapper](https://github.com/gisbi-kim/lt-mapper), whose session format and multi-session design our multi-session SLAM follows.
