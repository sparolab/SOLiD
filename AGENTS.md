# SOLiD — setup guide for coding agents

You are setting up and running SOLiD (a LiDAR / 4D radar place recognition descriptor with ROS 1, ROS 2 and macOS front-ends) for a user. Take them from a fresh machine to SOLiD's loop candidates on KITTI 00 and a plot of them. Follow the steps in order. Run every command yourself, check its output before moving on, and stop to ask the user only where a step says **Ask**.

## 0. Ask the user first

**Ask**, in a single message:

1. Platform: **ROS 1 (Noetic)**, **ROS 2 (Humble)** or **macOS / Linux without ROS**.
2. For ROS: **Docker** (recommended; any Linux host with Docker) or **from source** (Noetic on Ubuntu 20.04, Humble on Ubuntu 22.04).
3. Where their KITTI odometry sequence 00 is (the `sequences/00` folder with `velodyne/`, `poses.txt`, `calib.txt`). If they do not have it, tell them to download it from the KITTI website (registration required): the velodyne scans (80 GB for all sequences), poses and calibration.
4. Where to save results (default `~/solid_results`).

## 1. Check the machine

- Docker path: `docker --version`. Source path: `ls /opt/ros` must show `noetic` or `humble`.
- macOS: `xcode-select -p` (Command Line Tools), `brew --version`, and a Python 3.10–3.13.
- Free disk space with `df -h`: about 3 GB is enough.

## 2. Get the code

```bash
git clone https://github.com/sparolab/SOLiD.git
cd SOLiD
```

Layout: `cpp/` is the SOLiD library that everything links; `slam/ros1`, `slam/ros2` (package `solid_ros`) and `slam/mac` are the loop detection front-ends; `multi_session_slam/` aligns saved runs; `slam/config/` holds one YAML per sensor.

## 3. Build

### Docker

```bash
docker build -f docker/Dockerfile.noetic -t solid:noetic .   # ROS 1
docker build -f docker/Dockerfile.humble -t solid:humble .   # ROS 2
```

### From source — ROS 1 (Noetic)

```bash
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0
sudo apt install -y libeigen3-dev libyaml-cpp-dev libgtsam-dev libgtsam-unstable-dev
mkdir -p ~/catkin_ws/src && ln -s "$PWD" ~/catkin_ws/src/SOLiD
cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### From source — ROS 2 (Humble)

```bash
sudo apt install -y libeigen3-dev libyaml-cpp-dev ros-humble-gtsam
mkdir -p ~/ros2_ws/src && ln -s "$PWD" ~/ros2_ws/src/SOLiD
cd ~/ros2_ws && source /opt/ros/humble/setup.bash
colcon build --base-paths src/SOLiD/slam/ros2 src/SOLiD/multi_session_slam/ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`--base-paths` is required. Without it, colcon also finds the ROS 1 packages.

### macOS / Linux without ROS

```bash
xcode-select --install; brew install cmake eigen          # macOS only
python3 -m venv .venv && source .venv/bin/activate
pip install . rosbags matplotlib
python -c "import solid; print(solid.backend)"            # must print: cpp
python python/tests/test_binding.py                       # must end with: PASSED
```

## 4. Prepare the data

The macOS tool reads the KITTI folder directly. For ROS, convert it to a bag (`/cloud` in the sensor frame, `/odom` with the ground-truth poses). The converter only needs `rosbags`, so run it with the Python from step 3 (or `pip install rosbags numpy` on the host):

```bash
cd slam/mac
python kitti_to_bag.py /path/to/sequences/00 ~/kitti00.bag --voxel 0.4     # ROS 1, about 0.75 GB
python kitti_to_bag.py /path/to/sequences/00 ~/kitti00_ros2 --voxel 0.4    # ROS 2
```

## 5. Run

```bash
# ROS 1 (two terminals, or background the launch)
roslaunch solid_ros solid.launch config:=kitti save_dir:=$HOME/solid_results
rosbag play --clock kitti00.bag

# ROS 2
ros2 launch solid_ros solid.launch.py config:=kitti save_dir:=$HOME/solid_results
ros2 bag play kitti00_ros2 --clock

# macOS / Linux without ROS
cd slam/mac
python solid_loops.py --kitti /path/to/sequences/00 --config ../config/kitti.yaml --out $HOME/solid_results --gt-radius 5
```

When the bag has finished, stop the launch with **Ctrl-C**: the node saves on exit.

## 6. Check the result

- `$HOME/solid_results/loops.csv` and `keyframes.txt` must exist (plus `Scans/`, `SCDs/`, `SOLiDs/`, `singlesession_posegraph.g2o`).
- Expected on full KITTI 00: about **2712 keyframes and about 416 loop candidates**, all within 5 m of each other (the macOS tool prints `precision 1.000` with `--gt-radius 5`). Small differences between the KITTI folder, the ROS 1 and the ROS 2 runs come from the bag's voxel filter and message timing.
- macOS: open `loops.png`. The red loop markers sit on the stretches KITTI 00 drives twice.
- Report the numbers and the plot to the user.

## 7. Optional: multi-session SLAM

Cut KITTI 00 into two sessions and align them (ROS 1 or ROS 2 build):

```bash
cd slam/mac
python solid_loops.py --kitti /path/to/sequences/00 --config ../config/kitti.yaml --out sessions/00a --range 0:2000 --rebase
python solid_loops.py --kitti /path/to/sequences/00 --config ../config/kitti.yaml --out sessions/00b --range 2000:4541 --rebase
roslaunch solid_multi_session align.launch sessions:="$PWD/sessions/00a $PWD/sessions/00b" save_dir:=$PWD/aligned
```

The node prints, for 00b, the SOLiD candidates, the verified loops and `ALIGNED`. See [multi_session_slam/GUIDE.md](multi_session_slam/GUIDE.md).

## Troubleshooting

- **No loops at all**: the cloud is probably not in a z-up sensor frame (e.g. an IMU or camera optical frame), or the FOV in the config does not match the sensor. See [slam/config/GUIDE.md](slam/config/GUIDE.md#pitfalls).
- **ROS 2 node saves nothing**: it saves on shutdown; stop it with Ctrl-C (SIGINT), not `kill -9`.
- **macOS build fails**: check `brew list cmake eigen` and that the Python is not the Apple system Python 3.9.
