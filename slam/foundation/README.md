<div align="center">
  <h1>SOLiD x VGGT-SLAM</h1>
  <a href="https://arxiv.org/abs/2408.07330"><img src="https://img.shields.io/badge/arXiv-2408.07330-b31b1b?logo=arxiv&logoColor=white" alt="arXiv" /></a>
  <a href="https://sparolab.github.io/research/solid/"><img src="https://img.shields.io/badge/Project-Commerge-6f42c1" alt="SOLiDxVGGT-SLAM" /></a>
  <a href=""><img src="https://img.shields.io/badge/YouTube-Video-FF0000?logo=youtube&logoColor=white" alt="YouTube" /></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-BSD--3--Clause-green" alt="BSD 3-Clause License" /></a>
  <br />
  <br />
  <p align="center">
  <img src="https://readme-typing-svg.demolab.com?background=0D1117&color=22C55E&font=Fira+Code&size=14&duration=1500&pause=1000&center=true&vCenter=true&width=200&height=24&lines=SOLiD+meets+VGGT+SLAM%21" alt="tagline"/>
  </p>
</div>

<div align="center">
  <img src="assets/vggt_solid_slam.gif" width="720"/>
</div>

## :package: Installation

> [!NOTE]
> SOLiD runs on the CPU, but VGGT-SLAM requires an NVIDIA GPU and CUDA-enabled
> PyTorch. The integration has been tested with Python 3.11.

### 1. Clone SOLiD with VGGT-SLAM

VGGT-SLAM is included as a Git submodule pinned to the tested SOLiD integration:

```bash
git clone --recurse-submodules --branch test https://github.com/sparolab/SOLiD.git
cd SOLiD
```

If SOLiD was cloned without `--recurse-submodules`, initialize VGGT-SLAM with:

```bash
git submodule update --init --recursive
```

### 2. Install VGGT-SLAM

Create its conda environment and run the upstream setup script:

```bash
cd slam/foundation/VGGT-SLAM

conda create -n vggt-slam python=3.11 -y
conda activate vggt-slam

chmod +x setup.sh
./setup.sh
```

The VGGT-SLAM setup script installs its model dependencies and downloads the
required third-party packages. See the upstream
[installation guide](https://github.com/MIT-SPARK/VGGT-SLAM#installation-of-vggt-slam)
for platform-specific requirements.

### 3. Install SOLiD

Install SOLiD from the parent repository into the same conda environment:

```bash
pip install -e ../../..
```

The SOLiD package builds its lightweight C++ core and installs the Python module
as `solid`. No learned weights or additional GPU dependencies are required.

The VGGT-SLAM submodule already contains the integration: no patching or manual
file copying is required. It tracks the `solid-foundation` branch of
[`hogyun2/VGGT-SLAM`](https://github.com/hogyun2/VGGT-SLAM/tree/solid-foundation)
and is pinned to a tested commit for reproducibility.

### 4. Verify the installation

```bash
python -c "import solid; print('SOLiD import: OK')"
python main.py --help | grep retrieval
```

The first command should print `SOLiD import: OK`; the second should list the
`salad`, `netvlad`, and `solid` retrieval backends.

## Result

On the bundled `office_loop` sequence (473 frames), the validated configuration
produces exactly one loop at the intended final revisit:

| Result | Value |
|---|---:|
| accepted loop | submap `204 -> 0` |
| SOLiD descriptor distance | `0.457` |
| odometry radius | `0.4` |
| VGGT image match ratio | `0.998` |
| false loops before final revisit | `0` |
| descriptor input | XYZ geometry only |

The radius search is an odometry-space candidate gate, not an RGB/intensity
feature. `SOLID_MIN_SUBMAP_GAP=50` prevents adjacent-submap matches.

## Files

| File | Purpose |
|---|---|
| `VGGT-SLAM/` | tested VGGT-SLAM integration, pinned as a Git submodule |
| `VGGT-SLAM/vggt_slam/pluggable_retrieval.py` | SALAD/NetVLAD/SOLiD retrieval backends and radius gate |
| `VGGT-SLAM/vggt_slam/o3d_loop_vis.py` | capture loop-before/loop-after maps and camera wireframes |
| `HANDOFF_solid_loop_closure.md` | experiments, reasoning, and detailed handoff |

## Reproduce the validated office loop

```bash
VGGT_SEED=0 \
SOLID_NO_FLOOR=1 SOLID_CAMERA_Z_UP=1 SOLID_INTENSITY=0 SOLID_AZIMUTH=0 \
SOLID_MAX_RANGE=2.0 SOLID_NUM_RANGE=10 SOLID_NUM_HEIGHT=12 \
SOLID_FOV=30 SOLID_MIN_RANGE=0.05 SOLID_VOXEL=0.04 SOLID_SUB=6 \
SOLID_MEAN_REMOVE=0 SOLID_MIN_SUBMAP_GAP=50 SOLID_ODOM_RADIUS=0.4 \
SOLID_ICP_MIN=0 VGGT_IMG_THRES=0 \
python -u main.py --image_folder office_loop --submap_size 16 \
  --max_loops 1 --retrieval solid --lc_thres 0.55
```

To capture loop-before/after snapshots, append:

```bash
--o3d_lc_vis --o3d_lc_no_window --o3d_lc_dir o3d_loop_vis --o3d_lc_voxel 0.03
```

The visualization above uses VGGT-SLAM's common dense reconstruction, not an
externally measured ground-truth map.
