# SOLiD core (cpp)

Modality-neutral **SOLiD** descriptor library — pure C++ (Eigen + PCL), no ROS.
This is the single C++ source of truth: the `ros/`, `ros2/` and `python/`
front-ends and every `slam/<type>/` integration consume it.

Scope: **place recognition** — descriptor extraction + KD-tree retrieval
(loop candidate + yaw + score). Registration and pose-graph optimization are
done by the downstream SLAM (see repo-root README).

## Layout

```
cpp/
├── include/solid/
│   ├── config.hpp      # Config: every param runtime-configurable (LiDAR/radar/GS profiles)
│   ├── descriptor.hpp  # Descriptor {rsolid, asolid}, Candidate {id, score, yaw_rad}
│   ├── extractor.hpp   # Extractor: points(+weight) -> Descriptor  (+ loopSimilarity / poseYawDeg)
│   └── database.hpp    # Database: KD-tree retrieval over rsolid, re-scored by cosine + yaw
├── src/{extractor,database}.cpp
├── third_party/nanoflann.hpp   # vendored (v1.3.0)
└── test/
    ├── verify_golden.cpp       # regression vs frozen golden descriptors (no gtest needed)
    ├── test_solid_core.cpp     # gtest unit tests (opt-in)
    └── data/{golden/*, pcd/*}  # frozen ground-truth + sample KITTI scans
```

## Design notes

- **Runtime config, no hardcoding.** `Config` defaults reproduce the original
  SOLiD constants exactly. Radar/GS profiles just change fields
  (`num_height`, `voxel_size`, `use_weight`, `max_range`, FOV…).
- **Modality-neutral input.** `extract()` takes `std::vector<Eigen::Vector3f>`
  (+ optional per-point weight for radar RCS/power), so LiDAR / 4D-radar /
  GS-depth clouds share one code path.
- **rsolid** (range signature) is the KD-tree key; **asolid** (angular) recovers yaw.

## Build & verify (per-step gate)

The build ignores an active Anaconda/Conda prefix (its older libstdc++ otherwise
breaks running against system PCL). If you still hit a `GLIBCXX` runtime error,
run with a clean env: `env -u LD_LIBRARY_PATH ./build/verify_golden`.

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/verify_golden      # expect: ==== PASSED (0 failures) ====
```

The golden files in `test/data/golden/` were snapshotted from the original
SOLiDModule reference (the algorithm's ground-truth output on scans 313/314/315).
solid_core must match them to `1e-9`.

Optional gtest suite (needs a system GTest, not the Conda one):
```bash
cmake -B build -S . -DBUILD_SOLID_TESTS=ON && cmake --build build -j && ctest --test-dir build
```
