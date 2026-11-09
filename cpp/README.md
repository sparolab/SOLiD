# SOLiD library (cpp)

The single C++ source of truth, laid out like a ROS-independent core library: the ROS 1 / ROS 2
packages, the Python package and the macOS tools are thin wrappers around these targets.

| Target | What | Needs |
| --- | --- | --- |
| `solid::core` | descriptor extraction + KD-tree retrieval (`Extractor`, `Database`) | Eigen (PCL optional) |
| `solid::slam` | keyframes, submaps and loop candidates (`slam::LoopDetector`), LT-mapper session writer (`slam::SessionWriter`), YAML loader | header-only; yaml-cpp for the loader |
| `solid::backend` | online loop closing (`backend::LoopCloser`: GICP check + iSAM2), session loading and multi-session alignment (`backend::Aligner`) | PCL (common, filters, registration), GTSAM, yaml-cpp |

```bash
cmake -S . -B build -DSOLID_USE_PCL=OFF -DSOLID_BUILD_BACKEND=ON
cmake --build build -j && sudo cmake --install build
```
```cmake
find_package(solid REQUIRED COMPONENTS slam backend)
target_link_libraries(my_slam solid::backend)
```

ROS-free tools (`examples/`, built with the back-end):

| Tool | |
| --- | --- |
| `solid_close_loops <session> <config.yaml> [out]` | replays a saved session through `LoopCloser`: optimized poses + every checked candidate |
| `solid_align_sessions <config.yaml> <out> <central> <session> ...` | multi-session alignment of saved sessions |

## Layout

```
cpp/
├── include/solid/
│   ├── config.hpp, descriptor.hpp, extractor.hpp, database.hpp   # solid::core
│   ├── slam/      loop_detector.hpp, session_writer.hpp, config_yaml.hpp
│   └── backend/   loop_closer.hpp, aligner.hpp, session.hpp, params_yaml.hpp
├── src/           extractor.cpp, database.cpp, backend/*.cpp
├── examples/      close_loops.cpp, align_sessions.cpp
├── third_party/nanoflann.hpp
└── test/          verify_golden.cpp (needs SOLID_USE_PCL=ON), test_solid_core.cpp
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
