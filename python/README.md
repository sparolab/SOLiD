# solid-descriptor (Python)

pip-installable **SOLiD** global descriptor with two interchangeable backends
sharing one API:

- **C++ core binding** (fast, via pybind11 of `../cpp`) — top-level `import solid`
- **pure-Python** (NumPy only) — `import solid.pure`

Both expose the same names as the C++ core: `Config`, `Descriptor`, `Candidate`,
`Extractor` (`extract`, `loop_similarity`, `pose_yaw_deg`), `Database`
(`add`, `query`).

## Install

The compiled extension pulls the C++ core from the sibling `../cpp`, so install
from the repository tree (not from a copied/isolated build):

```bash
# from the SOLiD repo root
pip install -e python                    # editable (recommended)
# or
pip install --no-build-isolation ./python
```

The extension is built **PCL-free** (internal voxel) and statically links the
C++ runtime, so it imports under any Python — including Conda envs such as the
GS-ICP-SLAM environment.

## Usage

```python
import numpy as np
import solid                      # C++ backend if built, else pure-Python
print(solid.backend)             # "cpp" or "python"

cfg = solid.Config()             # LiDAR defaults; tweak for radar / GS profiles
ext = solid.Extractor(cfg)
desc = ext.extract(points)       # points: (N,3) float array  (+ optional weights)

db = solid.Database(cfg)
db.add(0, desc)
hits = db.query(other_desc)      # -> [Candidate(id, score, yaw_rad), ...]
```

To force the pure-Python backend: `from solid.pure import Extractor, Database, Config`.

## Verify

```bash
python tests/test_binding.py     # cross-checks C++ binding vs pure-Python
```
