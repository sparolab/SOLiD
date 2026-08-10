"""Verify the C++ binding against the independent pure-Python implementation.

The two are separate implementations of the same algorithm, so agreement is a
strong correctness check. Runnable directly (no pytest needed):

    python tests/test_binding.py
"""
import os
import struct
import sys

import numpy as np

import solid
from solid.pure import Config as PyConfig
from solid.pure import Database as PyDatabase
from solid.pure import Extractor as PyExtractor

DATA = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "cpp", "test", "data"))

_fail = 0


def check(cond, msg):
    global _fail
    print(f"  [{'ok  ' if cond else 'FAIL'}] {msg}")
    if not cond:
        _fail += 1


def load_pcd(path):
    """Minimal binary-PCD reader (FIELDS x y z / TYPE F / DATA binary)."""
    with open(path, "rb") as f:
        n = 0
        while True:
            line = f.readline().decode("ascii", "replace")
            key = line.split()[0] if line.split() else ""
            if key == "POINTS":
                n = int(line.split()[1])
            elif key == "DATA":
                break
        buf = f.read(n * 12)
    return np.frombuffer(buf, dtype="<f4").reshape(n, 3).astype(np.float64)


def main():
    print(f"SOLiD python binding vs pure-python\n  backend={solid.backend}\n  data={DATA}\n")
    check(solid.backend == "cpp", "compiled C++ backend is active")

    p313 = load_pcd(os.path.join(DATA, "pcd", "000313.pcd"))
    p314 = load_pcd(os.path.join(DATA, "pcd", "000314.pcd"))
    p315 = load_pcd(os.path.join(DATA, "pcd", "000315.pcd"))

    # [1] Agreement with voxel OFF. The C++ core computes geometry in float32
    # (Eigen::Vector3f) while pure-Python uses float64, so a point sitting exactly
    # on a bin boundary can round to a neighbouring bin — a discretization effect,
    # not a logic difference. We therefore require the descriptors to be
    # essentially identical (cosine >= 1 - 1e-6) with at most a couple of bins
    # differing by more than a count.
    def sim(a, b):
        a = np.asarray(a); b = np.asarray(b)
        return float(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)))

    print("\n[1] binding vs pure, voxel_size=0 (cosine >= 1-1e-6)")
    cfg_c = solid.Config(); cfg_c.voxel_size = 0.0
    cfg_p = PyConfig(voxel_size=0.0)
    ec, ep = solid.Extractor(cfg_c), PyExtractor(cfg_p)
    for name, pts in (("313", p313), ("314", p314), ("315", p315)):
        dc = ec.extract(pts.astype(np.float32))
        dp = ep.extract(pts)
        sr, sa = sim(dc.rsolid, dp.rsolid), sim(dc.asolid, dp.asolid)
        nd = int(np.sum(np.abs(np.asarray(dc.rsolid) - dp.rsolid) > 1e-6) +
                 np.sum(np.abs(np.asarray(dc.asolid) - dp.asolid) > 1e-6))
        print(f"    scan {name}: cos(rsolid)={sr:.9f} cos(asolid)={sa:.9f} bins_differing={nd}")
        check(sr > 1 - 1e-6, f"rsolid ~ pure (scan {name})")
        check(sa > 1 - 1e-6, f"asolid ~ pure (scan {name})")
        check(nd <= 2, f"<=2 boundary-flip bins (scan {name})")

    # [2] loop_similarity / pose_yaw_deg identical across backends.
    print("\n[2] loop_similarity / pose_yaw_deg")
    dc3 = ec.extract(p313.astype(np.float32)); dc4 = ec.extract(p314.astype(np.float32))
    dp3 = ep.extract(p313); dp4 = ep.extract(p314)
    sc = solid.Extractor.loop_similarity(dc3.rsolid, dc4.rsolid)
    sp = PyExtractor.loop_similarity(dp3.rsolid, dp4.rsolid)
    yc = solid.Extractor.pose_yaw_deg(dc3.asolid, dc4.asolid)
    yp = PyExtractor.pose_yaw_deg(dp3.asolid, dp4.asolid)
    print(f"    similarity: cpp={sc:.9f} py={sp:.9f} | yaw(deg): cpp={yc:.6f} py={yp:.6f}")
    check(abs(sc - sp) < 1e-9, "loop_similarity matches")
    check(abs(yc - yp) < 1e-9, "pose_yaw_deg matches")

    # [3] Database retrieval agrees (default config).
    print("\n[3] Database retrieval (default config)")
    dbc = solid.Database(solid.Config())
    dbp = PyDatabase(PyConfig())
    for rid, pts in ((314, p314), (315, p315)):
        dbc.add(rid, solid.Extractor(solid.Config()).extract(pts.astype(np.float32)))
        dbp.add(rid, PyExtractor(PyConfig()).extract(pts))
    qc = solid.Extractor(solid.Config()).extract(p313.astype(np.float32))
    qp = PyExtractor(PyConfig()).extract(p313)
    hc = dbc.query(qc)
    hp = dbp.query(qp)
    check(len(hc) == 2 and len(hp) == 2, "both backends retrieve 2 candidates")
    check(hc[0].id == hp[0].id == 314, "both rank scan 314 first")

    print(f"\n==== {'FAILED' if _fail else 'PASSED'} ({_fail} failure{'' if _fail==1 else 's'}) ====")
    return 1 if _fail else 0


if __name__ == "__main__":
    sys.exit(main())
