"""Pure-Python SOLiD core.

Mirrors the C++/binding API *exactly* (same class and method names):
``Config``, ``Descriptor``, ``Candidate``, ``Extractor``, ``Database`` with
``extract`` / ``loop_similarity`` / ``pose_yaw_deg`` / ``add`` / ``query``.

This is the dependency-free fallback (NumPy only). For the same input and Config
it reproduces the C++ core; see tests/test_binding.py which cross-checks them.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class Config:
    fov_up: float = 2.0
    fov_down: float = -24.8
    num_angle: int = 60
    num_range: int = 40
    num_height: int = 32
    min_range: float = 3.0
    max_range: float = 80.0
    voxel_size: float = 0.4
    use_weight: bool = False
    knn: int = 30
    min_similarity: float = 0.0


@dataclass
class Descriptor:
    rsolid: np.ndarray = field(default_factory=lambda: np.zeros(0))
    asolid: np.ndarray = field(default_factory=lambda: np.zeros(0))

    def empty(self) -> bool:
        return self.rsolid.size == 0 and self.asolid.size == 0

    def combined(self) -> np.ndarray:
        return np.concatenate([self.rsolid, self.asolid])


@dataclass
class Candidate:
    id: int
    score: float
    yaw_rad: float


def _xy2theta(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    k = 180.0 / np.pi
    theta = np.empty_like(x)
    m1 = (x >= 0) & (y >= 0)
    m2 = (x < 0) & (y > 0)
    m3 = (x < 0) & (y < 0)
    m4 = (x >= 0) & (y < 0)
    theta[m1] = k * np.arctan(y[m1] / x[m1])
    theta[m2] = 180.0 - k * np.arctan(y[m2] / (-x[m2]))
    theta[m3] = 180.0 + k * np.arctan(y[m3] / x[m3])
    theta[m4] = 360.0 - k * np.arctan((-y[m4]) / x[m4])
    return theta


class Extractor:
    def __init__(self, config: Config):
        self.cfg = config

    def config(self) -> Config:
        return self.cfg

    def extract(self, points: np.ndarray,
                weights: Optional[np.ndarray] = None) -> Descriptor:
        pts = np.asarray(points, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError("points must be (N, 3)")
        w = None if weights is None else np.asarray(weights, dtype=np.float64)
        pts, w = self._preprocess(pts, w)
        return self._make_solid(pts, w)

    # --- preprocessing: range gate + optional voxel (mirrors C++) ---
    def _preprocess(self, pts, w):
        c = self.cfg
        has_w = bool(c.use_weight) and w is not None and len(w) == len(pts)
        dist = np.sqrt((pts ** 2).sum(axis=1))
        keep = (dist > c.min_range) & (dist < c.max_range)
        pts = pts[keep]
        if has_w:
            w = w[keep]
        else:
            w = None
        if c.voxel_size > 0.0 and not has_w:
            pts = self._voxel(pts, c.voxel_size)
        return pts, w

    @staticmethod
    def _voxel(pts, leaf):
        if len(pts) == 0:
            return pts
        keys = np.floor(pts / leaf).astype(np.int64)
        _, inv = np.unique(keys, axis=0, return_inverse=True)
        inv = inv.ravel()
        n = inv.max() + 1
        sums = np.zeros((n, 3))
        cnt = np.zeros(n)
        np.add.at(sums, inv, pts)
        np.add.at(cnt, inv, 1.0)
        return sums / cnt[:, None]

    def _make_solid(self, pts, w) -> Descriptor:
        c = self.cfg
        if len(pts) == 0:
            return Descriptor(np.zeros(c.num_range), np.zeros(c.num_angle))
        x = pts[:, 0].copy()
        y = pts[:, 1].copy()
        z = pts[:, 2]
        x[x == 0.0] = 0.001
        y[y == 0.0] = 0.001

        gap_angle = 360.0 / c.num_angle
        gap_range = c.max_range / c.num_range
        gap_height = (c.fov_up - c.fov_down) / c.num_height

        theta = _xy2theta(x, y)
        dist_xy = np.sqrt(x * x + y * y)
        phi = np.rad2deg(np.arctan2(z, dist_xy))

        # Clamp to valid bins ([0, n-1]); the lower bound only matters for points
        # outside the vertical FOV (dense camera/foundation clouds). Matches the
        # C++ core; in-FOV LiDAR is unaffected.
        idx_r = np.clip((dist_xy / gap_range).astype(np.int64), 0, c.num_range - 1)
        idx_a = np.clip((theta / gap_angle).astype(np.int64), 0, c.num_angle - 1)
        idx_h = np.clip(((phi - c.fov_down) / gap_height).astype(np.int64),
                        0, c.num_height - 1)

        contrib = w if (w is not None) else np.ones(len(pts))
        range_matrix = np.zeros((c.num_range, c.num_height))
        angle_matrix = np.zeros((c.num_angle, c.num_height))
        np.add.at(range_matrix, (idx_r, idx_h), contrib)
        np.add.at(angle_matrix, (idx_a, idx_h), contrib)

        number_vector = range_matrix.sum(axis=0)
        mn = number_vector.min()
        mx = number_vector.max()
        denom = mx - mn
        if denom > 1e-12:
            number_vector = (number_vector - mn) / denom
        else:
            number_vector = np.zeros_like(number_vector)

        return Descriptor(rsolid=range_matrix @ number_vector,
                          asolid=angle_matrix @ number_vector)

    # --- matching helpers (same names as C++/binding) ---
    @staticmethod
    def loop_similarity(rsolid_query: np.ndarray,
                        rsolid_candidate: np.ndarray) -> float:
        n = np.linalg.norm(rsolid_query) * np.linalg.norm(rsolid_candidate)
        if n <= 0.0:
            return 0.0
        return float(np.dot(rsolid_query, rsolid_candidate) / n)

    @staticmethod
    def pose_yaw_deg(asolid_query: np.ndarray,
                     asolid_candidate: np.ndarray) -> float:
        n = len(asolid_query)
        best, best_shift = np.inf, 0
        for shift in range(n):
            shifted = np.roll(asolid_query, shift)
            l1 = np.abs(asolid_candidate - shifted).sum()
            if l1 < best:
                best, best_shift = l1, shift
        return (best_shift + 1) * (360.0 / n)


class Database:
    def __init__(self, config: Config):
        self.cfg = config
        self._ids: List[int] = []
        self._r: List[np.ndarray] = []
        self._a: List[np.ndarray] = []

    def add(self, id: int, descriptor: Descriptor) -> None:
        self._ids.append(int(id))
        self._r.append(descriptor.rsolid)
        self._a.append(descriptor.asolid)

    def size(self) -> int:
        return len(self._ids)

    def empty(self) -> bool:
        return not self._ids

    def query(self, descriptor: Descriptor) -> List[Candidate]:
        if not self._ids or descriptor.rsolid.size == 0:
            return []
        out = []
        for i, rid in enumerate(self._ids):
            s = Extractor.loop_similarity(descriptor.rsolid, self._r[i])
            if s >= self.cfg.min_similarity:
                yaw = np.deg2rad(Extractor.pose_yaw_deg(descriptor.asolid,
                                                        self._a[i]))
                out.append(Candidate(id=rid, score=s, yaw_rad=float(yaw)))
        out.sort(key=lambda c: c.score, reverse=True)
        return out[: self.cfg.knn]
