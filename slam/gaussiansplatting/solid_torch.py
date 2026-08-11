"""SOLiD rsolid extraction in pure PyTorch (CPU or GPU).

A faithful, fully-vectorized port of the SOLiD range descriptor so the *same*
geometric algorithm can run on a GPU tensor — for an apples-to-apples compute
comparison (FLOPs / latency, CPU-vs-CPU and GPU-vs-GPU) against the learned
visual descriptors (NetVLAD / SALAD / DINOv2), which are torch modules.

    from solid_torch import SolidTorch
    ext = SolidTorch(solid.load_config("gs_rgbd"), device="cuda")
    rsolid = ext(points_tensor)          # (num_range,) L2-comparable descriptor

Matches `solid` (C++/numpy) rsolid up to float precision (voxel included).
"""
import math

import numpy as np
import torch


class SolidTorch:
    def __init__(self, cfg, device="cpu"):
        self.device = device
        self.num_angle = int(cfg.num_angle)
        self.num_range = int(cfg.num_range)
        self.num_height = int(cfg.num_height)
        self.min_range = float(cfg.min_range)
        self.max_range = float(cfg.max_range)
        self.fov_up = float(cfg.fov_up)
        self.fov_down = float(cfg.fov_down)
        self.voxel_size = float(cfg.voxel_size)

    @torch.no_grad()
    def _voxel(self, pts):
        # 1-D hash of the 3-D voxel index, then unique on the hash (much faster
        # than torch.unique(dim=0), which dominates the torch runtime otherwise).
        leaf = self.voxel_size
        k = torch.floor(pts / leaf).long()
        k = k - k.min(0).values                      # non-negative
        M = k.max(0).values + 1
        h = (k[:, 0] * M[1] + k[:, 1]) * M[2] + k[:, 2]
        uniq, inv = torch.unique(h, return_inverse=True)
        n = uniq.shape[0]
        sums = torch.zeros(n, 3, device=pts.device, dtype=pts.dtype).index_add_(0, inv, pts)
        cnt = torch.zeros(n, device=pts.device, dtype=pts.dtype).index_add_(
            0, inv, torch.ones(len(pts), device=pts.device, dtype=pts.dtype))
        return sums / cnt[:, None]

    @torch.no_grad()
    def __call__(self, points):
        if not torch.is_tensor(points):
            points = torch.as_tensor(points)
        pts = points.to(self.device, torch.float32)

        # range gate
        dist = pts.pow(2).sum(1).sqrt()
        pts = pts[(dist > self.min_range) & (dist < self.max_range)]
        if self.voxel_size > 0 and len(pts):
            pts = self._voxel(pts)
        if len(pts) == 0:
            return torch.zeros(self.num_range, device=self.device)

        x = pts[:, 0].clone(); y = pts[:, 1].clone(); z = pts[:, 2]
        x[x == 0] = 1e-3; y[y == 0] = 1e-3

        gap_a = 360.0 / self.num_angle
        gap_r = self.max_range / self.num_range
        gap_h = (self.fov_up - self.fov_down) / self.num_height

        theta = torch.rad2deg(torch.atan2(y, x)) % 360.0           # azimuth [0,360)
        dist_xy = torch.sqrt(x * x + y * y)
        phi = torch.rad2deg(torch.atan2(z, dist_xy))               # elevation

        idx_r = (dist_xy / gap_r).long().clamp_(0, self.num_range - 1)
        idx_a = (theta / gap_a).long().clamp_(0, self.num_angle - 1)
        idx_h = ((phi - self.fov_down) / gap_h).long().clamp_(0, self.num_height - 1)

        ones = torch.ones(len(pts), device=self.device)
        R = torch.zeros(self.num_range * self.num_height, device=self.device)
        R.index_add_(0, idx_r * self.num_height + idx_h, ones)
        range_matrix = R.view(self.num_range, self.num_height)

        number_vector = range_matrix.sum(0)
        mn, mx = number_vector.min(), number_vector.max()
        number_vector = (number_vector - mn) / (mx - mn) if (mx - mn) > 1e-12 \
            else torch.zeros_like(number_vector)
        return range_matrix @ number_vector                        # (num_range,)


if __name__ == "__main__":
    # correctness check vs the numpy/C++ backend
    import solid
    cfg = solid.load_config("gs_rgbd")
    rng = np.random.default_rng(0)
    pc = (rng.standard_normal((4000, 3)) * np.array([2, 2, 5])).astype(np.float32)
    ref = np.asarray(solid.Extractor(cfg).extract(pc).rsolid, np.float32)
    for dev in (["cpu", "cuda"] if torch.cuda.is_available() else ["cpu"]):
        got = SolidTorch(cfg, dev)(pc).cpu().numpy()
        cos = float(ref @ got / (np.linalg.norm(ref) * np.linalg.norm(got) + 1e-9))
        print(f"[{dev}] cos(numpy, torch) = {cos:.4f}  dim={got.shape[0]}")
