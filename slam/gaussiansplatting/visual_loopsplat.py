"""Visual place-recognition descriptors (DINOv2 / SALAD) for LoopSplat.

Same drop-in idea as solid_loopsplat.py: LoopSplat detects loops by cosine
similarity of per-keyframe submap descriptors (`update_submaps_info` builds them,
`detect_closure` matches). The base uses NetVLAD; here we swap in DINOv2 or
DINOv2-SALAD so all four detectors (netvlad/solid/dinov2/salad) run in the SAME
LoopSplat GS-tracking + 3DGS-registration + PGO pipeline, isolating detector
quality on identical odometry/closure machinery.

Only `update_submaps_info` changes (image -> L2-normalized global descriptor);
detect_closure / gs_reg / PGO / Gaussian deformation are inherited unchanged.

    lc = VisualLoopClosure(config, dataset, logger, kind="dinov2")
"""
import numpy as np
import torch

from src.entities.lc import Loop_closure

_IMAGENET_MEAN = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
_IMAGENET_STD = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)


class VisualLoopClosure(Loop_closure):
    def __init__(self, config, dataset, logger, kind="dinov2"):
        super().__init__(config, dataset, logger)
        self.kind = kind
        self._mean = _IMAGENET_MEAN.to(self.device)
        self._std = _IMAGENET_STD.to(self.device)
        if kind == "dinov2":
            self.model = torch.hub.load("facebookresearch/dinov2",
                                        "dinov2_vits14").to(self.device).eval()
            self.size = 224                       # 16*14
        elif kind == "salad":
            self.model = torch.hub.load("serizba/salad", "dinov2_salad",
                                        trust_repo=True).to(self.device).eval()
            self.size = 322                       # 23*14
        else:
            raise ValueError(f"unknown visual detector {kind}")
        print(f"[VisualLoopClosure] detector={kind} input={self.size}")

    def _describe(self, rgb_hw3):
        # rgb_hw3: HxWx3 uint8 (as returned by dataset[key][1])
        t = torch.from_numpy(np.ascontiguousarray(rgb_hw3)).float().to(self.device)
        t = t.permute(2, 0, 1)[None] / 255.0
        t = torch.nn.functional.interpolate(t, size=(self.size, self.size),
                                            mode="bilinear", align_corners=False)
        t = (t - self._mean) / self._std
        f = self.model(t).squeeze().float()
        f = f / (f.norm() + 1e-12)                # L2 -> einsum == cosine
        return f[None]

    def update_submaps_info(self, keyframes_info):
        with torch.no_grad():
            kf_ids = sorted(list(keyframes_info.keys()))
            descs = [self._describe(self.dataset[k][1]) for k in kf_ids]
            submap_desc = torch.cat(descs)
            self_sim = torch.einsum("id,jd->ij", submap_desc, submap_desc)
            k = max(int(len(submap_desc) * self.config["lc"]["min_similarity"]), 1)
            score_min, _ = self_sim.topk(k)
        self.submap_lc_info[self.submap_id] = {
            "submap_id": self.submap_id,
            "kf_id": np.array(kf_ids),
            "kf_desc": submap_desc,
            "self_sim": score_min,
        }
