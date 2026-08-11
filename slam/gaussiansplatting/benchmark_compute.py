"""Fair compute comparison: SOLiD vs learned visual place-recognition descriptors.

SOLiD's speed is usually quoted CPU-vs-GPU (apples-to-oranges). Here we compare
on equal footing:
  - FLOPs per descriptor (device-independent) via fvcore
  - latency on CPU (CPU-vs-CPU) and GPU (GPU-vs-GPU)

SOLiD runs as the torch port (`solid_torch.SolidTorch`) on CPU and GPU too, so
every descriptor is measured in PyTorch (same framework overhead). Any descriptor
that fails to load is skipped. Produces `compute_comparison.png`.
(SOLiD's optimized C++ CPU backend is even faster ~0.9ms, but we compare
torch-to-torch here for fairness.)

  python benchmark_compute.py [--points 12000 --iters 30]
"""
import argparse
import sys
import time

import numpy as np
import torch

sys.path.insert(0, "/home/hogyun2/Test/Etc/research/SKiD-SLAM2/reference/LoopSplat")


def timeit(fn, iters, device):
    for _ in range(5):
        fn()
    if device == "cuda":
        torch.cuda.synchronize()
    ts = []
    for _ in range(iters):
        if device == "cuda":
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        fn()
        if device == "cuda":
            torch.cuda.synchronize()
        ts.append((time.perf_counter() - t0) * 1e3)
    return float(np.median(ts))


def flops_fvcore(model, inputs):
    from fvcore.nn import FlopCountAnalysis
    import logging
    logging.getLogger("fvcore").setLevel(logging.ERROR)
    fa = FlopCountAnalysis(model, inputs)
    fa.unsupported_ops_warnings(False); fa.uncalled_modules_warnings(False)
    return 2 * fa.total()                       # FLOPs = 2 * MACs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--points", type=int, default=12000)
    ap.add_argument("--iters", type=int, default=30)
    args = ap.parse_args()
    devices = ["cpu"] + (["cuda"] if torch.cuda.is_available() else [])
    rows = []   # (name, flops, {device: ms}, color)

    # ---------------- SOLiD ----------------
    try:
        import solid
        from solid_torch import SolidTorch
        cfg = solid.load_config("gs_rgbd")
        rng = np.random.default_rng(0)
        pc = (rng.standard_normal((args.points, 3)) * np.array([2, 2, 5])).astype(np.float32)
        lat = {}
        # torch on every device (fair python-vs-python; C++ is faster but the
        # visual baselines are all torch, so compare torch-to-torch)
        for d in devices:
            tt = torch.from_numpy(pc).to(d); text = SolidTorch(cfg, d)
            lat[d] = timeit(lambda text=text, tt=tt: text(tt), args.iters, d)
        flops = args.points * 40 + cfg.num_range * cfg.num_height * 2   # analytical
        rows.append(("SOLiD", flops, lat, "#22c55e"))
        print(f"SOLiD: {flops/1e6:.2f} MFLOPs | " + " | ".join(f"{d} {lat[d]:.3f}ms" for d in lat))
    except Exception as e:
        print("SOLiD skipped:", repr(e))

    # ---------------- learned visual descriptors ----------------
    def add_net(name, model, make_input, color):
        try:
            model = model.eval()
            x = make_input()
            fl = None
            try:
                fl = flops_fvcore(model.to("cpu"), (x.to("cpu"),))
            except Exception as e:
                print(f"  {name} FLOPs failed: {e}")
            lat = {}
            for d in devices:
                md = model.to(d); xd = x.to(d)
                lat[d] = timeit(lambda md=md, xd=xd: md(xd), args.iters, d)
            rows.append((name, fl, lat, color))
            print(f"{name}: {'?' if fl is None else f'{fl/1e9:.2f} GFLOPs'} | " +
                  " | ".join(f"{d} {lat[d]:.2f}ms" for d in lat))
        except Exception as e:
            print(f"{name} skipped:", repr(e))

    try:
        m = torch.hub.load("facebookresearch/dinov2", "dinov2_vits14", pretrained=True)
        add_net("DINOv2", m, lambda: torch.randn(1, 3, 224, 224), "#3b82f6")
    except Exception as e:
        print("DINOv2 skipped:", repr(e))

    try:
        import torch.nn as nn
        from src.gsr.descriptor import GlobalDesc

        class NVWrap(nn.Module):                      # netvlad takes {'image': x}
            def __init__(self, nv): super().__init__(); self.nv = nv
            def forward(self, x): return self.nv({"image": x})["global_descriptor"]
        add_net("NetVLAD", NVWrap(GlobalDesc().netvlad),
                lambda: torch.rand(1, 3, 224, 224), "#f59e0b")
    except Exception as e:
        print("NetVLAD skipped:", repr(e))

    try:
        m = torch.hub.load("serizba/salad", "dinov2_salad", trust_repo=True)
        add_net("SALAD", m, lambda: torch.randn(1, 3, 322, 322), "#ef4444")
    except Exception as e:
        print("SALAD skipped:", repr(e))

    # ---------------- plot (clean, paper/project-page style) ----------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Liberation Serif", "Nimbus Roman", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "xtick.direction": "in", "ytick.direction": "in",
        "xtick.major.size": 4.5, "ytick.major.size": 4.5,
        "ytick.minor.size": 2.5, "ytick.minor.visible": True,
        "axes.linewidth": 0.9, "font.size": 14,
    })
    # soft pastel palette, SOLiD as the highlighted hero
    PAL = {"SOLiD": "#5fbf8f", "DINOv2": "#9ec1e8", "NetVLAD": "#eccd8f", "SALAD": "#e39a9a"}
    names = [r[0] for r in rows]
    cols = [PAL.get(n, "#c9c9c9") for n in names]
    fl = [r[1] if r[1] else np.nan for r in rows]
    cpu = [r[2].get("cpu", np.nan) for r in rows]
    gpu = [r[2].get("cuda", np.nan) for r in rows]

    def fmt_flops(v):
        return f"{v/1e9:.1f}G" if v >= 1e9 else f"{v/1e6:.2f}M"
    panels = [(fl, "FLOPs", fmt_flops),
              (cpu, "CPU latency (ms)", lambda v: f"{v:.1f}"),
              (gpu, "GPU latency (ms)", lambda v: f"{v:.2f}")]
    x = np.arange(len(names))
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    for ax, (vals, ylab, fmt) in zip(axes, panels):
        ax.bar(x, vals, width=0.6, color=cols, edgecolor="white", linewidth=1.2, zorder=3)
        ax.set_yscale("log")
        ax.set_ylabel(ylab, fontsize=15)
        ax.set_xticks(x); ax.set_xticklabels(names, fontsize=13)
        ax.tick_params(axis="x", length=0)
        ax.grid(axis="y", which="major", alpha=0.16, zorder=0)
        ax.set_axisbelow(True)
        good = [v for v in vals if v == v]
        top = max(good) * 4 if good else 1
        ax.set_ylim(top=top)
        for xi, v in zip(x, vals):
            if v == v:
                ax.text(xi, v * 1.25, fmt(v), ha="center", va="bottom", fontsize=12.5)
    plt.tight_layout()
    plt.savefig("compute_comparison.png", dpi=200, bbox_inches="tight")
    print("saved compute_comparison.png")


if __name__ == "__main__":
    main()
