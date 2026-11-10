"""SOLiD multi-session GIF (fig/solid_multi_session_v2.gif): LiDAR (left) and 4D radar (right). Sessions recorded separately
start where their own odometry put them, then snap into one map with SOLiD's alignment, and the
inter-session loops appear.

    python fig/gen/ms.py solid_multi_session.gif --lidar <LiDAR sessions> --radar <radar sessions>
The sessions are aligned first with solid_align_sessions (aligned_ab/, aligned_st3/).
Needs opencv-python, scipy and ffmpeg."""
import csv
import os
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot, Slerp

import argparse
import tempfile

ap = argparse.ArgumentParser(description=__doc__)
ap.add_argument("out", help="output .gif")
ap.add_argument("--lidar", default="data/lidar/sessions", help="folder of the LiDAR sessions and aligned_ab/")
ap.add_argument("--radar", default="data/radar/sessions", help="folder of the radar sessions and aligned_st3/")
ap.add_argument("--frames", help="keep the PNG frames here")
ap.add_argument("--only", default="", help="render only these frames (preview)")
ARGS = ap.parse_args()
OUT = ARGS.frames or tempfile.mkdtemp(prefix="solid_gif_")
ICONS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icons")
os.makedirs(OUT, exist_ok=True)
W, H, SS = 800, 450, 2
PW = W // 2
APART, MOVE, LOOPS, MERGE, HOLD = 30, 80, 40, 50, 40
TOTAL = APART + MOVE + LOOPS + MERGE + HOLD
STACK = 0.22  # height gap between sessions while the loops are shown, as a fraction of the map size
ONLY = [int(x) for x in ARGS.only.split(",") if x]
rng = np.random.default_rng(0)
ELEV = 50.0

INDIGO, MAGENTA, TEAL = np.array([63, 81, 181]) / 255, np.array([216, 27, 96]) / 255, np.array([0, 137, 123]) / 255
L = ARGS.lidar
R = ARGS.radar
PANELS = [
    dict(icon="lidar", title="LiDAR  ·  2 sessions", dir=L, aligned=f"{L}/aligned_ab",
         names=["kitti00_a", "kitti00_b"], cols=[INDIGO, TEAL], npts=600, stride=3, stack=0.35),
    dict(icon="radar", title="4D radar  ·  3 days", dir=R, aligned=f"{R}/aligned_st3",
         names=["st_20240113", "st_20231208", "st_20231213"], cols=[INDIGO, MAGENTA, TEAL], npts=500, stride=2),
]


def kitti(p):
    a = np.loadtxt(p).reshape(-1, 3, 4)
    T = np.tile(np.eye(4), (len(a), 1, 1))
    T[:, :3, :4] = a
    return T


def read_pcd(p, n):
    with open(p, "rb") as f:
        while not f.readline().startswith(b"DATA"):
            pass
        a = np.frombuffer(f.read(), np.float32).reshape(-1, 4)[:, :3]
    return a[rng.choice(len(a), n, replace=False)] if len(a) > n else a


def ease(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)


for P in PANELS:
    P["sess"] = []
    for n in P["names"]:
        odo = kitti(f"{P['dir']}/{n}/odom_poses.txt")
        ali = kitti(f"{P['aligned']}/{n}_aligned.txt")
        Dm = ali[0] @ np.linalg.inv(odo[0])
        local = [(i, read_pcd(f"{P['dir']}/{n}/Scans/{i:06d}.pcd", P["npts"])) for i in range(0, len(odo), P["stride"])]
        P["sess"].append(dict(odo=odo, ali=ali, D=Dm, local=local))
    P["loops"] = []
    for r in csv.DictReader(open(f"{P['aligned']}/inter_session_loops.csv")):
        if r["inlier"] == "1":
            P["loops"].append((P["names"].index(r["query_session"]), int(r["central_kf"]), int(r["query_kf"])))
    allp = np.concatenate([s["odo"][:, :3, 3] for s in P["sess"]] + [s["ali"][:, :3, 3] for s in P["sess"]])
    tight = np.concatenate([s["ali"][:, :3, 3] for s in P["sess"]])
    P["ground"] = np.median(tight[:, 2])

    def view(pts, c=None):
        lo, hi = pts[:, :2].min(0), pts[:, :2].max(0)
        return (lo + hi) / 2, max(hi[0] - lo[0], (hi[1] - lo[1]) / np.sin(np.radians(ELEV)) * PW / (H - 50))
    P["wide"], P["tight"] = view(allp), view(tight)
    n = len(P["names"])
    P["dz"] = P.get("stack", STACK) * P["tight"][1]
    # stacked view: the same map, taller by the height gap
    P["stack"] = (P["tight"][0], max(P["tight"][1], P["tight"][1] + (n - 1) * P["dz"] * np.cos(np.radians(ELEV)) / np.sin(np.radians(ELEV)) * PW / (H - 50) * 0.5))


def project(P, xyz, k, m=1.0):
    # k: zoom from the wide view to the stacked view; m: merge from the stacked view to the tight one
    (cw, sw), (ct, st), (_, sk) = P["wide"], P["tight"], P["stack"]
    st = (1 - m) * sk + m * st
    c = (1 - k) * cw + k * ct
    span = (1 - k) * sw + k * st * 1.08
    s = 0.92 * PW * SS / span
    e = np.radians(ELEV)
    x, y = xyz[:, 0] - c[0], xyz[:, 1] - c[1]
    z = xyz[:, 2] - P["ground"] - k * (1 - m) * (len(P["names"]) - 1) * P["dz"] / 2
    return np.stack([PW * SS / 2 + s * x, H * SS / 2 + 12 * SS - s * (y * np.sin(e) + z * np.cos(e))], 1)


def splat(uv, col, k=0.6):
    w, h = PW * SS, H * SS
    u, v = uv[:, 0].astype(int), uv[:, 1].astype(int)
    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    lin = v[ok] * w + u[ok]
    cnt = np.bincount(lin, minlength=w * h).astype(np.float32)
    acc = np.stack([np.bincount(lin, weights=col[ok, i], minlength=w * h) for i in range(3)], 1)
    rgb = acc / np.maximum(cnt, 1)[:, None]
    a = 1 - np.exp(-k * cnt)
    img = np.ones((w * h, 3)) * (1 - a[:, None]) + rgb * a[:, None]
    return (img.reshape(h, w, 3)[:, :, ::-1] * 255).astype(np.uint8).copy()


def bgr(c):
    return tuple(int(255 * v) for v in c[::-1])


icons = {k: cv2.imread(f"{ICONS}/{k}.png", cv2.IMREAD_UNCHANGED) for k in ("lidar", "radar")}

for f in (ONLY or range(TOTAL)):
    snap = ease((f - APART) / MOVE)
    zoom = ease((f - APART + 10) / (MOVE + 10))
    loop_a = np.clip((f - APART - MOVE) / (LOOPS * 0.6), 0, 1)
    merge = ease((f - APART - MOVE - LOOPS) / MERGE)
    panels = []
    for P in PANELS:
        pts, cols, trajs = [], [], []
        lifts = [j * P["dz"] * snap * (1 - merge) for j in range(len(P["sess"]))]
        for s, c, lift in zip(P["sess"], P["cols"], lifts):
            rot = Slerp([0, 1], Rot.from_matrix([np.eye(3), s["D"][:3, :3]]))
            M = np.eye(4)
            M[:3, :3] = rot(snap).as_matrix()
            M[:3, 3] = snap * s["D"][:3, 3]
            poses = (s["ali"] if snap >= 1.0 else np.einsum("ij,njk->nik", M, s["odo"])).copy()
            poses[:, 2, 3] += lift
            for i, p in s["local"]:
                pts.append((poses[i][:3, :3] @ p.T).T + poses[i][:3, 3])
                cols.append(np.tile(c * 0.85, (len(p), 1)))
            trajs.append(poses[:, :3, 3])
        img = splat(project(P, np.concatenate(pts), zoom, merge), np.concatenate(cols))
        for t, c in zip(trajs, P["cols"]):
            cv2.polylines(img, [project(P, t, zoom, merge).astype(np.int32)], False, bgr(c * 0.75), int(1.8 * SS), cv2.LINE_AA)
        if loop_a > 0:
            lay = img.copy()
            c0 = trajs[0]
            for si, ck, qk in P["loops"]:
                a, b = c0[ck], trajs[si][qk]
                pa, pb = project(P, np.array([a, b]), zoom, merge).astype(int)
                cv2.line(lay, tuple(pa), tuple(pb), (40, 40, 40), SS, cv2.LINE_AA)
                for q in (pa, pb):
                    cv2.circle(lay, tuple(q), int(1.6 * SS), (40, 40, 40), -1, cv2.LINE_AA)
            w = (0.75 - 0.25 * merge) * loop_a  # strong while the layers are apart, lighter once merged
            img = cv2.addWeighted(lay, w, img, 1 - w, 0)
        tile = cv2.resize(img, (PW, H), interpolation=cv2.INTER_AREA)
        ic = icons[P["icon"]]
        h, w = ic.shape[:2]
        al = ic[:, :, 3:4] / 255.0
        tile[8:8 + h, 8:8 + w] = (tile[8:8 + h, 8:8 + w] * (1 - al) + ic[:, :, :3] * al).astype(np.uint8)
        cv2.putText(tile, P["title"], (8 + w + 8, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 40, 40), 1, cv2.LINE_AA)
        panels.append(tile)
    frame = np.concatenate(panels, 1)
    cv2.line(frame, (PW, 10), (PW, H - 10), (225, 225, 225), 1)
    cv2.imwrite(f"{OUT}/{f:04d}.png", frame)
print("frames", TOTAL)

if not ONLY:
    import shutil
    import subprocess
    if shutil.which("ffmpeg") is None:
        sys.exit(f"ffmpeg not found; the frames are in {OUT}")
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-framerate", "20", "-i", f"{OUT}/%04d.png", "-vf",
                    "split[a][b];[a]palettegen=max_colors=128[p];[b][p]paletteuse=dither=bayer:bayer_scale=4",
                    ARGS.out], check=True)
    print(f"-> {ARGS.out}")
