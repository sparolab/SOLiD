"""SOLiD place recognition (+ optional yaw-seeded ICP) for LoopSplat.

LoopSplat detects loops by cosine similarity of per-keyframe descriptors
(`Loop_closure.update_submaps_info` -> NetVLAD; `detect_closure` -> einsum vs an
adaptive `self_sim`). SOLiD's `rsolid` similarity is ALSO cosine, so an
L2-normalized rsolid is a drop-in for the NetVLAD descriptor: only
`update_submaps_info` changes; `detect_closure` / pose-graph / PGO / deformation
are inherited unchanged.

Extra: SOLiD also yields a yaw estimate (from `asolid`). With use_solid_yaw, the
loop-edge registration is done by ICP seeded with that yaw (about gravity/up),
instead of gs_reg — to test whether ICP "attaches" better with the SOLiD prior.

Inject from a runner (see run_slam_lc.py):
    lc = SolidLoopClosure(config, gslam.dataset, gslam.logger)
    lc.submap_path = gslam.output_path / "submaps"
    lc.est_c2ws = gslam.estimated_c2ws      # live tensor ref (poses)
    gslam.loop_closer = lc
"""
import numpy as np
import torch
import cv2
import open3d as o3d

import solid
from src.entities.lc import Loop_closure


class SolidLoopClosure(Loop_closure):
    def __init__(self, config, dataset, logger):
        super().__init__(config, dataset, logger)
        cam = config["cam"]
        lc = config.get("lc", {})
        self.solid_cfg = solid.load_config(lc.get("solid_profile", "gs_rgbd"))
        self.solid_ext = solid.Extractor(self.solid_cfg)
        self.fx = cam["fx"]; self.fy = cam.get("fy", cam["fx"])
        self.cx = cam["cx"]; self.cy = cam["cy"]
        self.depth_scale = cam["depth_scale"]
        self.depth_trunc = self.solid_cfg.max_range
        self.px_stride = 3
        self.est_c2ws = None                  # live estimated_c2ws tensor (set externally)
        self.solid_asolid = {}                # submap_id -> representative asolid (yaw)
        self.use_solid_yaw = lc.get("use_solid_yaw", True)
        # mean-removal: rsolid is a non-negative range histogram, so all office
        # scenes share a large common component -> cosine ~0.9 for everything
        # (aliasing). Subtracting the running global mean rsolid before L2-norm
        # roughly DOUBLES true/false separation (0.08 -> 0.16 on fr3). Toggle via
        # SOLID_MEAN_REMOVE (default on).
        self.mean_remove = lc.get("mean_remove", True)
        self._rsolid_sum = None
        self._rsolid_cnt = 0
        # floor-gravity: align each cloud's z to the floor-plane normal (drift-
        # INVARIANT) instead of the drifting est pose -> stable rsolid across runs,
        # so strict detection catches the loop reliably. Falls back to est pose when
        # no floor-like plane is found. Toggle SOLID_FLOOR_GRAVITY (default on).
        self.floor_gravity = lc.get("floor_gravity", True)
        print(f"[SolidLoopClosure] profile=gs_rgbd use_solid_yaw={self.use_solid_yaw} "
              f"mean_remove={self.mean_remove} floor_gravity={self.floor_gravity}")

    @staticmethod
    def _R_align(n, z=np.array([0, 0, 1.0])):
        n = n / (np.linalg.norm(n) + 1e-9)
        v = np.cross(n, z); s = np.linalg.norm(v); c = float(np.dot(n, z))
        if s < 1e-8:
            return np.eye(3) if c > 0 else np.diag([1., -1., -1.])
        vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))

    def _floor_R(self, p_cam, est_R):
        """Drift-invariant gravity: rotate so the floor normal -> +Z. Use the floor
        plane only if it agrees (<40 deg) with the est-pose up, else est pose."""
        est_up = est_R[:3, :3].T @ np.array([0, 0, 1.0])   # world-up in camera frame
        try:
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(p_cam.astype(np.float64))
            model, inl = pc.segment_plane(0.05, 3, 150)
            n = np.array(model[:3]); n /= (np.linalg.norm(n) + 1e-9)
            if np.dot(n, est_up) < 0:
                n = -n
            if np.dot(n, est_up) > np.cos(np.deg2rad(40)):   # plane ~ floor
                return self._R_align(n)
        except Exception:
            pass
        return est_R[:3, :3]                                 # fallback: est pose

    # --- build a gravity/pose-aligned point cloud from a keyframe's depth ---
    def _cloud(self, kf_id):
        depth = cv2.imread(str(self.dataset.depth_paths[kf_id]), cv2.IMREAD_UNCHANGED)
        depth = depth.astype(np.float32) / self.depth_scale
        h, w = depth.shape
        vs, us = np.mgrid[0:h:self.px_stride, 0:w:self.px_stride]
        d = depth[vs, us]
        m = (d > 0) & (d < self.depth_trunc)
        d = d[m]
        x = (us[m] - self.cx) * d / self.fx
        y = (vs[m] - self.cy) * d / self.fy
        p_cam = np.stack([x, y, d], axis=-1).astype(np.float32)
        c2w = self.est_c2ws[kf_id]
        c2w = c2w.detach().cpu().numpy() if hasattr(c2w, "detach") else np.asarray(c2w)
        R = self._floor_R(p_cam, c2w) if self.floor_gravity else c2w[:3, :3]
        return (R @ p_cam.T).T.astype(np.float32)

    # --- SOLiD descriptors replace NetVLAD (cosine-compatible drop-in) ---
    def update_submaps_info(self, keyframes_info):
        kf_ids = sorted(keyframes_info.keys())
        raws, asolids = [], []
        for kf in kf_ids:
            d = self.solid_ext.extract(self._cloud(kf))
            raws.append(np.asarray(d.rsolid, np.float32))
            asolids.append(np.asarray(d.asolid, np.float64))
        # update running global mean over all rsolid seen so far
        if self.mean_remove:
            for r in raws:
                self._rsolid_sum = r.copy() if self._rsolid_sum is None else self._rsolid_sum + r
                self._rsolid_cnt += 1
            mean = self._rsolid_sum / self._rsolid_cnt
        else:
            mean = 0.0
        descs = []
        for r in raws:
            v = r - mean                       # remove common office-histogram component
            n = np.linalg.norm(v)
            if n > 0:
                v = v / n                      # L2-normalize -> einsum == cosine
            descs.append(torch.from_numpy(v).float().to(self.device)[None])
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
        self.solid_asolid[self.submap_id] = asolids[len(asolids) // 2]

    # --- SOLiD-yaw-seeded ICP for loop-edge registration ---
    def pairwise_registration(self, submap_source, submap_target, method="gs_reg"):
        if not (self.use_solid_yaw and method == "gs_reg"):
            return super().pairwise_registration(submap_source, submap_target, method)

        seg_s = self.submap_to_segment(submap_source)
        seg_t = self.submap_to_segment(submap_target)
        cs = o3d.geometry.PointCloud()
        cs.points = o3d.utility.Vector3dVector(np.array(seg_s["points"]))
        cs.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=50))
        ct = o3d.geometry.PointCloud()
        ct.points = o3d.utility.Vector3dVector(np.array(seg_t["points"]))
        ct.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=50))

        # SOLiD yaw (deg) between the two submaps -> initial rotation about up (world Z)
        yaw = np.deg2rad(solid.Extractor.pose_yaw_deg(
            self.solid_asolid[submap_source["submap_id"]],
            self.solid_asolid[submap_target["submap_id"]]))
        c, s = np.cos(yaw), np.sin(yaw)
        init = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

        coarse = o3d.pipelines.registration.registration_icp(
            cs, ct, 0.3, init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        fine = o3d.pipelines.registration.registration_icp(
            cs, ct, 0.03, coarse.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        info = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
            cs, ct, 0.03, fine.transformation)
        min_overlap = self.config.get("lc", {}).get("registration", {}).get(
            "min_overlap_ratio", 0.2)
        successful = bool(fine.fitness >= min_overlap)
        print(f"[SOLiD-LC reg] submap {submap_source['submap_id']}->{submap_target['submap_id']} "
              f"fitness={fine.fitness:.3f} thr={min_overlap:.2f} -> {'ACCEPT' if successful else 'REJECT'}", flush=True)
        return {"transformation": np.array(fine.transformation),
                "information": np.array(info),
                "successful": successful,
                "overlap": float(fine.fitness)}
