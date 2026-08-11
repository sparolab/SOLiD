"""Open3D before/after loop-closure snapshots with camera wireframes."""
import copy
import os

import numpy as np
import open3d as o3d


def camera_wireframe(poses, color, scale=0.08, stride=1):
    """Create 5-point camera pyramids for camera-to-world SE(3) poses."""
    w, h, f = 0.75 * scale, 0.55 * scale, scale
    corners = np.array([[w, h, f], [w, -h, f], [-w, -h, f], [-w, h, f]])
    points, lines = [], []
    for pose in poses[::stride]:
        R, t = pose[:3, :3], pose[:3, 3]
        world_corners = (R @ corners.T + t[:, None]).T
        base = len(points)
        points.append(t.tolist()); points.extend(world_corners.tolist())
        for i in range(4):
            lines.append([base, base + 1 + i])
            lines.append([base + 1 + i, base + 1 + (i + 1) % 4])
    result = o3d.geometry.LineSet()
    result.points = o3d.utility.Vector3dVector(np.asarray(points))
    result.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    result.colors = o3d.utility.Vector3dVector(np.tile(color, (len(lines), 1)))
    return result


def capture(solver, voxel=0.02):
    """Freeze the current regular-submap map and optimized camera poses."""
    point_chunks, color_chunks, pose_chunks = [], [], []
    for submap in solver.map.ordered_submaps_by_key():
        if submap.get_lc_status():
            continue
        points = submap.get_points_in_world_frame(solver.graph).astype(np.float64)
        colors = submap.get_points_colors().astype(np.float64)
        if colors.max(initial=0) > 1:
            colors /= 255.0
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.colors = o3d.utility.Vector3dVector(colors)
        if voxel > 0:
            cloud = cloud.voxel_down_sample(voxel)
        point_chunks.append(np.asarray(cloud.points).copy())
        color_chunks.append(np.asarray(cloud.colors).copy())
        pose_chunks.append(submap.get_all_poses_world(solver.graph))
    return {
        "points": np.concatenate(point_chunks),
        "colors": np.concatenate(color_chunks),
        "poses": np.concatenate(pose_chunks),
    }


def _cloud(snapshot):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(snapshot["points"])
    cloud.colors = o3d.utility.Vector3dVector(snapshot["colors"])
    return cloud


def save_and_show(before, after, out_dir, loop_index, show=True, cam_stride=1):
    """Save NPZ/PLY artifacts and optionally open a side-by-side Open3D window."""
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.join(out_dir, f"loop_{loop_index:02d}")
    for name, snap in (("before", before), ("after", after)):
        np.savez_compressed(f"{stem}_{name}.npz", **snap)
        o3d.io.write_point_cloud(f"{stem}_{name}.ply", _cloud(snap))

    extent = max(np.ptp(before["points"], axis=0).max(),
                 np.ptp(after["points"], axis=0).max())
    cam_scale = max(extent * 0.012, 1e-3)
    before_cam = camera_wireframe(before["poses"], [0.9, 0.1, 0.1], cam_scale, cam_stride)
    after_cam = camera_wireframe(after["poses"], [0.05, 0.75, 0.05], cam_scale, cam_stride)
    o3d.io.write_line_set(f"{stem}_before_cameras.ply", before_cam)
    o3d.io.write_line_set(f"{stem}_after_cameras.ply", after_cam)
    print(f"[O3D-LC] saved {stem}_before/after.(npz|ply)")

    if show:
        before_cloud, after_cloud = _cloud(before), _cloud(after)
        shift = np.array([extent * 1.25, 0, 0])
        before_cloud.translate(-0.5 * shift); before_cam.translate(-0.5 * shift)
        after_cloud.translate(0.5 * shift); after_cam.translate(0.5 * shift)
        o3d.visualization.draw_geometries(
            [before_cloud, before_cam, after_cloud, after_cam],
            window_name="Loop closure: BEFORE (left/red) | AFTER (right/green)",
            width=1600, height=900)
