import numpy as np
import open3d as o3d

def remove_closest_points(points, thres):
    dists = np.sum(np.square(points[:, :3]), axis=1)
    cloud_out = points[dists > thres*thres]
    return cloud_out

def remove_far_points(points, thres):
    dists = np.sum(np.square(points[:, :3]), axis=1)
    cloud_out = points[dists < thres*thres]
    return cloud_out

def down_sampling(points, voxel_size=1):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, 0:3])
    # Voxel Down-sampling
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    # Compute the mean intensity for each voxel
    orig_points_np = np.asarray(pcd.points)
    down_points_np = np.asarray(down_pcd.points)
    # return down_points_np
    return down_points_np

def readScan(bin_path):
    ## HeLiPR Aeva Datasets
    # lidar_dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('reflectivity', np.float32), ('velocity', np.float32), ('time_offset_ns', np.int32), ('line_index', np.uint8)]
    # lidar_dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('reflectivity', np.float32), ('velocity', np.float32), ('time_offset_ns', np.int32), ('line_index', np.uint8), ('intensity', np.float32)]
    ## HeLiPR Avia Datasets
    # lidar_dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('reflectivity', np.uint8), ('tag', np.uint8), ('line', np.uint8), ('offset_time', np.uint32)]
    ## DISCO PARK Datasets
    # lidar_dtype=[('x', np.float64), ('y', np.float64), ('z', np.float64), ('intensity', np.float64)]
    ## KITTI Datasets
    lidar_dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)]

    scan         = np.fromfile(bin_path, dtype=lidar_dtype)
    points  = np.stack((scan['x'], scan['y'], scan['z']), axis = -1)
    
    return points
