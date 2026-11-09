// Online loop closing for one session (solid::backend).
//
// Keeps every keyframe's scan, checks each SOLiD loop candidate with GICP between
// keyframe submaps, and adds the accepted loops to an iSAM2 pose graph next to the
// odometry edges. The corrected keyframe poses (and the correction to apply to the
// latest odometry) are available after every keyframe.
#ifndef SOLID_BACKEND_LOOP_CLOSER_HPP
#define SOLID_BACKEND_LOOP_CLOSER_HPP

#include <memory>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace solid {
namespace backend {

struct LoopClosingParams {
  bool enable = false;
  int submap_keyframes = 5;          // +- keyframes merged around each loop keyframe
  double scan_voxel = 0.4;           // m, voxel of the stored keyframe scans
  double icp_voxel = 1.0;            // m, voxel of the submaps fed to GICP
  double icp_max_dist = 2.0;         // m, GICP max correspondence distance
  double icp_max_fitness = 0.3;      // m^2, mean squared residual to accept a loop
  double odom_sigma_trans = 0.1, odom_sigma_rot_deg = 1.0;
  double loop_sigma_trans = 0.5, loop_sigma_rot_deg = 3.0;
};

struct LoopResult {
  std::size_t query = 0, match = 0;
  bool accepted = false;
  double fitness = -1.0;
  Eigen::Isometry3d T_match_query = Eigen::Isometry3d::Identity();
};

class LoopCloser {
 public:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;

  explicit LoopCloser(const LoopClosingParams& p);
  ~LoopCloser();

  // Adds keyframe `id` (ids must be 0, 1, 2, ...) with its odometry pose and sensor-frame scan.
  void add_keyframe(std::size_t id, const Eigen::Isometry3d& odom_pose, const Cloud& scan);

  // Checks a candidate (query = newest keyframe) with GICP, SOLiD yaw as the initial guess.
  // On success adds the loop to the graph and re-optimizes.
  LoopResult try_loop(std::size_t query, std::size_t match, double yaw_rad);

  // Optimized keyframe poses (odometry poses until the first loop is accepted).
  const std::vector<Eigen::Isometry3d>& poses() const;
  // map <- odom: apply to an odometry pose to get the corrected pose.
  Eigen::Isometry3d correction() const;
  const std::vector<LoopResult>& accepted_loops() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace backend
}  // namespace solid

#endif  // SOLID_BACKEND_LOOP_CLOSER_HPP
