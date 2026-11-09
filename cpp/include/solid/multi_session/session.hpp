// A keyframe session saved by a SOLiD front-end (ROS 1 / ROS 2 node, Mac tool):
// singlesession_posegraph.g2o + SOLiDs/ + Scans/ (LT-mapper layout).
#ifndef SOLID_MULTI_SESSION_SESSION_HPP
#define SOLID_MULTI_SESSION_SESSION_HPP

#include <map>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "solid/descriptor.hpp"

namespace solid {
namespace ms {

using PointT = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointT>;

struct OdomEdge {
  int from = 0, to = 0;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();  // pose of `to` in `from`
};

class Session {
 public:
  // Reads poses, odometry edges and SOLiD descriptors. Scans are loaded lazily.
  Session(const std::string& dir, const std::string& name, double scan_voxel);

  const std::string& name() const { return name_; }
  const std::string& dir() const { return dir_; }
  std::size_t size() const { return poses_.size(); }
  const std::vector<Eigen::Isometry3d>& poses() const { return poses_; }
  const std::vector<OdomEdge>& edges() const { return edges_; }
  const std::vector<solid::Descriptor>& descriptors() const { return descs_; }
  const std::vector<double>& stamps() const { return stamps_; }

  // Downsampled keyframe scan in its own sensor frame (cached).
  Cloud::ConstPtr scan(int i) const;
  // Keyframes i-n..i+n merged into the frame of keyframe i.
  Cloud::Ptr submap(int i, int n) const;

 private:
  std::string dir_, name_;
  double voxel_;
  std::vector<Eigen::Isometry3d> poses_;
  std::vector<OdomEdge> edges_;
  std::vector<solid::Descriptor> descs_;
  std::vector<double> stamps_;
  mutable std::map<int, Cloud::Ptr> cache_;
};

}  // namespace ms
}  // namespace solid

#endif  // SOLID_MULTI_SESSION_SESSION_HPP
