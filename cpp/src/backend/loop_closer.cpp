#include "solid/backend/loop_closer.hpp"

#include <cmath>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

namespace solid {
namespace backend {

namespace {

gtsam::Pose3 to_gtsam(const Eigen::Isometry3d& T) { return gtsam::Pose3(T.matrix()); }

Eigen::Isometry3d to_eigen(const gtsam::Pose3& P) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.matrix() = P.matrix();
  return T;
}

gtsam::SharedNoiseModel diag(double trans, double rot_deg) {
  const double r = rot_deg * M_PI / 180.0;
  return gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << r, r, r, trans, trans, trans).finished());
}

LoopCloser::Cloud::Ptr voxel(const LoopCloser::Cloud::ConstPtr& in, double leaf) {
  LoopCloser::Cloud::Ptr out(new LoopCloser::Cloud);
  if (leaf <= 0.0) {
    *out = *in;
    return out;
  }
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setLeafSize(leaf, leaf, leaf);
  vg.setInputCloud(in);
  vg.filter(*out);
  return out;
}

}  // namespace

struct LoopCloser::Impl {
  LoopClosingParams p;
  std::vector<Cloud::Ptr> scans;
  std::vector<Eigen::Isometry3d> odom;
  std::vector<Eigen::Isometry3d> poses;
  std::vector<LoopResult> loops;
  gtsam::ISAM2 isam;
  gtsam::NonlinearFactorGraph pending;
  gtsam::Values pending_values;

  explicit Impl(const LoopClosingParams& params) : p(params) {
    gtsam::ISAM2Params ip;
    ip.relinearizeThreshold = 0.01;
    ip.relinearizeSkip = 1;
    isam = gtsam::ISAM2(ip);
  }

  // Keyframes i-n..i+n, in the frame of keyframe i, using the current pose estimates.
  Cloud::Ptr submap(std::size_t i, int n) const {
    Cloud::Ptr out(new Cloud);
    const Eigen::Isometry3d inv = poses[i].inverse();
    const long lo = std::max<long>(0, static_cast<long>(i) - n);
    const long hi = std::min<long>(static_cast<long>(scans.size()) - 1, static_cast<long>(i) + n);
    for (long k = lo; k <= hi; ++k) {
      Cloud tmp;
      pcl::transformPointCloud(*scans[k], tmp, (inv * poses[k]).matrix().cast<float>());
      *out += tmp;
    }
    return voxel(out, p.icp_voxel);
  }

  void update() {
    isam.update(pending, pending_values);
    pending.resize(0);
    pending_values.clear();
    if (!loops.empty()) isam.update();  // extra iterations after a loop
    const gtsam::Values est = isam.calculateEstimate();
    for (std::size_t i = 0; i < poses.size(); ++i)
      poses[i] = to_eigen(est.at<gtsam::Pose3>(gtsam::Symbol('x', i)));
  }
};

LoopCloser::LoopCloser(const LoopClosingParams& p) : impl_(new Impl(p)) {}
LoopCloser::~LoopCloser() = default;

void LoopCloser::add_keyframe(std::size_t id, const Eigen::Isometry3d& odom_pose, const Cloud& scan) {
  Impl& m = *impl_;
  m.scans.push_back(voxel(Cloud::ConstPtr(new Cloud(scan)), m.p.scan_voxel));
  const gtsam::Symbol key('x', id);
  if (m.odom.empty()) {
    m.pending.add(gtsam::PriorFactor<gtsam::Pose3>(key, to_gtsam(odom_pose), diag(1e-4, 1e-4)));
    m.pending_values.insert(key, to_gtsam(odom_pose));
    m.poses.push_back(odom_pose);
  } else {
    const Eigen::Isometry3d rel = m.odom.back().inverse() * odom_pose;
    m.pending.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', id - 1), key, to_gtsam(rel),
                                                     diag(m.p.odom_sigma_trans, m.p.odom_sigma_rot_deg)));
    const Eigen::Isometry3d guess = m.poses.back() * rel;
    m.pending_values.insert(key, to_gtsam(guess));
    m.poses.push_back(guess);
  }
  m.odom.push_back(odom_pose);
  m.update();
}

LoopResult LoopCloser::try_loop(std::size_t query, std::size_t match, double yaw_rad) {
  Impl& m = *impl_;
  LoopResult r;
  r.query = query;
  r.match = match;
  if (query >= m.scans.size() || match >= m.scans.size()) return r;

  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
  gicp.setMaxCorrespondenceDistance(m.p.icp_max_dist);
  gicp.setMaximumIterations(50);
  gicp.setTransformationEpsilon(1e-6);
  gicp.setInputSource(m.submap(query, m.p.submap_keyframes));
  gicp.setInputTarget(m.submap(match, m.p.submap_keyframes));
  // The SOLiD yaw sign depends on which scan is the reference; keep the better fit.
  for (double yaw : {yaw_rad, -yaw_rad}) {
    Cloud aligned;
    gicp.align(aligned, Eigen::Affine3f(Eigen::AngleAxisf(static_cast<float>(yaw),
                                                          Eigen::Vector3f::UnitZ())).matrix());
    if (!gicp.hasConverged()) continue;
    const double fit = gicp.getFitnessScore(m.p.icp_max_dist);
    if (r.fitness < 0.0 || fit < r.fitness) {
      r.fitness = fit;
      r.T_match_query.matrix() = gicp.getFinalTransformation().cast<double>();
    }
  }
  r.accepted = r.fitness >= 0.0 && r.fitness < m.p.icp_max_fitness;
  if (!r.accepted) return r;

  const auto noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
      diag(m.p.loop_sigma_trans, m.p.loop_sigma_rot_deg));
  m.pending.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', match), gtsam::Symbol('x', query),
                                                   to_gtsam(r.T_match_query), noise));
  m.loops.push_back(r);
  m.update();
  return r;
}

const std::vector<Eigen::Isometry3d>& LoopCloser::poses() const { return impl_->poses; }

Eigen::Isometry3d LoopCloser::correction() const {
  if (impl_->poses.empty()) return Eigen::Isometry3d::Identity();
  return impl_->poses.back() * impl_->odom.back().inverse();
}

const std::vector<LoopResult>& LoopCloser::accepted_loops() const { return impl_->loops; }

}  // namespace backend
}  // namespace solid
