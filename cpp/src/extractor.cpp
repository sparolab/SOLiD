// SOLiD core — descriptor extractor implementation.
//
// This is a faithful, parameterized port of the reference SOLiDModule
// (reference/SOLiD/cpp/src/solid.cpp). With default Config values every
// numeric result is identical to the original; see test/verify_equivalence.cpp.
#include "solid/extractor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

// PCL is used only for voxel downsampling. It is optional so the Python wheel
// (and any lightweight consumer) can build PCL-free; define SOLID_USE_PCL to use
// PCL's VoxelGrid (the default for the C++ library and ROS wrappers, which keeps
// the descriptor bit-identical to the original reference).
#ifdef SOLID_USE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#endif

namespace solid {

namespace {

// Bin indices for one point. Mirrors SOLiDModule::pt2rah / xy2theta.
struct RAH {
  int idx_range = 0;
  int idx_angle = 0;
  int idx_height = 0;
};

inline float xy2theta(float x, float y) {
  const float k = 180.0f / static_cast<float>(M_PI);
  if (x >= 0 && y >= 0) return k * std::atan(y / x);
  if (x < 0 && y > 0) return 180.0f - k * std::atan(y / (-x));
  if (x < 0 && y < 0) return 180.0f + k * std::atan(y / x);
  // x >= 0 && y < 0
  return 360.0f - k * std::atan((-y) / x);
}

inline float rad2deg(float rad) { return rad * 180.0f / static_cast<float>(M_PI); }

RAH pt2rah(const Eigen::Vector3f& p, const Config& cfg, float gap_angle,
           float gap_range, float gap_height) {
  float x = p.x();
  float y = p.y();
  float z = p.z();
  if (x == 0.0f) x = 0.001f;
  if (y == 0.0f) y = 0.001f;

  const float theta = xy2theta(x, y);
  const float dist_xy = std::sqrt(x * x + y * y);
  const float phi = rad2deg(std::atan2(z, dist_xy));

  // Clamp to valid bin ranges. The upper min() matches the original reference;
  // the lower max(0, ...) is new but only triggers for points outside the
  // vertical FOV (phi < fov_down), which never occurs for in-FOV LiDAR (so the
  // golden output is unchanged) but does occur for dense camera/foundation-model
  // clouds — where the missing lower clamp caused a negative index and a crash.
  RAH rah;
  rah.idx_range = std::max(
      0, std::min(static_cast<int>(dist_xy / gap_range), cfg.num_range - 1));
  rah.idx_angle = std::max(
      0, std::min(static_cast<int>(theta / gap_angle), cfg.num_angle - 1));
  rah.idx_height = std::max(
      0, std::min(static_cast<int>((phi - cfg.fov_down) / gap_height),
                  cfg.num_height - 1));
  return rah;
}

}  // namespace

void Extractor::preprocess(const std::vector<Eigen::Vector3f>& pts_in,
                           const std::vector<float>* weight_in,
                           std::vector<Eigen::Vector3f>& pts_out,
                           std::vector<float>& weight_out) const {
  pts_out.clear();
  weight_out.clear();
  const bool has_w = cfg_.use_weight && weight_in != nullptr &&
                     weight_in->size() == pts_in.size();

  // range gate: keep min_range < dist < max_range (3D distance, as reference).
  pts_out.reserve(pts_in.size());
  if (has_w) weight_out.reserve(pts_in.size());
  for (std::size_t i = 0; i < pts_in.size(); ++i) {
    const Eigen::Vector3f& p = pts_in[i];
    const float dist = std::sqrt(p.x() * p.x() + p.y() * p.y() + p.z() * p.z());
    if (dist > cfg_.min_range && dist < cfg_.max_range) {
      pts_out.push_back(p);
      if (has_w) weight_out.push_back((*weight_in)[i]);
    }
  }

  // Optional voxel downsample. Skipped when weighting is active so per-point
  // weights stay aligned (radar profile should set voxel_size <= 0).
  if (cfg_.voxel_size > 0.0f && !has_w) {
#ifdef SOLID_USE_PCL
    // PCL VoxelGrid — bit-identical to the original SOLiD reference.
    pcl::PointCloud<pcl::PointXYZ>::Ptr in(new pcl::PointCloud<pcl::PointXYZ>);
    in->points.reserve(pts_out.size());
    for (const auto& p : pts_out) in->points.emplace_back(p.x(), p.y(), p.z());
    in->width = in->points.size();
    in->height = 1;

    pcl::PointCloud<pcl::PointXYZ> down;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(cfg_.voxel_size, cfg_.voxel_size, cfg_.voxel_size);
    vg.filter(down);

    pts_out.clear();
    pts_out.reserve(down.points.size());
    for (const auto& p : down.points) pts_out.emplace_back(p.x, p.y, p.z);
#else
    // PCL-free voxel centroid downsample (used by the Python wheel). May differ
    // from PCL at the sub-voxel level; semantically equivalent.
    struct Acc { double x = 0, y = 0, z = 0; std::uint64_t n = 0; };
    struct KeyHash {
      std::size_t operator()(const std::array<std::int64_t, 3>& k) const {
        std::size_t h = 1469598103934665603ull;
        for (std::int64_t v : k) {
          h = (h ^ static_cast<std::size_t>(v)) * 1099511628211ull;
        }
        return h;
      }
    };
    std::unordered_map<std::array<std::int64_t, 3>, Acc, KeyHash> grid;
    const double inv = 1.0 / cfg_.voxel_size;
    for (const auto& p : pts_out) {
      std::array<std::int64_t, 3> key{
          static_cast<std::int64_t>(std::floor(p.x() * inv)),
          static_cast<std::int64_t>(std::floor(p.y() * inv)),
          static_cast<std::int64_t>(std::floor(p.z() * inv))};
      Acc& a = grid[key];
      a.x += p.x();
      a.y += p.y();
      a.z += p.z();
      ++a.n;
    }
    pts_out.clear();
    pts_out.reserve(grid.size());
    for (const auto& kv : grid) {
      const Acc& a = kv.second;
      pts_out.emplace_back(static_cast<float>(a.x / a.n),
                           static_cast<float>(a.y / a.n),
                           static_cast<float>(a.z / a.n));
    }
#endif
  }
}

Descriptor Extractor::extract(const std::vector<Eigen::Vector3f>& pts,
                              const std::vector<float>* weight) const {
  std::vector<Eigen::Vector3f> p;
  std::vector<float> w;
  preprocess(pts, weight, p, w);
  const bool has_w = cfg_.use_weight && !w.empty() && w.size() == p.size();

  Eigen::MatrixXd range_matrix =
      Eigen::MatrixXd::Zero(cfg_.num_range, cfg_.num_height);
  Eigen::MatrixXd angle_matrix =
      Eigen::MatrixXd::Zero(cfg_.num_angle, cfg_.num_height);

  const float gap_angle = 360.0f / cfg_.num_angle;
  const float gap_range = cfg_.max_range / cfg_.num_range;
  const float gap_height = (cfg_.fov_up - cfg_.fov_down) / cfg_.num_height;

  for (std::size_t i = 0; i < p.size(); ++i) {
    const RAH rah = pt2rah(p[i], cfg_, gap_angle, gap_range, gap_height);
    const double contrib = has_w ? static_cast<double>(w[i]) : 1.0;
    range_matrix(rah.idx_range, rah.idx_height) += contrib;
    angle_matrix(rah.idx_angle, rah.idx_height) += contrib;
  }

  // per-height occupancy, min-max normalized (guarded against a constant column
  // set — the reference does not guard, but on non-degenerate data the result
  // is identical, and this avoids NaNs on sparse/empty scans).
  Eigen::VectorXd number_vector(cfg_.num_height);
  for (int c = 0; c < range_matrix.cols(); ++c) {
    number_vector(c) = range_matrix.col(c).sum();
  }
  const double min_val = number_vector.minCoeff();
  const double max_val = number_vector.maxCoeff();
  const double denom = max_val - min_val;
  if (denom > 1e-12) {
    number_vector = (number_vector.array() - min_val) / denom;
  } else {
    number_vector.setZero();
  }

  Descriptor d;
  d.rsolid = range_matrix * number_vector;  // length num_range
  d.asolid = angle_matrix * number_vector;  // length num_angle
  return d;
}

double Extractor::loop_similarity(const Eigen::VectorXd& q,
                                  const Eigen::VectorXd& c) {
  const double n = q.norm() * c.norm();
  if (n <= 0.0) return 0.0;
  return q.dot(c) / n;
}

double Extractor::pose_yaw_deg(const Eigen::VectorXd& a_query,
                               const Eigen::VectorXd& a_candidate) {
  const int n = static_cast<int>(a_query.size());
  double min_l1 = std::numeric_limits<double>::max();
  int min_index = 0;
  for (int shift = 0; shift < n; ++shift) {
    Eigen::VectorXd shifted = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
      shifted((i + shift) % n) = a_query(i);
    }
    const double l1 = (a_candidate - shifted).cwiseAbs().sum();
    if (l1 < min_l1) {
      min_l1 = l1;
      min_index = shift;
    }
  }
  return (min_index + 1) * (360.0 / n);
}

}  // namespace solid
