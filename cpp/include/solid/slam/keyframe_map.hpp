// SOLiD keyframe map: a light copy of every keyframe cloud, re-posed with the latest
// (loop-closed) keyframe poses. Used by the ROS nodes to show the corrected map.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>

namespace solid::slam {

class KeyframeMap {
 public:
  // voxel: leaf size (m) each keyframe cloud is reduced to; <= 0 keeps every point.
  explicit KeyframeMap(float voxel = 0.5f) : voxel_(voxel) {}

  // Stores keyframe `size()` (points in the keyframe's sensor frame, optional intensity).
  void add(const std::vector<Eigen::Vector3f>& pts, const std::vector<float>* intensity = nullptr) {
    Keyframe kf;
    if (voxel_ <= 0.0f) {
      kf.pts = pts;
      if (intensity && intensity->size() == pts.size()) kf.intensity = *intensity;
      else kf.intensity.assign(pts.size(), 0.0f);
    } else {
      const float inv = 1.0f / voxel_;
      std::unordered_map<std::uint64_t, std::size_t> cell;
      cell.reserve(pts.size());
      for (std::size_t i = 0; i < pts.size(); ++i) {
        const auto& p = pts[i];
        const auto ix = static_cast<std::int64_t>(std::floor(p.x() * inv)) & 0x1FFFFF;
        const auto iy = static_cast<std::int64_t>(std::floor(p.y() * inv)) & 0x1FFFFF;
        const auto iz = static_cast<std::int64_t>(std::floor(p.z() * inv)) & 0x1FFFFF;
        const std::uint64_t key = (static_cast<std::uint64_t>(ix) << 42) |
                                  (static_cast<std::uint64_t>(iy) << 21) |
                                  static_cast<std::uint64_t>(iz);
        if (cell.emplace(key, kf.pts.size()).second) {  // first point of a voxel represents it
          kf.pts.push_back(p);
          kf.intensity.push_back(intensity && i < intensity->size() ? (*intensity)[i] : 0.0f);
        }
      }
    }
    kfs_.push_back(std::move(kf));
  }

  // All keyframes in the world frame, one pose per keyframe (extra keyframes are skipped).
  void build(const std::vector<Eigen::Isometry3d>& poses, std::vector<Eigen::Vector3f>& out,
             std::vector<float>* out_intensity = nullptr) const {
    out.clear();
    if (out_intensity) out_intensity->clear();
    const std::size_t n = std::min(poses.size(), kfs_.size());
    for (std::size_t k = 0; k < n; ++k) {
      const Eigen::Isometry3f T = poses[k].cast<float>();
      for (std::size_t i = 0; i < kfs_[k].pts.size(); ++i) {
        out.push_back(T * kfs_[k].pts[i]);
        if (out_intensity) out_intensity->push_back(kfs_[k].intensity[i]);
      }
    }
  }

  std::size_t size() const { return kfs_.size(); }

 private:
  struct Keyframe {
    std::vector<Eigen::Vector3f> pts;
    std::vector<float> intensity;
  };
  float voxel_;
  std::vector<Keyframe> kfs_;
};

}  // namespace solid::slam
