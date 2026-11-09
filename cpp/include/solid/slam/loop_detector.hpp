// SOLiD loop detector (solid::slam, no ROS dependency). The ROS 1 node, the ROS 2
// node and the Python package (solid.slam, used by slam/mac) all run this class.
//
// Feed it one (pose, cloud) pair per scan. It picks keyframes by travelled
// distance / rotation, builds a SOLiD descriptor for each keyframe and returns
// loop candidates against keyframes that are at least `exclude_recent`
// keyframes old. Registration and pose-graph optimization are left to the SLAM
// that consumes the candidates.
#ifndef SOLID_SLAM_LOOP_DETECTOR_HPP
#define SOLID_SLAM_LOOP_DETECTOR_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "solid/config.hpp"
#include "solid/database.hpp"
#include "solid/descriptor.hpp"
#include "solid/extractor.hpp"

namespace solid {
namespace slam {

struct Params {
  solid::Config solid;          // descriptor + database (slam/config/*.yaml `solid:`)

  double keyframe_dist = 1.0;       // m, new keyframe after this much translation
  double keyframe_angle_deg = 10.0; // deg, ... or this much rotation

  int exclude_recent = 50;      // skip the last N keyframes when searching
  double min_score = 0.99;      // min cosine similarity of R-SOLiD
  int max_candidates = 1;       // candidates reported per keyframe

  bool cloud_in_world = false;  // true if the input cloud is in the odometry frame
  std::string weight_field;     // per-point weight field (e.g. radar power), "" = none
};

struct Keyframe {
  std::size_t id = 0;
  double stamp = 0.0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
};

struct Loop {
  std::size_t query_id = 0;
  std::size_t match_id = 0;
  double score = 0.0;    // cosine similarity, higher is better
  double yaw_rad = 0.0;  // SOLiD yaw of the query relative to the match
};

// A read-only view over an interleaved point buffer (sensor_msgs/PointCloud2 layout).
struct CloudView {
  const std::uint8_t* data = nullptr;
  std::size_t num_points = 0;
  std::size_t point_step = 0;
  int offset_x = -1, offset_y = -1, offset_z = -1;
  std::uint8_t datatype_xyz = 7;  // PointField::FLOAT32
  int offset_w = -1;              // -1: no weight field
  std::uint8_t datatype_w = 7;
  int offset_i = -1;              // -1: no intensity field (only saved with the session)
  std::uint8_t datatype_i = 7;
};

// Reads one value of a PointField datatype (1=INT8 ... 8=FLOAT64).
inline double read_field(const std::uint8_t* p, std::uint8_t datatype) {
  switch (datatype) {
    case 1: { std::int8_t v; std::memcpy(&v, p, 1); return v; }
    case 2: { std::uint8_t v; std::memcpy(&v, p, 1); return v; }
    case 3: { std::int16_t v; std::memcpy(&v, p, 2); return v; }
    case 4: { std::uint16_t v; std::memcpy(&v, p, 2); return v; }
    case 5: { std::int32_t v; std::memcpy(&v, p, 4); return v; }
    case 6: { std::uint32_t v; std::memcpy(&v, p, 4); return v; }
    case 7: { float v; std::memcpy(&v, p, 4); return v; }
    case 8: { double v; std::memcpy(&v, p, 8); return v; }
    default: return 0.0;
  }
}

// Converts a cloud to sensor-frame points (+ weights, + intensity if asked).
// Drops non-finite points. `world_to_body` is applied when the cloud is in the
// odometry frame.
inline void to_points(const CloudView& c, const Eigen::Isometry3d* world_to_body,
                      std::vector<Eigen::Vector3f>& pts, std::vector<float>& w,
                      std::vector<float>* intensity = nullptr) {
  pts.clear();
  w.clear();
  if (intensity) intensity->clear();
  pts.reserve(c.num_points);
  if (c.offset_w >= 0) w.reserve(c.num_points);
  const Eigen::Isometry3f T =
      world_to_body ? world_to_body->cast<float>() : Eigen::Isometry3f::Identity();
  for (std::size_t i = 0; i < c.num_points; ++i) {
    const std::uint8_t* p = c.data + i * c.point_step;
    Eigen::Vector3f q(static_cast<float>(read_field(p + c.offset_x, c.datatype_xyz)),
                      static_cast<float>(read_field(p + c.offset_y, c.datatype_xyz)),
                      static_cast<float>(read_field(p + c.offset_z, c.datatype_xyz)));
    if (!q.allFinite()) continue;
    pts.push_back(world_to_body ? Eigen::Vector3f(T * q) : q);
    if (c.offset_w >= 0) w.push_back(static_cast<float>(read_field(p + c.offset_w, c.datatype_w)));
    if (intensity)
      intensity->push_back(
          c.offset_i >= 0 ? static_cast<float>(read_field(p + c.offset_i, c.datatype_i)) : 0.0f);
  }
}

class LoopDetector {
 public:
  explicit LoopDetector(const Params& p)
      : params_(p), extractor_(p.solid), database_(p.solid) {}

  const Params& params() const { return params_; }
  const std::vector<Keyframe>& keyframes() const { return keyframes_; }
  const std::vector<Loop>& loops() const { return loops_; }
  // Descriptor of the latest keyframe (before normalization).
  const solid::Descriptor& last_descriptor() const { return last_; }

  // True if `pose` is far enough from the last keyframe to start a new one.
  bool is_keyframe(const Eigen::Isometry3d& pose) const {
    if (keyframes_.empty()) return true;
    const Eigen::Isometry3d d = keyframes_.back().pose.inverse() * pose;
    const double angle = Eigen::AngleAxisd(d.rotation()).angle() * 180.0 / M_PI;
    return d.translation().norm() >= params_.keyframe_dist ||
           angle >= params_.keyframe_angle_deg;
  }

  // Adds a keyframe from sensor-frame points and returns its loop candidates.
  std::vector<Loop> add_keyframe(double stamp, const Eigen::Isometry3d& pose,
                                 const std::vector<Eigen::Vector3f>& pts,
                                 const std::vector<float>* weights = nullptr) {
    Keyframe kf;
    kf.id = keyframes_.size();
    kf.stamp = stamp;
    kf.pose = pose;
    keyframes_.push_back(kf);

    const bool weighted = params_.solid.use_weight && weights && weights->size() == pts.size();
    solid::Descriptor d = extractor_.extract(pts, weighted ? weights : nullptr);
    last_ = d;
    // L2-normalized R-SOLiD makes the KD-tree distance monotonic in cosine similarity.
    const double n = d.rsolid.norm();
    if (n > 0.0) d.rsolid /= n;

    std::vector<Loop> out;
    if (!database_.empty()) {
      for (const solid::Candidate& c : database_.query(d)) {
        if (c.score < params_.min_score) break;  // sorted by score
        out.push_back({kf.id, c.id, c.score, c.yaw_rad});
        if (static_cast<int>(out.size()) >= params_.max_candidates) break;
      }
    }
    loops_.insert(loops_.end(), out.begin(), out.end());

    // A keyframe becomes searchable once it is `exclude_recent` keyframes old.
    pending_.push_back({kf.id, d});
    while (static_cast<int>(pending_.size()) > params_.exclude_recent) {
      database_.add(pending_.front().first, pending_.front().second);
      pending_.pop_front();
    }
    return out;
  }

  // Writes <dir>/keyframes.txt (TUM: stamp x y z qx qy qz qw) and <dir>/loops.csv.
  bool save(const std::string& dir) const {
    std::ofstream kf(dir + "/keyframes.txt");
    std::ofstream lp(dir + "/loops.csv");
    if (!kf || !lp) return false;
    kf << std::fixed << std::setprecision(9);
    for (const Keyframe& k : keyframes_) {
      const Eigen::Quaterniond q(k.pose.rotation());
      const Eigen::Vector3d t = k.pose.translation();
      kf << k.stamp << ' ' << t.x() << ' ' << t.y() << ' ' << t.z() << ' ' << q.x() << ' '
         << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
    }
    lp << "query_id,match_id,query_stamp,match_stamp,score,yaw_rad\n" << std::fixed
       << std::setprecision(6);
    for (const Loop& l : loops_) {
      lp << l.query_id << ',' << l.match_id << ',' << keyframes_[l.query_id].stamp << ','
         << keyframes_[l.match_id].stamp << ',' << l.score << ',' << l.yaw_rad << '\n';
    }
    return true;
  }

 private:
  Params params_;
  solid::Extractor extractor_;
  solid::Database database_;
  std::vector<Keyframe> keyframes_;
  std::vector<Loop> loops_;
  std::deque<std::pair<std::size_t, solid::Descriptor>> pending_;
  solid::Descriptor last_;
};

}  // namespace slam
}  // namespace solid

#endif  // SOLID_SLAM_LOOP_DETECTOR_HPP
