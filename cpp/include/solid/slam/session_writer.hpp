// Writes a keyframe session in the LT-mapper / SC-LIO-SAM layout, so the
// output of any SOLiD front-end (ROS 1, ROS 2, Mac) can go straight into
// multi-session SLAM (https://github.com/gisbi-kim/lt-mapper):
//
//   <dir>/Scans/000000.pcd               keyframe cloud, sensor frame (binary PCD, x y z intensity)
//   <dir>/SCDs/000000.scd                Scan Context (20 x 60), read by LT-SLAM
//   <dir>/SOLiDs/000000.txt              SOLiD descriptor (line 1: R-SOLiD, line 2: A-SOLiD)
//   <dir>/singlesession_posegraph.g2o    keyframe poses + odometry edges (+ verified loops)
//   <dir>/optimized_poses.txt            KITTI poses after loop closing (if enabled)
//   <dir>/odom_poses.txt, times.txt      KITTI-format poses and keyframe stamps
//   <dir>/keyframes.txt, loops.csv       TUM poses and SOLiD loop candidates
//
// No PCL needed: the PCD and SCD files are written by hand.
#ifndef SOLID_SLAM_SESSION_WRITER_HPP
#define SOLID_SLAM_SESSION_WRITER_HPP

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "solid/descriptor.hpp"
#include "solid/slam/loop_detector.hpp"

namespace solid {
namespace slam {

// Scan Context as in SC-LIO-SAM / LT-SLAM: max height (+ lidar_height) per ring x sector bin.
inline Eigen::MatrixXd make_scan_context(const std::vector<Eigen::Vector3f>& pts,
                                         int num_ring = 20, int num_sector = 60,
                                         double max_radius = 80.0, double lidar_height = 2.0) {
  constexpr double kNoPoint = -1000.0;
  Eigen::MatrixXd desc = Eigen::MatrixXd::Constant(num_ring, num_sector, kNoPoint);
  for (const auto& p : pts) {
    const double z = p.z() + lidar_height;
    const double r = std::hypot(p.x(), p.y());
    if (r > max_radius) continue;
    double theta = std::atan2(p.y(), p.x()) * 180.0 / M_PI;
    if (theta < 0.0) theta += 360.0;
    const int ring = std::max(std::min(num_ring, static_cast<int>(std::ceil(r / max_radius * num_ring))), 1);
    const int sector =
        std::max(std::min(num_sector, static_cast<int>(std::ceil(theta / 360.0 * num_sector))), 1);
    double& bin = desc(ring - 1, sector - 1);
    if (bin < z) bin = z;
  }
  for (int i = 0; i < desc.size(); ++i)
    if (desc.data()[i] == kNoPoint) desc.data()[i] = 0.0;
  return desc;
}

// A loop that the back-end verified: pose of keyframe `to` in keyframe `from`.
struct VerifiedLoop {
  std::size_t from = 0, to = 0;
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
};

class SessionWriter {
 public:
  // Creates <dir> and its sub-folders. `dir` empty = disabled.
  explicit SessionWriter(std::string dir) : dir_(std::move(dir)) {
    if (dir_.empty()) return;
    namespace fs = std::filesystem;
    for (const char* sub : {"Scans", "SCDs", "SOLiDs"}) fs::create_directories(fs::path(dir_) / sub);
  }

  bool enabled() const { return !dir_.empty(); }
  const std::string& dir() const { return dir_; }

  // Writes the per-keyframe files right away so nothing large stays in memory.
  void add_keyframe(std::size_t id, const std::vector<Eigen::Vector3f>& pts,
                    const std::vector<float>& intensity, const solid::Descriptor& d) {
    if (!enabled()) return;
    const std::string name = index_name(id);
    write_pcd(dir_ + "/Scans/" + name + ".pcd", pts, intensity);

    std::ofstream scd(dir_ + "/SCDs/" + name + ".scd");
    const Eigen::IOFormat fmt(3, Eigen::DontAlignCols, " ", "\n");
    scd << make_scan_context(pts).format(fmt);

    std::ofstream sd(dir_ + "/SOLiDs/" + name + ".txt");
    sd << std::setprecision(9);
    for (int i = 0; i < d.rsolid.size(); ++i) sd << (i ? " " : "") << d.rsolid(i);
    sd << '\n';
    for (int i = 0; i < d.asolid.size(); ++i) sd << (i ? " " : "") << d.asolid(i);
    sd << '\n';
  }

  // Writes the pose graph, KITTI poses, stamps, TUM keyframes and loop candidates.
  // With loop closing, pass the optimized poses and the verified loops: the g2o file
  // then holds the optimized vertices and the loop edges, and optimized_poses.txt is added.
  bool finish(const LoopDetector& det, const std::vector<Eigen::Isometry3d>* optimized = nullptr,
              const std::vector<VerifiedLoop>* verified = nullptr) const {
    if (!enabled()) return false;
    const auto& kfs = det.keyframes();
    std::ofstream g2o(dir_ + "/singlesession_posegraph.g2o");
    std::ofstream kitti(dir_ + "/odom_poses.txt");
    std::ofstream times(dir_ + "/times.txt");
    if (!g2o || !kitti || !times) return false;
    g2o << std::fixed << std::setprecision(9);
    kitti << std::setprecision(9);
    times << std::fixed << std::setprecision(9);
    const bool opt = optimized && optimized->size() == kfs.size();
    for (const Keyframe& k : kfs) {
      g2o << "VERTEX_SE3:QUAT " << k.id << ' ' << pose_str(opt ? (*optimized)[k.id] : k.pose) << '\n';
      const Eigen::Matrix<double, 3, 4> m = k.pose.matrix().topRows<3>();
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) kitti << m(r, c) << ((r == 2 && c == 3) ? '\n' : ' ');
      times << k.stamp << '\n';
    }
    // Odometry edges only: SOLiD candidates are unverified, so they stay in loops.csv.
    static const char* kInfo = "1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1";
    for (std::size_t i = 1; i < kfs.size(); ++i) {
      g2o << "EDGE_SE3:QUAT " << kfs[i - 1].id << ' ' << kfs[i].id << ' '
          << pose_str(kfs[i - 1].pose.inverse() * kfs[i].pose) << ' ' << kInfo << '\n';
    }
    if (verified)
      for (const VerifiedLoop& l : *verified)
        g2o << "EDGE_SE3:QUAT " << l.from << ' ' << l.to << ' ' << pose_str(l.T) << ' ' << kInfo << '\n';
    if (opt) {
      std::ofstream o(dir_ + "/optimized_poses.txt");
      o << std::setprecision(9);
      for (const auto& T : *optimized) {
        const Eigen::Matrix<double, 3, 4> m = T.matrix().topRows<3>();
        for (int r = 0; r < 3; ++r)
          for (int c = 0; c < 4; ++c) o << m(r, c) << ((r == 2 && c == 3) ? '\n' : ' ');
      }
    }
    return det.save(dir_);
  }

 private:
  static std::string index_name(std::size_t id) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%06zu", id);
    return buf;
  }

  static std::string pose_str(const Eigen::Isometry3d& T) {
    const Eigen::Quaterniond q(T.rotation());
    std::ostringstream s;
    s << std::fixed << std::setprecision(9) << T.translation().x() << ' ' << T.translation().y()
      << ' ' << T.translation().z() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' '
      << q.w();
    return s.str();
  }

  // Binary PCD with fields x y z intensity (pcl::PointXYZI compatible).
  static void write_pcd(const std::string& path, const std::vector<Eigen::Vector3f>& pts,
                        const std::vector<float>& intensity) {
    std::ofstream f(path, std::ios::binary);
    f << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
      << "FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
      << "WIDTH " << pts.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
      << "POINTS " << pts.size() << "\nDATA binary\n";
    std::vector<float> buf(pts.size() * 4);
    for (std::size_t i = 0; i < pts.size(); ++i) {
      buf[4 * i + 0] = pts[i].x();
      buf[4 * i + 1] = pts[i].y();
      buf[4 * i + 2] = pts[i].z();
      buf[4 * i + 3] = i < intensity.size() ? intensity[i] : 0.0f;
    }
    f.write(reinterpret_cast<const char*>(buf.data()),
            static_cast<std::streamsize>(buf.size() * sizeof(float)));
  }

  std::string dir_;
};

}  // namespace slam
}  // namespace solid

#endif  // SOLID_SLAM_SESSION_WRITER_HPP
