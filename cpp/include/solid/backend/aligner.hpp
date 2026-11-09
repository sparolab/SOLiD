// SOLiD multi-session aligner (ROS-free).
//
//   1. Retrieval : every query keyframe searches the central session's SOLiD database.
//   2. Check     : each candidate is registered with GICP (keyframe submaps, SOLiD yaw as
//                  the initial guess) and kept if the fit is good.
//   3. Consensus : each verified loop implies one central<-query transform; the largest
//                  group of loops that agree on it is kept, the rest are outliers.
//   4. Graph     : all sessions go into one GTSAM pose graph (odometry + inlier loops,
//                  Cauchy kernel), initialized with the consensus transforms.
#ifndef SOLID_BACKEND_ALIGNER_HPP
#define SOLID_BACKEND_ALIGNER_HPP

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "solid/config.hpp"
#include "solid/backend/session.hpp"

namespace solid {
namespace backend {

struct AlignParams {
  solid::Config solid;          // knn used for retrieval
  double min_score = 0.99;      // min R-SOLiD cosine similarity of a candidate
  int candidate_stride = 1;     // test every N-th query keyframe
  int max_verifications = 100;  // GICP-check at most this many candidates (highest scores first)

  int submap_keyframes = 5;     // +- keyframes merged around each loop keyframe
  double scan_voxel = 0.4;      // m, voxel of the loaded scans
  double icp_voxel = 1.0;       // m, voxel of the submaps fed to GICP
  double icp_max_dist = 2.0;    // m, GICP max correspondence distance
  double icp_max_fitness = 0.3; // mean squared residual (m^2) to accept a loop

  double consensus_trans = 3.0;    // m, loops agreeing on the session transform
  double consensus_rot_deg = 5.0;  // deg
  int min_inliers = 3;             // fewer agreeing loops: the session stays unaligned

  double odom_sigma_trans = 0.1, odom_sigma_rot_deg = 1.0;
  double loop_sigma_trans = 0.5, loop_sigma_rot_deg = 3.0;
  int optimizer_iterations = 100;
};

struct InterLoop {
  int session = 0;             // query session index
  int central_kf = 0, query_kf = 0;
  double score = 0.0;          // SOLiD similarity
  double fitness = 0.0;        // GICP fitness (m^2), < 0 if registration failed
  bool verified = false;       // passed GICP
  bool inlier = false;         // agrees with the consensus transform
  Eigen::Isometry3d T_cq = Eigen::Isometry3d::Identity();  // query kf in central kf frame
};

struct SessionResult {
  bool aligned = false;
  int inliers = 0;
  Eigen::Isometry3d T_central_session = Eigen::Isometry3d::Identity();  // before the graph
  std::vector<Eigen::Isometry3d> poses;  // optimized, in the central session's frame
};

struct AlignResult {
  std::vector<SessionResult> sessions;  // index 0 = central
  std::vector<InterLoop> loops;
  double seconds = 0.0;
};

// All sessions' keyframe scans in the central frame, voxel-downsampled.
// Point intensity = session index (0 = central), handy for coloring.
Cloud::Ptr build_map(const std::vector<std::shared_ptr<Session>>& sessions, const AlignResult& r,
                     double voxel, int keyframe_stride = 1);

class Aligner {
 public:
  explicit Aligner(const AlignParams& p) : p_(p) {}

  // sessions[0] is the central session; the others are aligned to it.
  AlignResult align(const std::vector<std::shared_ptr<Session>>& sessions) const;

  // Writes <dir>/<name>_aligned.txt (KITTI) and _aligned_tum.txt per session, and
  // <dir>/inter_session_loops.csv.
  static bool save(const std::string& dir, const std::vector<std::shared_ptr<Session>>& sessions,
                   const AlignResult& r);

 private:
  std::vector<InterLoop> find_loops(const Session& central, const Session& query, int qidx) const;
  bool verify(const Session& central, const Session& query, InterLoop& l, double yaw) const;
  AlignParams p_;
};

}  // namespace backend
}  // namespace solid

#endif  // SOLID_BACKEND_ALIGNER_HPP
