// SOLiD core — descriptor extractor.
//
// Modality-neutral: input is a plain list of 3D points (+ optional per-point
// weights), so LiDAR (PointXYZI/Ouster), 4D radar (with RCS/power as weight)
// and GS-depth back-projected clouds all feed the same code path.
#ifndef SOLID_EXTRACTOR_HPP
#define SOLID_EXTRACTOR_HPP

#include <vector>
#include <Eigen/Dense>

#include "solid/config.hpp"
#include "solid/descriptor.hpp"

namespace solid {

class Extractor {
 public:
  explicit Extractor(const Config& cfg) : cfg_(cfg) {}

  const Config& config() const { return cfg_; }

  // Build a SOLiD descriptor from raw points.
  //   pts    : points in the sensor/body frame (meters).
  //   weight : optional per-point weight, same length as pts. Used only when
  //            cfg.use_weight is true (e.g. radar RCS/power). Ignored otherwise.
  // Preprocessing (range gating + optional voxel downsample) is applied
  // internally according to the Config.
  Descriptor extract(const std::vector<Eigen::Vector3f>& pts,
                     const std::vector<float>* weight = nullptr) const;

  // --- exposed for reuse / testing (mirror the reference SOLiDModule API) ---

  // Cosine similarity between two range signatures (higher = more similar).
  // Name kept snake_case to match the Python API (solid.Extractor.loop_similarity).
  static double loop_similarity(const Eigen::VectorXd& rsolid_query,
                                const Eigen::VectorXd& rsolid_candidate);

  // Relative yaw (degrees) recovered by circular-shift L1 matching of the
  // angular signatures. Matches the Python API (Extractor.pose_yaw_deg).
  static double pose_yaw_deg(const Eigen::VectorXd& asolid_query,
                             const Eigen::VectorXd& asolid_candidate);

 private:
  // range-gate (min_range < dist < max_range) and optional voxel downsample,
  // carrying weights when use_weight is set.
  void preprocess(const std::vector<Eigen::Vector3f>& pts_in,
                  const std::vector<float>* weight_in,
                  std::vector<Eigen::Vector3f>& pts_out,
                  std::vector<float>& weight_out) const;

  Config cfg_;
};

}  // namespace solid

#endif  // SOLID_EXTRACTOR_HPP
