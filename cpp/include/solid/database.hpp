// SOLiD core — descriptor database (KD-tree retrieval).
//
// Stores rsolid signatures and retrieves nearest candidates for a query
// descriptor via a nanoflann KD-tree (L2), then re-scores each hit with the
// SOLiD cosine similarity and recovers a yaw guess from asolid. Mirrors the
// KNNSearch + distBtnSOLiDs stage of Distributed-SOLiD-SLAM's mapFusion.
#ifndef SOLID_DATABASE_HPP
#define SOLID_DATABASE_HPP

#include <cstddef>
#include <vector>
#include <Eigen/Dense>

#include "solid/config.hpp"
#include "solid/descriptor.hpp"

namespace solid {

class Database {
 public:
  explicit Database(const Config& cfg) : cfg_(cfg) {}

  // Insert a descriptor under a caller-assigned id (e.g. keyframe index).
  void add(std::size_t id, const Descriptor& d);

  // Retrieve up to cfg.knn nearest candidates for `query`, sorted by cosine
  // similarity (descending) and filtered by cfg.min_similarity. Each candidate
  // carries its id, similarity score, and estimated relative yaw (radians).
  // Returns empty if the database is empty.
  std::vector<Candidate> query(const Descriptor& query) const;

  std::size_t size() const { return ids_.size(); }
  bool empty() const { return ids_.empty(); }
  const Config& config() const { return cfg_; }

 private:
  Config cfg_;
  std::vector<std::size_t> ids_;
  std::vector<Eigen::VectorXd> rsolid_;  // KD-tree keys (length num_range)
  std::vector<Eigen::VectorXd> asolid_;  // kept for yaw recovery
};

}  // namespace solid

#endif  // SOLID_DATABASE_HPP
