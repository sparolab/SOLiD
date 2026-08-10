// SOLiD core — descriptor and candidate types.
#ifndef SOLID_DESCRIPTOR_HPP
#define SOLID_DESCRIPTOR_HPP

#include <cstddef>
#include <Eigen/Dense>

namespace solid {

// A SOLiD descriptor is two decoupled signatures:
//   rsolid : range signature   (length num_range)  — rotation invariant, KD-tree key
//   asolid : angular signature (length num_angle)  — used to recover relative yaw
//
// In the original SOLiDModule these were packed into one VectorXd as
// [rsolid ; asolid]; here they are kept separate for clarity. `combined()`
// reproduces the original packed layout when needed.
struct Descriptor {
  Eigen::VectorXd rsolid;
  Eigen::VectorXd asolid;

  bool empty() const { return rsolid.size() == 0 && asolid.size() == 0; }

  Eigen::VectorXd combined() const {
    Eigen::VectorXd v(rsolid.size() + asolid.size());
    v.head(rsolid.size()) = rsolid;
    v.tail(asolid.size()) = asolid;
    return v;
  }
};

// A retrieval result from the descriptor Database.
struct Candidate {
  std::size_t id = 0;      // caller-assigned id passed to Database::add()
  double score = 0.0;      // cosine similarity of rsolid in [-1, 1] (higher = better)
  double yaw_rad = 0.0;    // relative yaw estimate (query w.r.t. candidate), radians
};

}  // namespace solid

#endif  // SOLID_DESCRIPTOR_HPP
