// SOLiD core — descriptor database implementation.
#include "solid/database.hpp"

#include <algorithm>
#include <cmath>

#include "solid/extractor.hpp"  // Extractor::loop_similarity / pose_yaw_deg
#include "nanoflann.hpp"

namespace solid {

namespace {

// nanoflann dataset adaptor over a list of equal-length rsolid vectors.
struct RsolidAdaptor {
  const std::vector<Eigen::VectorXd>& pts;
  int dim;

  RsolidAdaptor(const std::vector<Eigen::VectorXd>& p, int d) : pts(p), dim(d) {}

  inline std::size_t kdtree_get_point_count() const { return pts.size(); }
  inline double kdtree_get_pt(const std::size_t idx, const std::size_t d) const {
    return pts[idx](static_cast<int>(d));
  }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, RsolidAdaptor>, RsolidAdaptor,
    -1 /* dynamic dim */, std::size_t>;

constexpr double kDeg2Rad = M_PI / 180.0;

}  // namespace

void Database::add(std::size_t id, const Descriptor& d) {
  ids_.push_back(id);
  rsolid_.push_back(d.rsolid);
  asolid_.push_back(d.asolid);
}

std::vector<Candidate> Database::query(const Descriptor& q) const {
  std::vector<Candidate> out;
  if (ids_.empty() || q.rsolid.size() == 0) return out;

  const int dim = static_cast<int>(q.rsolid.size());
  RsolidAdaptor adaptor(rsolid_, dim);
  KDTree index(dim, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  index.buildIndex();

  const std::size_t k =
      std::min<std::size_t>(static_cast<std::size_t>(cfg_.knn), ids_.size());
  std::vector<std::size_t> nn_idx(k);
  std::vector<double> nn_dist(k);
  nanoflann::KNNResultSet<double, std::size_t> result(k);
  result.init(nn_idx.data(), nn_dist.data());
  index.findNeighbors(result, q.rsolid.data(), nanoflann::SearchParams(10));

  out.reserve(k);
  for (std::size_t i = 0; i < result.size(); ++i) {
    const std::size_t slot = nn_idx[i];
    Candidate c;
    c.id = ids_[slot];
    c.score = Extractor::loop_similarity(q.rsolid, rsolid_[slot]);
    c.yaw_rad = Extractor::pose_yaw_deg(q.asolid, asolid_[slot]) * kDeg2Rad;
    if (c.score >= cfg_.min_similarity) out.push_back(c);
  }

  std::sort(out.begin(), out.end(),
            [](const Candidate& a, const Candidate& b) {
              return a.score > b.score;
            });
  return out;
}

}  // namespace solid
