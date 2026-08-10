// Self-contained gtest unit tests for solid_core (no reference dependency).
// Built only when GTest is available (see CMakeLists.txt).
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "solid/database.hpp"
#include "solid/extractor.hpp"

namespace {

// A synthetic ring of points at fixed radius/height, plus noise-free structure.
std::vector<Eigen::Vector3f> ring(float radius, float z, int n) {
  std::vector<Eigen::Vector3f> pts;
  for (int i = 0; i < n; ++i) {
    const float a = 2.0f * static_cast<float>(M_PI) * i / n;
    pts.emplace_back(radius * std::cos(a), radius * std::sin(a), z);
  }
  return pts;
}

}  // namespace

TEST(Extractor, DescriptorDimensions) {
  solid::Config cfg;
  solid::Extractor ext(cfg);
  const auto d = ext.extract(ring(10.0f, 0.0f, 2000));
  EXPECT_EQ(d.rsolid.size(), cfg.num_range);
  EXPECT_EQ(d.asolid.size(), cfg.num_angle);
}

TEST(Extractor, IdenticalCloudsAreMaximallySimilar) {
  solid::Config cfg;
  solid::Extractor ext(cfg);
  auto pts = ring(15.0f, 0.5f, 3000);
  const auto a = ext.extract(pts);
  const auto b = ext.extract(pts);
  EXPECT_NEAR(solid::Extractor::loop_similarity(a.rsolid, b.rsolid), 1.0, 1e-9);
}

TEST(Database, RetrievesSortedByScore) {
  solid::Config cfg;
  solid::Extractor ext(cfg);
  solid::Database db(cfg);
  db.add(1, ext.extract(ring(10.0f, 0.0f, 2000)));
  db.add(2, ext.extract(ring(30.0f, 0.0f, 2000)));

  const auto q = ext.extract(ring(10.0f, 0.0f, 2000));  // matches id 1
  const auto hits = db.query(q);
  ASSERT_FALSE(hits.empty());
  EXPECT_EQ(hits.front().id, 1u);
  for (std::size_t i = 1; i < hits.size(); ++i) {
    EXPECT_GE(hits[i - 1].score, hits[i].score);
  }
}

TEST(Config, RadarProfileWeightingRuns) {
  solid::Config cfg;
  cfg.use_weight = true;
  cfg.voxel_size = 0.0f;   // radar profile: no downsample
  cfg.num_height = 8;
  solid::Extractor ext(cfg);
  auto pts = ring(20.0f, 0.0f, 500);
  std::vector<float> w(pts.size(), 2.0f);  // uniform RCS-like weight
  const auto d = ext.extract(pts, &w);
  EXPECT_EQ(d.rsolid.size(), cfg.num_range);
  EXPECT_GT(d.rsolid.norm(), 0.0);
}
