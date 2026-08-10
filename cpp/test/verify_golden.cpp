// Regression check: solid_core output vs frozen golden descriptors.
//
// The golden files (test/data/golden/golden_{313,314,315}.txt) were snapshotted
// from the original SOLiDModule reference and are the ground truth. solid_core
// must reproduce them exactly. This replaces the old compile-the-reference test
// now that cpp/ is the library itself.
//
// No gtest dependency: prints a report and returns non-zero on any failure.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "solid/database.hpp"
#include "solid/extractor.hpp"

using solid::Candidate;
using solid::Config;
using solid::Database;
using solid::Descriptor;
using solid::Extractor;

namespace {

int g_fail = 0;
constexpr double kTol = 1e-9;

void check(bool cond, const std::string& msg) {
  std::printf("  [%s] %s\n", cond ? "ok  " : "FAIL", msg.c_str());
  if (!cond) ++g_fail;
}

// Minimal binary-PCD reader ("FIELDS x y z / TYPE F / DATA binary"), VTK-free.
pcl::PointCloud<pcl::PointXYZ>::Ptr load(const std::string& path) {
  auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    std::printf("  [FAIL] cannot open %s\n", path.c_str());
    ++g_fail;
    return cloud;
  }
  std::size_t n = 0;
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ss(line);
    std::string key;
    ss >> key;
    if (key == "POINTS") {
      ss >> n;
    } else if (key == "DATA") {
      std::string fmt;
      ss >> fmt;
      if (fmt != "binary") {
        std::printf("  [FAIL] %s: only binary PCD supported\n", path.c_str());
        ++g_fail;
        return cloud;
      }
      break;
    }
  }
  cloud->points.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    float xyz[3];
    in.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (!in) {
      ++g_fail;
      break;
    }
    cloud->points.emplace_back(xyz[0], xyz[1], xyz[2]);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  return cloud;
}

std::vector<Eigen::Vector3f> toVec(const pcl::PointCloud<pcl::PointXYZ>& c) {
  std::vector<Eigen::Vector3f> v;
  v.reserve(c.points.size());
  for (const auto& p : c.points) v.emplace_back(p.x, p.y, p.z);
  return v;
}

// Golden format: "NR NA" then NR rsolid values then NA asolid values.
Descriptor loadGolden(const std::string& path) {
  Descriptor d;
  std::ifstream in(path);
  if (!in) {
    std::printf("  [FAIL] cannot open golden %s\n", path.c_str());
    ++g_fail;
    return d;
  }
  int nr = 0, na = 0;
  in >> nr >> na;
  d.rsolid.resize(nr);
  d.asolid.resize(na);
  for (int i = 0; i < nr; ++i) in >> d.rsolid(i);
  for (int i = 0; i < na; ++i) in >> d.asolid(i);
  return d;
}

double maxAbsDiff(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
  if (a.size() != b.size()) return std::numeric_limits<double>::infinity();
  return (a - b).cwiseAbs().maxCoeff();
}

}  // namespace

int main() {
  const std::string data = SOLID_DATA_DIR;
  std::printf("solid_core vs golden regression check\n  data dir: %s\n\n",
              data.c_str());

  auto c313 = load(data + "/pcd/000313.pcd");
  auto c314 = load(data + "/pcd/000314.pcd");
  auto c315 = load(data + "/pcd/000315.pcd");
  const Descriptor g313 = loadGolden(data + "/golden/golden_313.txt");
  const Descriptor g314 = loadGolden(data + "/golden/golden_314.txt");
  const Descriptor g315 = loadGolden(data + "/golden/golden_315.txt");
  if (g_fail) return 1;

  Config cfg;  // defaults == original SOLiDModule constants
  Extractor ext(cfg);
  const Descriptor d313 = ext.extract(toVec(*c313));
  const Descriptor d314 = ext.extract(toVec(*c314));
  const Descriptor d315 = ext.extract(toVec(*c315));

  std::printf("[1] descriptor vs golden (max abs diff, tol=%.0e)\n", kTol);
  struct Row { const char* name; const Descriptor* g; const Descriptor* d; };
  const Row rows[] = {{"313", &g313, &d313},
                      {"314", &g314, &d314},
                      {"315", &g315, &d315}};
  for (const auto& r : rows) {
    const double dr = maxAbsDiff(r.d->rsolid, r.g->rsolid);
    const double da = maxAbsDiff(r.d->asolid, r.g->asolid);
    std::printf("    scan %s : rsolid=%.3e asolid=%.3e\n", r.name, dr, da);
    check(dr < kTol, std::string("rsolid matches golden (scan ") + r.name + ")");
    check(da < kTol, std::string("asolid matches golden (scan ") + r.name + ")");
  }

  std::printf("\n[2] loop similarity consistency\n");
  const double s14 = Extractor::loop_similarity(d313.rsolid, d314.rsolid);
  const double s15 = Extractor::loop_similarity(d313.rsolid, d315.rsolid);
  const double gs14 = Extractor::loop_similarity(g313.rsolid, g314.rsolid);
  const double gs15 = Extractor::loop_similarity(g313.rsolid, g315.rsolid);
  std::printf("    313-314: core=%.9f golden=%.9f | 313-315: core=%.9f golden=%.9f\n",
              s14, gs14, s15, gs15);
  check(std::abs(s14 - gs14) < kTol, "loop_similarity 313-314 matches golden");
  check(std::abs(s15 - gs15) < kTol, "loop_similarity 313-315 matches golden");

  std::printf("\n[3] Database retrieval\n");
  Database db(cfg);
  db.add(314, d314);
  db.add(315, d315);
  const std::vector<Candidate> hits = db.query(d313);
  check(!hits.empty(), "database returns candidates");
  if (!hits.empty()) {
    const std::size_t winner = (s14 >= s15) ? 314u : 315u;
    std::printf("    top id=%zu score=%.9f (expected winner=%zu)\n",
                hits.front().id, hits.front().score, winner);
    check(hits.front().id == winner, "top candidate == best-similarity scan");
    check(hits.size() == 2, "both descriptors retrieved (knn)");
  }

  std::printf("\n==== %s (%d failure%s) ====\n", g_fail ? "FAILED" : "PASSED",
              g_fail, g_fail == 1 ? "" : "s");
  return g_fail ? 1 : 0;
}
