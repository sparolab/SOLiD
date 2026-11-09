#include "solid/multi_session/session.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

namespace solid {
namespace ms {

namespace {

Eigen::Isometry3d make_pose(double x, double y, double z, double qx, double qy, double qz,
                            double qw) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
  T.translation() << x, y, z;
  return T;
}

Eigen::VectorXd read_vector(const std::string& line) {
  std::istringstream ss(line);
  std::vector<double> v;
  double x;
  while (ss >> x) v.push_back(x);
  return Eigen::Map<Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
}

std::string index_name(int i) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%06d", i);
  return buf;
}

}  // namespace

Session::Session(const std::string& dir, const std::string& name, double scan_voxel)
    : dir_(dir), name_(name), voxel_(scan_voxel) {
  std::ifstream g2o(dir_ + "/singlesession_posegraph.g2o");
  if (!g2o) throw std::runtime_error("no singlesession_posegraph.g2o in " + dir_);
  std::map<int, Eigen::Isometry3d> nodes;
  std::string line;
  while (std::getline(g2o, line)) {
    std::istringstream ss(line);
    std::string tag;
    ss >> tag;
    double x, y, z, qx, qy, qz, qw;
    if (tag == "VERTEX_SE3:QUAT") {
      int id;
      ss >> id >> x >> y >> z >> qx >> qy >> qz >> qw;
      nodes[id] = make_pose(x, y, z, qx, qy, qz, qw);
    } else if (tag == "EDGE_SE3:QUAT") {
      OdomEdge e;
      ss >> e.from >> e.to >> x >> y >> z >> qx >> qy >> qz >> qw;
      e.T = make_pose(x, y, z, qx, qy, qz, qw);
      edges_.push_back(e);
    }
  }
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    if (!nodes.count(static_cast<int>(i)))
      throw std::runtime_error("g2o vertices are not 0..N-1 in " + dir_);
    poses_.push_back(nodes[static_cast<int>(i)]);
  }

  for (std::size_t i = 0; i < poses_.size(); ++i) {
    std::ifstream f(dir_ + "/SOLiDs/" + index_name(static_cast<int>(i)) + ".txt");
    if (!f) throw std::runtime_error("missing SOLiDs/" + index_name(static_cast<int>(i)) + ".txt in " + dir_);
    std::string r, a;
    std::getline(f, r);
    std::getline(f, a);
    solid::Descriptor d;
    d.rsolid = read_vector(r);
    d.asolid = read_vector(a);
    descs_.push_back(d);
  }

  std::ifstream t(dir_ + "/times.txt");
  double s;
  while (t >> s) stamps_.push_back(s);
}

Cloud::ConstPtr Session::scan(int i) const {
  auto it = cache_.find(i);
  if (it != cache_.end()) return it->second;
  Cloud::Ptr raw(new Cloud), out(new Cloud);
  const std::string path = dir_ + "/Scans/" + index_name(i) + ".pcd";
  if (pcl::io::loadPCDFile<PointT>(path, *raw) != 0) throw std::runtime_error("cannot read " + path);
  if (voxel_ > 0.0) {
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(voxel_, voxel_, voxel_);
    vg.setInputCloud(raw);
    vg.filter(*out);
  } else {
    out = raw;
  }
  cache_[i] = out;
  return out;
}

Cloud::Ptr Session::submap(int i, int n) const {
  Cloud::Ptr out(new Cloud);
  const Eigen::Isometry3d inv = poses_[i].inverse();
  for (int k = std::max(0, i - n); k <= std::min<int>(static_cast<int>(size()) - 1, i + n); ++k) {
    Cloud tmp;
    pcl::transformPointCloud(*scan(k), tmp, (inv * poses_[k]).matrix().cast<float>());
    *out += tmp;
  }
  return out;
}

}  // namespace ms
}  // namespace solid
