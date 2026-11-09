// Loads solid_slam::Params from a config/*.yaml file (yaml-cpp).
#ifndef SOLID_SLAM_CONFIG_YAML_HPP
#define SOLID_SLAM_CONFIG_YAML_HPP

#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "solid/slam/loop_detector.hpp"

namespace solid {
namespace slam {

namespace detail {
template <typename T>
void get(const YAML::Node& n, const char* key, T& out) {
  if (n && n[key]) out = n[key].as<T>();
}
}  // namespace detail

inline Params load_params(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    throw std::runtime_error("cannot read config '" + path + "': " + e.what());
  }
  Params p;
  using detail::get;
  const YAML::Node s = root["solid"];
  get(s, "fov_up", p.solid.fov_up);
  get(s, "fov_down", p.solid.fov_down);
  get(s, "num_angle", p.solid.num_angle);
  get(s, "num_range", p.solid.num_range);
  get(s, "num_height", p.solid.num_height);
  get(s, "min_range", p.solid.min_range);
  get(s, "max_range", p.solid.max_range);
  get(s, "voxel_size", p.solid.voxel_size);
  get(s, "use_weight", p.solid.use_weight);
  get(s, "knn", p.solid.knn);
  get(s, "min_similarity", p.solid.min_similarity);

  const YAML::Node k = root["keyframe"];
  get(k, "dist", p.keyframe_dist);
  get(k, "angle_deg", p.keyframe_angle_deg);

  const YAML::Node m = root["submap"];
  get(m, "scans", p.submap_scans);
  get(m, "max_dist", p.submap_max_dist);

  const YAML::Node l = root["loop"];
  get(l, "exclude_recent", p.exclude_recent);
  get(l, "min_score", p.min_score);
  get(l, "max_candidates", p.max_candidates);

  const YAML::Node i = root["input"];
  get(i, "cloud_in_world", p.cloud_in_world);
  get(i, "weight_field", p.weight_field);
  // input.extrinsic: {t: [x, y, z], R: [9 values, row-major]} = sensor pose in the odometry body frame
  if (i && i["extrinsic"]) {
    const YAML::Node e = i["extrinsic"];
    if (e["t"]) {
      const auto t = e["t"].as<std::vector<double>>();
      if (t.size() == 3) p.extrinsic.translation() << t[0], t[1], t[2];
    }
    if (e["R"]) {
      const auto r = e["R"].as<std::vector<double>>();
      if (r.size() == 9) {
        Eigen::Matrix3d R;
        R << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];
        p.extrinsic.linear() = Eigen::Quaterniond(R).normalized().toRotationMatrix();
      }
    }
  }
  return p;
}

}  // namespace slam
}  // namespace solid

#endif  // SOLID_SLAM_CONFIG_YAML_HPP
