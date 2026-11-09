// Loads solid_slam::Params from a slam/config/*.yaml file (yaml-cpp).
#ifndef SOLID_SLAM_CONFIG_YAML_HPP
#define SOLID_SLAM_CONFIG_YAML_HPP

#include <stdexcept>
#include <string>

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
  return p;
}

}  // namespace slam
}  // namespace solid

#endif  // SOLID_SLAM_CONFIG_YAML_HPP
