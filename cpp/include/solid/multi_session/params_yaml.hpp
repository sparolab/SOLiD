// Loads AlignParams from multi_session_slam/config/*.yaml (yaml-cpp).
#ifndef SOLID_MULTI_SESSION_PARAMS_YAML_HPP
#define SOLID_MULTI_SESSION_PARAMS_YAML_HPP

#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "solid/multi_session/aligner.hpp"

namespace solid {
namespace ms {

inline AlignParams load_params(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    throw std::runtime_error("cannot read config '" + path + "': " + e.what());
  }
  AlignParams p;
  auto get = [](const YAML::Node& n, const char* k, auto& out) {
    if (n && n[k]) out = n[k].as<std::decay_t<decltype(out)>>();
  };
  const YAML::Node r = root["retrieval"];
  get(r, "min_score", p.min_score);
  get(r, "knn", p.solid.knn);
  get(r, "candidate_stride", p.candidate_stride);
  get(r, "max_verifications", p.max_verifications);
  const YAML::Node v = root["verification"];
  get(v, "submap_keyframes", p.submap_keyframes);
  get(v, "scan_voxel", p.scan_voxel);
  get(v, "icp_voxel", p.icp_voxel);
  get(v, "icp_max_dist", p.icp_max_dist);
  get(v, "icp_max_fitness", p.icp_max_fitness);
  const YAML::Node c = root["consensus"];
  get(c, "trans", p.consensus_trans);
  get(c, "rot_deg", p.consensus_rot_deg);
  get(c, "min_inliers", p.min_inliers);
  const YAML::Node g = root["graph"];
  get(g, "odom_sigma_trans", p.odom_sigma_trans);
  get(g, "odom_sigma_rot_deg", p.odom_sigma_rot_deg);
  get(g, "loop_sigma_trans", p.loop_sigma_trans);
  get(g, "loop_sigma_rot_deg", p.loop_sigma_rot_deg);
  get(g, "iterations", p.optimizer_iterations);
  return p;
}

}  // namespace ms
}  // namespace solid

#endif  // SOLID_MULTI_SESSION_PARAMS_YAML_HPP
