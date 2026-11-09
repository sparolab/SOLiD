// Loads AlignParams from multi_session_config/*.yaml (yaml-cpp).
#ifndef SOLID_BACKEND_PARAMS_YAML_HPP
#define SOLID_BACKEND_PARAMS_YAML_HPP

#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "solid/backend/aligner.hpp"
#include "solid/backend/loop_closer.hpp"

namespace solid {
namespace backend {

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

// `loop_closing:` section of a config/ sensor YAML (single-session loop closing).
inline LoopClosingParams load_loop_closing(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception& e) {
    throw std::runtime_error("cannot read config '" + path + "': " + e.what());
  }
  LoopClosingParams p;
  const YAML::Node n = root["loop_closing"];
  auto get = [&n](const char* k, auto& out) {
    if (n && n[k]) out = n[k].as<std::decay_t<decltype(out)>>();
  };
  get("enable", p.enable);
  get("submap_keyframes", p.submap_keyframes);
  get("scan_voxel", p.scan_voxel);
  get("icp_voxel", p.icp_voxel);
  get("icp_max_dist", p.icp_max_dist);
  get("icp_max_fitness", p.icp_max_fitness);
  get("odom_sigma_trans", p.odom_sigma_trans);
  get("odom_sigma_rot_deg", p.odom_sigma_rot_deg);
  get("loop_sigma_trans", p.loop_sigma_trans);
  get("loop_sigma_rot_deg", p.loop_sigma_rot_deg);
  return p;
}

}  // namespace backend
}  // namespace solid

#endif  // SOLID_BACKEND_PARAMS_YAML_HPP
