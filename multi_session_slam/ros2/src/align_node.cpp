// SOLiD multi-session alignment node (ROS 2).
//
// Loads sessions saved by any SOLiD front-end, aligns them to the first one with
// solid::ms::Aligner, saves the result and publishes it for RViz (latched):
//   solid_ms/paths, solid_ms/loops (MarkerArray), solid_ms/map (PointCloud2), transient local.
#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "solid/multi_session/aligner.hpp"
#include "solid/multi_session/params_yaml.hpp"

namespace {

std::string base_name(std::string dir) {
  while (!dir.empty() && dir.back() == '/') dir.pop_back();
  return dir.substr(dir.find_last_of('/') + 1);
}

geometry_msgs::msg::Point to_point(const Eigen::Isometry3d& T) {
  geometry_msgs::msg::Point p;
  p.x = T.translation().x();
  p.y = T.translation().y();
  p.z = T.translation().z();
  return p;
}

// Session colors: indigo, magenta, teal, amber, ...
const float kColors[][3] = {{0.25f, 0.32f, 0.71f}, {0.85f, 0.11f, 0.38f}, {0.0f, 0.54f, 0.48f},
                            {1.0f, 0.62f, 0.0f},   {0.47f, 0.33f, 0.28f}};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("solid_multi_session");
  auto log = node->get_logger();
  int ret = 0;
  try {
    const std::string config = node->declare_parameter<std::string>("config", "");
    const std::string save_dir = node->declare_parameter<std::string>("save_dir", "");
    const std::string frame = node->declare_parameter<std::string>("frame_id", "map");
    const double map_voxel = node->declare_parameter<double>("map_voxel", 0.5);
    // `sessions`: one string of folders separated by spaces or commas (first = central).
    std::string joined = node->declare_parameter<std::string>("sessions", "");
    std::replace(joined.begin(), joined.end(), ',', ' ');
    std::vector<std::string> dirs;
    std::istringstream ss(joined);
    for (std::string d; ss >> d;) dirs.push_back(d);
    if (dirs.size() < 2) throw std::runtime_error("need at least two sessions (first = central)");
    const solid::ms::AlignParams params = solid::ms::load_params(config);

    std::vector<std::shared_ptr<solid::ms::Session>> sessions;
    for (const auto& d : dirs) {
      sessions.push_back(std::make_shared<solid::ms::Session>(d, base_name(d), params.scan_voxel));
      RCLCPP_INFO(log, "[solid_ms] session %s: %zu keyframes", sessions.back()->name().c_str(),
               sessions.back()->size());
    }

    const solid::ms::AlignResult r = solid::ms::Aligner(params).align(sessions);
    for (std::size_t s = 1; s < sessions.size(); ++s) {
      int cands = 0, verified = 0;
      for (const auto& l : r.loops)
        if (l.session == static_cast<int>(s)) cands++, verified += l.verified;
      RCLCPP_INFO(log, "[solid_ms] %s -> %s: %d SOLiD candidates, %d verified, %d inliers, %s",
               sessions[s]->name().c_str(), sessions[0]->name().c_str(), cands, verified,
               r.sessions[s].inliers, r.sessions[s].aligned ? "ALIGNED" : "not aligned");
    }
    RCLCPP_INFO(log, "[solid_ms] done in %.1f s", r.seconds);
    if (!save_dir.empty()) {
      solid::ms::Aligner::save(save_dir, sessions, r);
      RCLCPP_INFO(log, "[solid_ms] saved to %s", save_dir.c_str());
    }

    // ---- RViz (latched) ----
    const auto latched = rclcpp::QoS(1).transient_local();
    auto pub_path = node->create_publisher<visualization_msgs::msg::MarkerArray>("solid_ms/paths", latched);
    auto pub_loops = node->create_publisher<visualization_msgs::msg::MarkerArray>("solid_ms/loops", latched);
    auto pub_map = node->create_publisher<sensor_msgs::msg::PointCloud2>("solid_ms/map", latched);
    const auto now = node->now();

    visualization_msgs::msg::MarkerArray paths;
    for (std::size_t s = 0; s < sessions.size(); ++s) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame;
      m.header.stamp = now;
      m.ns = sessions[s]->name();
      m.type = visualization_msgs::msg::Marker::LINE_STRIP;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.6;
      const float* c = kColors[s % 5];
      m.color.r = c[0], m.color.g = c[1], m.color.b = c[2], m.color.a = 1.0f;
      for (const auto& T : r.sessions[s].poses) m.points.push_back(to_point(T));
      paths.markers.push_back(m);
    }
    pub_path->publish(paths);

    visualization_msgs::msg::MarkerArray loops;
    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = frame;
    lines.header.stamp = now;
    lines.ns = "inter_session_loops";
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.3;
    lines.color.r = 0.1f, lines.color.g = 0.8f, lines.color.b = 0.2f, lines.color.a = 1.0f;
    for (const auto& l : r.loops) {
      if (!l.inlier) continue;
      lines.points.push_back(to_point(r.sessions[0].poses[l.central_kf]));
      lines.points.push_back(to_point(r.sessions[l.session].poses[l.query_kf]));
    }
    if (!lines.points.empty()) loops.markers.push_back(lines);
    pub_loops->publish(loops);

    sensor_msgs::msg::PointCloud2 map;
    pcl::toROSMsg(*solid::ms::build_map(sessions, r, map_voxel), map);
    map.header.frame_id = frame;
    map.header.stamp = now;
    pub_map->publish(map);
    RCLCPP_INFO(log, "[solid_ms] published solid_ms/paths, loops, map (frame %s)", frame.c_str());
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(log, "[solid_ms] %s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
