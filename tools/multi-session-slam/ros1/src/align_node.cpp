// SOLiD multi-session alignment node (ROS 1).
//
// Loads sessions saved by any SOLiD front-end, aligns them to the first one with
// solid::backend::Aligner, saves the result and publishes it for RViz (latched):
//   ~paths (nav_msgs/Path per session), ~loops (MarkerArray), ~map (PointCloud2).
#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "solid/backend/aligner.hpp"
#include "solid/backend/params_yaml.hpp"

namespace {

std::string base_name(std::string dir) {
  while (!dir.empty() && dir.back() == '/') dir.pop_back();
  return dir.substr(dir.find_last_of('/') + 1);
}

geometry_msgs::Point to_point(const Eigen::Isometry3d& T) {
  geometry_msgs::Point p;
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
  ros::init(argc, argv, "solid_multi_session");
  ros::NodeHandle pnh("~");
  try {
    std::string config, save_dir, frame;
    std::vector<std::string> dirs;
    double map_voxel;
    pnh.param<std::string>("config", config, "");
    pnh.param<std::string>("save_dir", save_dir, "");
    pnh.param<std::string>("frame_id", frame, "map");
    pnh.param<double>("map_voxel", map_voxel, 0.5);
    // `sessions`: a list, or one string of folders separated by spaces or commas.
    if (!pnh.getParam("sessions", dirs)) {
      std::string joined;
      pnh.param<std::string>("sessions", joined, "");
      std::replace(joined.begin(), joined.end(), ',', ' ');
      std::istringstream ss(joined);
      for (std::string d; ss >> d;) dirs.push_back(d);
    }
    if (dirs.size() < 2) throw std::runtime_error("need at least two sessions (first = central)");
    const solid::backend::AlignParams params = solid::backend::load_params(config);

    std::vector<std::shared_ptr<solid::backend::Session>> sessions;
    for (const auto& d : dirs) {
      sessions.push_back(std::make_shared<solid::backend::Session>(d, base_name(d), params.scan_voxel));
      ROS_INFO("[solid_ms] session %s: %zu keyframes", sessions.back()->name().c_str(),
               sessions.back()->size());
    }

    const solid::backend::AlignResult r = solid::backend::Aligner(params).align(sessions);
    for (std::size_t s = 1; s < sessions.size(); ++s) {
      int cands = 0, verified = 0;
      for (const auto& l : r.loops)
        if (l.session == static_cast<int>(s)) cands++, verified += l.verified;
      ROS_INFO("[solid_ms] %s -> %s: %d SOLiD candidates, %d verified, %d inliers, %s",
               sessions[s]->name().c_str(), sessions[0]->name().c_str(), cands, verified,
               r.sessions[s].inliers, r.sessions[s].aligned ? "ALIGNED" : "not aligned");
    }
    ROS_INFO("[solid_ms] done in %.1f s", r.seconds);
    if (!save_dir.empty()) {
      solid::backend::Aligner::save(save_dir, sessions, r);
      ROS_INFO("[solid_ms] saved to %s", save_dir.c_str());
    }

    // ---- RViz (latched) ----
    ros::Publisher pub_path = pnh.advertise<visualization_msgs::MarkerArray>("paths", 1, true);
    ros::Publisher pub_loops = pnh.advertise<visualization_msgs::MarkerArray>("loops", 1, true);
    ros::Publisher pub_map = pnh.advertise<sensor_msgs::PointCloud2>("map", 1, true);
    const ros::Time now = ros::Time::now();

    visualization_msgs::MarkerArray paths;
    for (std::size_t s = 0; s < sessions.size(); ++s) {
      visualization_msgs::Marker m;
      m.header.frame_id = frame;
      m.header.stamp = now;
      m.ns = sessions[s]->name();
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.6;
      const float* c = kColors[s % 5];
      m.color.r = c[0], m.color.g = c[1], m.color.b = c[2], m.color.a = 1.0f;
      for (const auto& T : r.sessions[s].poses) m.points.push_back(to_point(T));
      paths.markers.push_back(m);
    }
    pub_path.publish(paths);

    visualization_msgs::MarkerArray loops;
    visualization_msgs::Marker lines;
    lines.header.frame_id = frame;
    lines.header.stamp = now;
    lines.ns = "inter_session_loops";
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.pose.orientation.w = 1.0;
    lines.scale.x = 0.3;
    lines.color.r = 0.1f, lines.color.g = 0.8f, lines.color.b = 0.2f, lines.color.a = 1.0f;
    for (const auto& l : r.loops) {
      if (!l.inlier) continue;
      lines.points.push_back(to_point(r.sessions[0].poses[l.central_kf]));
      lines.points.push_back(to_point(r.sessions[l.session].poses[l.query_kf]));
    }
    if (!lines.points.empty()) loops.markers.push_back(lines);
    pub_loops.publish(loops);

    sensor_msgs::PointCloud2 map;
    pcl::toROSMsg(*solid::backend::build_map(sessions, r, map_voxel), map);
    map.header.frame_id = frame;
    map.header.stamp = now;
    pub_map.publish(map);
    ROS_INFO("[solid_ms] published %s/paths, loops, map (frame %s)", pnh.getNamespace().c_str(),
             frame.c_str());
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("[solid_ms] %s", e.what());
    return 1;
  }
  return 0;
}
