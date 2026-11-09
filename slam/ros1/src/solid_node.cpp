// SOLiD loop candidate node (ROS 1).
//
// Subscribes to a point cloud and the odometry that goes with it, picks
// keyframes, and publishes SOLiD loop candidates for the SLAM back-end.
//   in : ~cloud (sensor_msgs/PointCloud2), ~odom (nav_msgs/Odometry)
//   out: ~loop_candidates (solid_ros/LoopCandidate), ~markers (visualization_msgs/MarkerArray)
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include "solid_ros/LoopCandidate.h"
#include "solid/slam/config_yaml.hpp"
#include "solid/slam/loop_detector.hpp"
#include "solid/slam/session_writer.hpp"

namespace {

Eigen::Isometry3d to_eigen(const geometry_msgs::Pose& p) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y,
                                  p.orientation.z).normalized().toRotationMatrix();
  T.translation() << p.position.x, p.position.y, p.position.z;
  return T;
}

geometry_msgs::Pose to_msg(const Eigen::Isometry3d& T) {
  geometry_msgs::Pose p;
  const Eigen::Quaterniond q(T.rotation());
  p.position.x = T.translation().x();
  p.position.y = T.translation().y();
  p.position.z = T.translation().z();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  return p;
}

}  // namespace

class SolidNode {
 public:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>;

  SolidNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    std::string config;
    pnh.param<std::string>("config", config, std::string(SOLID_CONFIG_DIR) + "/kitti.yaml");
    pnh.param<std::string>("save_dir", save_dir_, "");
    params_ = solid::slam::load_params(config);
    detector_ = std::make_unique<solid::slam::LoopDetector>(params_);
    session_ = std::make_unique<solid::slam::SessionWriter>(save_dir_);

    pub_loops_ = pnh.advertise<solid_ros::LoopCandidate>("loop_candidates", 100);
    pub_markers_ = pnh.advertise<visualization_msgs::MarkerArray>("markers", 1, true);

    sub_cloud_.subscribe(nh, "cloud", 10);
    sub_odom_.subscribe(nh, "odom", 100);
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100),
                                                                        sub_cloud_, sub_odom_);
    sync_->registerCallback(&SolidNode::callback, this);

    ROS_INFO("[solid] config %s | cloud %s | odom %s", config.c_str(),
             sub_cloud_.getSubscriber().getTopic().c_str(),
             sub_odom_.getSubscriber().getTopic().c_str());
  }

  // Finishes the LT-mapper session in save_dir (if set). Call after spinning.
  void save() const {
    if (save_dir_.empty()) return;
    if (session_->finish(*detector_))
      ROS_INFO("[solid] saved %zu keyframes, %zu loops to %s",
                  detector_->keyframes().size(), detector_->loops().size(), save_dir_.c_str());
    else
      ROS_ERROR("[solid] cannot write to %s", save_dir_.c_str());
  }

 private:
  void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                const nav_msgs::Odometry::ConstPtr& odom) {
    const Eigen::Isometry3d pose = to_eigen(odom->pose.pose);
    if (!detector_->is_keyframe(pose)) return;

    solid::slam::CloudView view;
    view.data = cloud->data.data();
    view.num_points = static_cast<std::size_t>(cloud->width) * cloud->height;
    view.point_step = cloud->point_step;
    for (const auto& f : cloud->fields) {
      if (f.name == "x") { view.offset_x = f.offset; view.datatype_xyz = f.datatype; }
      if (f.name == "y") view.offset_y = f.offset;
      if (f.name == "z") view.offset_z = f.offset;
      if (f.name == "intensity") { view.offset_i = f.offset; view.datatype_i = f.datatype; }
      if (!params_.weight_field.empty() && f.name == params_.weight_field) {
        view.offset_w = f.offset;
        view.datatype_w = f.datatype;
      }
    }
    if (view.offset_x < 0 || view.offset_y < 0 || view.offset_z < 0) {
      ROS_ERROR_THROTTLE(5.0, "[solid] cloud has no x/y/z fields");
      return;
    }
    if (!params_.weight_field.empty() && view.offset_w < 0)
      ROS_WARN_THROTTLE(5.0, "[solid] cloud has no '%s' field, using occupancy",
                        params_.weight_field.c_str());

    const Eigen::Isometry3d world_to_body = pose.inverse();
    solid::slam::to_points(view, params_.cloud_in_world ? &world_to_body : nullptr, pts_, w_,
                          session_->enabled() ? &intensity_ : nullptr);

    const ros::WallTime t0 = ros::WallTime::now();
    const auto loops = detector_->add_keyframe(cloud->header.stamp.toSec(), pose, pts_,
                                               w_.empty() ? nullptr : &w_);
    session_->add_keyframe(detector_->keyframes().back().id, pts_, intensity_,
                           detector_->last_descriptor());
    const double ms = (ros::WallTime::now() - t0).toSec() * 1e3;

    const auto& kfs = detector_->keyframes();
    for (const auto& l : loops) {
      solid_ros::LoopCandidate msg;
      msg.header.stamp = cloud->header.stamp;
      msg.header.frame_id = odom->header.frame_id;
      msg.query_id = l.query_id;
      msg.match_id = l.match_id;
      msg.match_stamp = ros::Time(kfs[l.match_id].stamp);
      msg.score = l.score;
      msg.yaw = l.yaw_rad;
      msg.query_pose = to_msg(kfs[l.query_id].pose);
      msg.match_pose = to_msg(kfs[l.match_id].pose);
      pub_loops_.publish(msg);
      ROS_INFO("[solid] loop %u -> %u score %.3f yaw %.1f deg (%.1f ms)", msg.query_id,
               msg.match_id, l.score, l.yaw_rad * 180.0 / M_PI, ms);
    }
    publish_markers(odom->header.frame_id, cloud->header.stamp);
  }

  void publish_markers(const std::string& frame, const ros::Time& stamp) {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker path, lines;
    path.header.frame_id = lines.header.frame_id = frame;
    path.header.stamp = lines.header.stamp = stamp;
    path.ns = "keyframes";
    lines.ns = "loops";
    path.type = visualization_msgs::Marker::LINE_STRIP;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    path.pose.orientation.w = lines.pose.orientation.w = 1.0;
    path.scale.x = 0.3;
    lines.scale.x = 0.3;
    path.color.r = path.color.g = path.color.b = 0.6f;
    path.color.a = 1.0f;
    lines.color.r = 0.9f;
    lines.color.g = 0.15f;
    lines.color.b = 0.15f;
    lines.color.a = 1.0f;
    const auto& kfs = detector_->keyframes();
    for (const auto& k : kfs) path.points.push_back(to_msg(k.pose).position);
    for (const auto& l : detector_->loops()) {
      lines.points.push_back(to_msg(kfs[l.query_id].pose).position);
      lines.points.push_back(to_msg(kfs[l.match_id].pose).position);
    }
    arr.markers.push_back(path);
    if (!lines.points.empty()) arr.markers.push_back(lines);
    pub_markers_.publish(arr);
  }

  solid::slam::Params params_;
  std::unique_ptr<solid::slam::LoopDetector> detector_;
  std::unique_ptr<solid::slam::SessionWriter> session_;
  std::string save_dir_;
  std::vector<Eigen::Vector3f> pts_;
  std::vector<float> w_, intensity_;

  ros::Publisher pub_loops_, pub_markers_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "solid_node");
  ros::NodeHandle nh, pnh("~");
  try {
    SolidNode node(nh, pnh);
    ros::spin();
    node.save();
  } catch (const std::exception& e) {
    ROS_FATAL("[solid] %s", e.what());
    return 1;
  }
  return 0;
}
