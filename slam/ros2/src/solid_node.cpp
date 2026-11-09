// SOLiD loop candidate node (ROS 2).
//
// Subscribes to a point cloud and the odometry that goes with it, picks
// keyframes, and publishes SOLiD loop candidates for the SLAM back-end.
//   in : cloud (sensor_msgs/PointCloud2), odom (nav_msgs/Odometry)
//   out: solid/loop_candidates (solid_ros/LoopCandidate), solid/markers (MarkerArray)
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "solid_ros/msg/loop_candidate.hpp"
#include "solid_slam/config_yaml.hpp"
#include "solid_slam/loop_detector.hpp"

namespace {

Eigen::Isometry3d to_eigen(const geometry_msgs::msg::Pose& p) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y,
                                  p.orientation.z).normalized().toRotationMatrix();
  T.translation() << p.position.x, p.position.y, p.position.z;
  return T;
}

geometry_msgs::msg::Pose to_msg(const Eigen::Isometry3d& T) {
  geometry_msgs::msg::Pose p;
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

double to_sec(const builtin_interfaces::msg::Time& t) { return t.sec + t.nanosec * 1e-9; }

builtin_interfaces::msg::Time to_time(double s) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(s));
  t.nanosec = static_cast<uint32_t>(std::llround((s - std::floor(s)) * 1e9) % 1000000000LL);
  return t;
}

}  // namespace

class SolidNode : public rclcpp::Node {
 public:
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Odom = nav_msgs::msg::Odometry;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Cloud, Odom>;

  SolidNode() : Node("solid") {
    const std::string config = declare_parameter<std::string>("config", "");
    save_dir_ = declare_parameter<std::string>("save_dir", "");
    if (config.empty()) throw std::runtime_error("parameter 'config' is empty");
    params_ = solid_slam::load_params(config);
    detector_ = std::make_unique<solid_slam::LoopDetector>(params_);

    pub_loops_ = create_publisher<solid_ros::msg::LoopCandidate>("solid/loop_candidates", 100);
    pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "solid/markers", rclcpp::QoS(1).transient_local());

    // Best effort matches both sensor drivers and bag playback; a deep queue avoids drops.
    auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    qos.depth = 100;
    sub_cloud_.subscribe(this, "cloud", qos);
    sub_odom_.subscribe(this, "odom", qos);
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(100),
                                                                        sub_cloud_, sub_odom_);
    sync_->registerCallback(std::bind(&SolidNode::callback, this, std::placeholders::_1,
                                      std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "[solid] config %s | cloud %s | odom %s", config.c_str(),
                sub_cloud_.getSubscriber()->get_topic_name(),
                sub_odom_.getSubscriber()->get_topic_name());
  }

  // Writes keyframes.txt and loops.csv to save_dir (if set). Call after spinning.
  void save() const {
    if (save_dir_.empty()) return;
    if (detector_->save(save_dir_))
      RCLCPP_INFO(get_logger(), "[solid] saved %zu keyframes, %zu loops to %s",
                  detector_->keyframes().size(), detector_->loops().size(), save_dir_.c_str());
    else
      RCLCPP_ERROR(get_logger(), "[solid] cannot write to %s", save_dir_.c_str());
  }

 private:
  void callback(const Cloud::ConstSharedPtr& cloud, const Odom::ConstSharedPtr& odom) {
    const Eigen::Isometry3d pose = to_eigen(odom->pose.pose);
    if (!detector_->is_keyframe(pose)) return;

    solid_slam::CloudView view;
    view.data = cloud->data.data();
    view.num_points = static_cast<std::size_t>(cloud->width) * cloud->height;
    view.point_step = cloud->point_step;
    for (const auto& f : cloud->fields) {
      if (f.name == "x") { view.offset_x = f.offset; view.datatype_xyz = f.datatype; }
      if (f.name == "y") view.offset_y = f.offset;
      if (f.name == "z") view.offset_z = f.offset;
      if (!params_.weight_field.empty() && f.name == params_.weight_field) {
        view.offset_w = f.offset;
        view.datatype_w = f.datatype;
      }
    }
    if (view.offset_x < 0 || view.offset_y < 0 || view.offset_z < 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "[solid] cloud has no x/y/z fields");
      return;
    }
    if (!params_.weight_field.empty() && view.offset_w < 0)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "[solid] cloud has no '%s' field, using occupancy",
                           params_.weight_field.c_str());

    const Eigen::Isometry3d world_to_body = pose.inverse();
    solid_slam::to_points(view, params_.cloud_in_world ? &world_to_body : nullptr, pts_, w_);

    const auto t0 = std::chrono::steady_clock::now();
    const auto loops = detector_->add_keyframe(to_sec(cloud->header.stamp), pose, pts_,
                                               w_.empty() ? nullptr : &w_);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    const auto& kfs = detector_->keyframes();
    for (const auto& l : loops) {
      solid_ros::msg::LoopCandidate msg;
      msg.header.stamp = cloud->header.stamp;
      msg.header.frame_id = odom->header.frame_id;
      msg.query_id = static_cast<uint32_t>(l.query_id);
      msg.match_id = static_cast<uint32_t>(l.match_id);
      msg.match_stamp = to_time(kfs[l.match_id].stamp);
      msg.score = l.score;
      msg.yaw = l.yaw_rad;
      msg.query_pose = to_msg(kfs[l.query_id].pose);
      msg.match_pose = to_msg(kfs[l.match_id].pose);
      pub_loops_->publish(msg);
      RCLCPP_INFO(get_logger(), "[solid] loop %u -> %u score %.3f yaw %.1f deg (%.1f ms)",
                  msg.query_id, msg.match_id, l.score, l.yaw_rad * 180.0 / M_PI, ms);
    }
    publish_markers(odom->header.frame_id, cloud->header.stamp);
  }

  void publish_markers(const std::string& frame, const builtin_interfaces::msg::Time& stamp) {
    using visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray arr;
    Marker path, lines;
    path.header.frame_id = lines.header.frame_id = frame;
    path.header.stamp = lines.header.stamp = stamp;
    path.ns = "keyframes";
    lines.ns = "loops";
    path.type = Marker::LINE_STRIP;
    lines.type = Marker::LINE_LIST;
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
    pub_markers_->publish(arr);
  }

  solid_slam::Params params_;
  std::unique_ptr<solid_slam::LoopDetector> detector_;
  std::string save_dir_;
  std::vector<Eigen::Vector3f> pts_;
  std::vector<float> w_;

  rclcpp::Publisher<solid_ros::msg::LoopCandidate>::SharedPtr pub_loops_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  message_filters::Subscriber<Cloud> sub_cloud_;
  message_filters::Subscriber<Odom> sub_odom_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    auto node = std::make_shared<SolidNode>();
    rclcpp::spin(node);
    node->save();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("solid"), "[solid] %s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
