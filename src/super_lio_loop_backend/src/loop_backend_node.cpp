#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/point_tests.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include "super_lio_loop_backend/upstream/BTC.h"
#include "super_lio_loop_backend/upstream/loop_refine.hpp"
#include "super_lio_loop_msgs/msg/keyframe.hpp"
#include "super_lio_loop_msgs/msg/loop_candidate.hpp"
#include "super_lio_loop_msgs/srv/get_corrected_poses.hpp"
#include "super_lio_loop_msgs/srv/run_global_ba.hpp"

namespace
{

using KeyframeMsg = super_lio_loop_msgs::msg::Keyframe;
using LoopCandidateMsg = super_lio_loop_msgs::msg::LoopCandidate;
using GetCorrectedPoses = super_lio_loop_msgs::srv::GetCorrectedPoses;
using RunGlobalBA = super_lio_loop_msgs::srv::RunGlobalBA;

std::atomic_bool stop_requested{false};

void signalHandler(int)
{
  stop_requested = true;
}

gtsam::Pose3 toGtsam(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Quaterniond q(
    pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);
  if (!std::isfinite(q.norm()) || q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  return gtsam::Pose3(
    gtsam::Rot3(q.toRotationMatrix()),
    gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::msg::Pose toRos(const gtsam::Pose3 & pose)
{
  geometry_msgs::msg::Pose result;
  const Eigen::Quaterniond q(pose.rotation().matrix());
  result.position.x = pose.translation().x();
  result.position.y = pose.translation().y();
  result.position.z = pose.translation().z();
  result.orientation.x = q.x();
  result.orientation.y = q.y();
  result.orientation.z = q.z();
  result.orientation.w = q.w();
  return result;
}

gtsam::Pose3 toGtsam(
  const Eigen::Matrix3d & rotation, const Eigen::Vector3d & translation)
{
  return gtsam::Pose3(gtsam::Rot3(rotation), gtsam::Point3(translation));
}

gtsam::noiseModel::Diagonal::shared_ptr diagonalNoise(
  const std::vector<double> & values, const std::vector<double> & defaults)
{
  const auto & source = values.size() == 6 ? values : defaults;
  gtsam::Vector6 sigmas;
  for (std::size_t i = 0; i < 6; ++i) {
    sigmas(static_cast<Eigen::Index>(i)) =
      std::max(source[i], 1e-9);
  }
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

struct StoredKeyframe
{
  std::uint32_t id;
  gtsam::Pose3 odometry_pose;
  pcl::PointCloud<pcl::PointXYZI>::Ptr btc_cloud;
  pcl::PointCloud<PointType>::Ptr gba_cloud;
};

}  // namespace

class LoopBackendNode final : public rclcpp::Node
{
public:
  LoopBackendNode()
  : Node("super_lio_loop_backend")
  {
    const auto keyframe_topic = declare_parameter<std::string>(
      "loop.keyframe_topic", "/super_lio/keyframe");
    const auto candidate_topic = declare_parameter<std::string>(
      "loop.candidate_topic", "/super_lio_loop_backend/loop_candidates");

    ConfigSetting config;
    config.useful_corner_num_ = declare_parameter<int>(
      "loop.btc.useful_corner_num", 100);
    config.plane_merge_normal_thre_ = declare_parameter<double>(
      "loop.btc.plane_merge_normal_threshold", 0.1);
    config.plane_merge_dis_thre_ = declare_parameter<double>(
      "loop.btc.plane_merge_distance_threshold", 0.3);
    config.plane_detection_thre_ = declare_parameter<double>(
      "loop.btc.plane_detection_threshold", 0.01);
    config.voxel_size_ = declare_parameter<double>(
      "loop.btc.voxel_size", 1.0);
    config.voxel_init_num_ = declare_parameter<int>(
      "loop.btc.voxel_init_num", 10);
    config.proj_plane_num_ = declare_parameter<int>(
      "loop.btc.projection_plane_num", 2);
    config.proj_image_resolution_ = declare_parameter<double>(
      "loop.btc.projection_resolution", 0.5);
    config.proj_image_high_inc_ = declare_parameter<double>(
      "loop.btc.projection_height_increment", 0.1);
    config.proj_dis_min_ = declare_parameter<double>(
      "loop.btc.projection_distance_min", 0.0);
    config.proj_dis_max_ = declare_parameter<double>(
      "loop.btc.projection_distance_max", 5.0);
    config.summary_min_thre_ = declare_parameter<double>(
      "loop.btc.summary_min_threshold", 10.0);
    config.line_filter_enable_ = declare_parameter<int>(
      "loop.btc.line_filter_enable", 1);
    config.touch_filter_enable_ = declare_parameter<int>(
      "loop.btc.touch_filter_enable", 0);
    config.descriptor_near_num_ = declare_parameter<double>(
      "loop.btc.descriptor_near_num", 15.0);
    config.descriptor_min_len_ = declare_parameter<double>(
      "loop.btc.descriptor_min_length", 2.0);
    config.descriptor_max_len_ = declare_parameter<double>(
      "loop.btc.descriptor_max_length", 50.0);
    config.non_max_suppression_radius_ = declare_parameter<double>(
      "loop.btc.non_max_suppression_radius", 2.0);
    config.std_side_resolution_ = declare_parameter<double>(
      "loop.btc.side_resolution", 0.2);
    config.skip_near_num_ = declare_parameter<int>(
      "loop.btc.skip_near_num", 30);
    config.candidate_num_ = declare_parameter<int>(
      "loop.btc.candidate_num", 20);
    config.rough_dis_threshold_ = declare_parameter<double>(
      "loop.btc.rough_distance_threshold", 0.01);
    config.similarity_threshold_ = declare_parameter<double>(
      "loop.btc.similarity_threshold", 0.7);
    config.icp_threshold_ = declare_parameter<double>(
      "loop.btc.icp_threshold", 0.15);
    config.normal_threshold_ = declare_parameter<double>(
      "loop.btc.normal_threshold", 0.2);
    config.dis_threshold_ = declare_parameter<double>(
      "loop.btc.distance_threshold", 0.5);

    loop_score_threshold_ = declare_parameter<double>(
      "loop.btc.score_threshold", 0.45);
    icp_eigenvalue_threshold_ = declare_parameter<double>(
      "loop.btc.icp_eigenvalue_threshold", 14.0);
    odom_noise_ = diagonalNoise(
      declare_parameter<std::vector<double>>(
        "loop.pgo.odom_noise", {0.02, 0.02, 0.02, 0.05, 0.05, 0.05}),
      {0.02, 0.02, 0.02, 0.05, 0.05, 0.05});
    loop_noise_ = diagonalNoise(
      declare_parameter<std::vector<double>>(
        "loop.pgo.loop_noise", {0.01, 0.01, 0.01, 0.03, 0.03, 0.03}),
      {0.01, 0.01, 0.01, 0.03, 0.03, 0.03});
    prior_noise_ = diagonalNoise(
      declare_parameter<std::vector<double>>(
        "loop.pgo.prior_noise", {1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6}),
      {1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6});

    gba_enabled_ = declare_parameter<bool>("loop.gba.enable", true);
    gba_voxel_size_ = declare_parameter<double>("loop.gba.voxel_size", 1.0);
    gba_min_eigen_value_ = declare_parameter<double>(
      "loop.gba.min_eigen_value", 0.01);
    gba_eigen_ratio_thresholds_ = declare_parameter<std::vector<double>>(
      "loop.gba.eigen_ratio_thresholds", {0.01, 0.01, 0.01});
    gba_max_layer_ = declare_parameter<int>("loop.gba.max_layer", 2);
    gba_max_iterations_ = declare_parameter<int>(
      "loop.gba.max_iterations", 5);
    gba_threads_ = std::max(
      1, static_cast<int>(
        declare_parameter<int>("loop.gba.thread_num", 2)));

    descriptor_manager_ = std::make_unique<STDescManager>(config);

    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = declare_parameter<double>(
      "loop.pgo.relinearize_threshold", 0.01);
    isam_params.relinearizeSkip = declare_parameter<int>(
      "loop.pgo.relinearize_skip", 1);
    isam_ = std::make_unique<gtsam::ISAM2>(isam_params);

    candidate_pub_ = create_publisher<LoopCandidateMsg>(
      candidate_topic, rclcpp::QoS(10));
    keyframe_sub_ = create_subscription<KeyframeMsg>(
      keyframe_topic, rclcpp::QoS(20).reliable(),
      std::bind(&LoopBackendNode::onKeyframe, this, std::placeholders::_1));
    corrected_pose_service_ = create_service<GetCorrectedPoses>(
      "/super_lio_loop_backend/get_corrected_poses",
      std::bind(
        &LoopBackendNode::onGetCorrectedPoses, this,
        std::placeholders::_1, std::placeholders::_2));
    gba_service_ = create_service<RunGlobalBA>(
      "/super_lio_loop_backend/run_global_ba",
      std::bind(
        &LoopBackendNode::onRunGlobalBA, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Loop backend ready: keyframes='%s', BTC + ISAM2 + OctreeGBA",
      keyframe_topic.c_str());
  }

private:
  void onKeyframe(const KeyframeMsg::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const std::uint32_t expected_id =
      static_cast<std::uint32_t>(keyframes_.size());
    if (message->id != expected_id) {
      RCLCPP_ERROR(
        get_logger(), "Rejecting keyframe %u: expected strictly increasing id %u",
        message->id, expected_id);
      return;
    }

    auto btc_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    try {
      pcl::fromROSMsg(message->cloud, *btc_cloud);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Failed to decode keyframe %u cloud: %s",
        message->id, error.what());
      return;
    }
    if (btc_cloud->empty()) {
      RCLCPP_WARN(
        get_logger(), "Ignoring keyframe %u with an empty cloud", message->id);
      return;
    }

    auto gba_cloud = std::make_shared<pcl::PointCloud<PointType>>();
    gba_cloud->reserve(btc_cloud->size());
    for (const auto & point : btc_cloud->points) {
      if (!pcl::isFinite(point)) {
        continue;
      }
      PointType output;
      output.x = point.x;
      output.y = point.y;
      output.z = point.z;
      output.intensity = point.intensity;
      gba_cloud->push_back(output);
    }
    if (gba_cloud->empty()) {
      RCLCPP_WARN(
        get_logger(), "Ignoring keyframe %u with no finite points", message->id);
      return;
    }

    const gtsam::Pose3 odometry_pose = toGtsam(message->world_pose);
    graph_delta_.resize(0);
    values_delta_.clear();
    values_delta_.insert(message->id, odometry_pose);
    if (keyframes_.empty()) {
      graph_delta_.addPrior(message->id, odometry_pose, prior_noise_);
    } else {
      const auto & previous = keyframes_.back();
      graph_delta_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        previous.id, message->id,
        previous.odometry_pose.between(odometry_pose), odom_noise_);
    }

    std::vector<STD> descriptors;
    descriptor_manager_->GenerateSTDescs(
      btc_cloud, descriptors, static_cast<int>(message->id));

    std::pair<int, double> result(-1, 0.0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
    std::vector<std::pair<STD, STD>> matches;
    descriptor_manager_->SearchLoop(
      descriptors, result, loop_transform, matches,
      descriptor_manager_->plane_cloud_vec_.back());

    if (
      result.first >= 0 &&
      result.first < static_cast<int>(message->id) &&
      result.second >= loop_score_threshold_)
    {
      const auto candidate_id = static_cast<std::uint32_t>(result.first);
      auto refined_transform = loop_transform;
      const bool icp_verified = icp_normal(
        *descriptor_manager_->plane_cloud_vec_.back(),
        *descriptor_manager_->plane_cloud_vec_[candidate_id],
        refined_transform, icp_eigenvalue_threshold_);
      if (icp_verified) {
        const gtsam::Pose3 relative_pose =
          toGtsam(refined_transform.second, refined_transform.first);
        graph_delta_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          candidate_id, message->id, relative_pose, loop_noise_);

        LoopCandidateMsg candidate;
        candidate.header = message->header;
        candidate.id_i = candidate_id;
        candidate.id_j = message->id;
        candidate.relative_pose = toRos(relative_pose);
        candidate.score = static_cast<float>(result.second);
        candidate_pub_->publish(candidate);
        RCLCPP_INFO(
          get_logger(), "Accepted loop %u -> %u (score %.3f)",
          candidate_id, message->id, result.second);
      } else {
        RCLCPP_DEBUG(
          get_logger(), "Rejected BTC candidate %u -> %u during plane ICP",
          candidate_id, message->id);
      }
    }

    descriptor_manager_->AddSTDescs(descriptors);
    try {
      isam_->update(graph_delta_, values_delta_);
      isam_->update();
      estimate_ = isam_->calculateEstimate();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "ISAM2 update for keyframe %u failed: %s",
        message->id, error.what());
      return;
    }

    keyframes_.push_back(
      StoredKeyframe{message->id, odometry_pose, btc_cloud, gba_cloud});
    gba_estimate_.clear();
    RCLCPP_DEBUG(
      get_logger(), "Stored keyframe %u (%zu points)",
      message->id, gba_cloud->size());
  }

  void onGetCorrectedPoses(
    const std::shared_ptr<GetCorrectedPoses::Request>,
    std::shared_ptr<GetCorrectedPoses::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const gtsam::Values & source =
      gba_estimate_.empty() ? estimate_ : gba_estimate_;
    response->success = !keyframes_.empty() &&
      source.size() == keyframes_.size();
    if (!response->success) {
      return;
    }
    response->ids.reserve(keyframes_.size());
    response->corrected_poses.reserve(keyframes_.size());
    for (const auto & keyframe : keyframes_) {
      if (!source.exists(keyframe.id)) {
        response->ids.clear();
        response->corrected_poses.clear();
        response->success = false;
        return;
      }
      response->ids.push_back(keyframe.id);
      response->corrected_poses.push_back(
        toRos(source.at<gtsam::Pose3>(keyframe.id)));
    }
  }

  void onRunGlobalBA(
    const std::shared_ptr<RunGlobalBA::Request>,
    std::shared_ptr<RunGlobalBA::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response->success = runGlobalBA(response->message);
  }

  bool runGlobalBA(std::string & message)
  {
    if (!gba_enabled_) {
      message = "OctreeGBA is disabled by loop.gba.enable";
      return false;
    }
    if (keyframes_.size() < 2 || estimate_.size() != keyframes_.size()) {
      message = "At least two optimized keyframes are required";
      return false;
    }

    std::vector<IMUST> states(keyframes_.size());
    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
      const auto pose = estimate_.at<gtsam::Pose3>(keyframes_[i].id);
      states[i].R = pose.rotation().matrix();
      states[i].p = pose.translation();
    }

    gba_voxel_size = gba_voxel_size_;
    gba_min_eigen_value = gba_min_eigen_value_;
    gba_eigen_value_array = gba_eigen_ratio_thresholds_;
    max_layer = std::max(0, gba_max_layer_);
    if (
      gba_eigen_value_array.size() <
      static_cast<std::size_t>(max_layer + 1))
    {
      gba_eigen_value_array.resize(
        static_cast<std::size_t>(max_layer + 1),
        gba_eigen_value_array.empty() ? 0.01 :
        gba_eigen_value_array.back());
    }

    std::unordered_map<VOXEL_LOC, OctreeGBA *> octree;
    for (std::size_t i = 0; i < keyframes_.size(); ++i) {
      OctreeGBA::cut_voxel(
        octree, states[i], keyframes_[i].gba_cloud,
        static_cast<int>(i), static_cast<int>(keyframes_.size()));
    }

    LidarFactor factors(static_cast<int>(keyframes_.size()));
    const int recut_threads = std::max(
      1, std::min(gba_threads_, static_cast<int>(octree.size())));
    OctreeGBA_multi_recut(octree, factors, recut_threads);
    if (factors.plvec_voxels.empty()) {
      message = "OctreeGBA found no shared planar voxels";
      return false;
    }

    Lidar_BA_Optimizer optimizer;
    optimizer.thd_num = std::max(
      1, std::min(gba_threads_, static_cast<int>(factors.plvec_voxels.size())));
    Eigen::MatrixXd hessian;
    std::vector<double> residuals;
    optimizer.damping_iter(
      states, factors, &hessian, residuals,
      std::max(1, gba_max_iterations_), false);

    gtsam::Values result;
    for (std::size_t i = 0; i < states.size(); ++i) {
      if (!states[i].R.allFinite() || !states[i].p.allFinite()) {
        message = "OctreeGBA produced a non-finite pose; result discarded";
        return false;
      }
      result.insert(
        keyframes_[i].id,
        gtsam::Pose3(gtsam::Rot3(states[i].R), gtsam::Point3(states[i].p)));
    }
    gba_estimate_ = std::move(result);
    message = "OctreeGBA optimized " + std::to_string(keyframes_.size()) +
      " keyframes using " + std::to_string(factors.plvec_voxels.size()) +
      " planar voxel factors";
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  std::mutex mutex_;
  std::unique_ptr<STDescManager> descriptor_manager_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::NonlinearFactorGraph graph_delta_;
  gtsam::Values values_delta_;
  gtsam::Values estimate_;
  gtsam::Values gba_estimate_;
  std::vector<StoredKeyframe> keyframes_;

  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr loop_noise_;
  double loop_score_threshold_;
  double icp_eigenvalue_threshold_;

  bool gba_enabled_;
  double gba_voxel_size_;
  double gba_min_eigen_value_;
  std::vector<double> gba_eigen_ratio_thresholds_;
  int gba_max_layer_;
  int gba_max_iterations_;
  int gba_threads_;

  rclcpp::Subscription<KeyframeMsg>::SharedPtr keyframe_sub_;
  rclcpp::Publisher<LoopCandidateMsg>::SharedPtr candidate_pub_;
  rclcpp::Service<GetCorrectedPoses>::SharedPtr corrected_pose_service_;
  rclcpp::Service<RunGlobalBA>::SharedPtr gba_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(),
    rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  const auto node = std::make_shared<LoopBackendNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !stop_requested) {
    executor.spin_once(std::chrono::milliseconds(20));
  }

  // Other nodes in the launch receive the same signal. Keep the corrected-pose
  // service alive briefly so Super-LIO can save before its final shutdown.
  const auto grace_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < grace_deadline) {
    executor.spin_once(std::chrono::milliseconds(20));
  }
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
