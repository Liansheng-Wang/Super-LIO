#include "lio/keyframe_manager.h"

#include <algorithm>
#include <cmath>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace LI2Sup
{

KeyframeManager::KeyframeManager(
  double translation_threshold, double rotation_threshold_deg,
  std::size_t submap_scan_count, double submap_voxel_size,
  KeyframeCallback keyframe_callback)
: translation_threshold_(std::max(0.0, translation_threshold)),
  rotation_threshold_rad_(
    std::max(0.0, rotation_threshold_deg) * std::acos(-1.0) / 180.0),
  submap_scan_count_(std::max<std::size_t>(1, submap_scan_count)),
  submap_voxel_size_(std::max(0.0, submap_voxel_size)),
  max_pending_scans_(std::max<std::size_t>(2, submap_scan_count_ * 2)),
  keyframe_callback_(std::move(keyframe_callback)),
  worker_(&KeyframeManager::workerLoop, this)
{
}

KeyframeManager::~KeyframeManager()
{
  stop();
}

void KeyframeManager::stop()
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stopping_) {
      return;
    }
    stopping_ = true;
  }
  queue_cv_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
}

std::vector<LoopKeyframe> KeyframeManager::keyframes() const
{
  std::lock_guard<std::mutex> lock(keyframes_mutex_);
  return keyframes_;
}

bool KeyframeManager::shouldCreate(const NavState & state) const
{
  if (!last_keyframe_state_) {
    return true;
  }

  const auto & previous = *last_keyframe_state_;
  const double translation =
    (state.p - previous.p).template cast<double>().norm();
  const Eigen::Matrix3d relative_rotation =
    (previous.R.R_.transpose() * state.R.R_).template cast<double>();
  const double cos_angle = std::clamp(
    (relative_rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double angle = std::acos(cos_angle);
  return translation >= translation_threshold_ ||
         angle >= rotation_threshold_rad_;
}

void KeyframeManager::consider(
  const NavState & state, const BASIC::CloudPtr & body_cloud)
{
  if (!body_cloud || body_cloud->empty()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stopping_ || pending_scans_.size() >= max_pending_scans_) {
      return;
    }
  }

  BufferedScan scan{
    state, std::make_shared<BASIC::PointCloudType>(*body_cloud)};
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (stopping_ || pending_scans_.size() >= max_pending_scans_) {
      return;
    }
    pending_scans_.push_back(std::move(scan));
  }
  queue_cv_.notify_one();
}

void KeyframeManager::workerLoop()
{
  while (true) {
    BufferedScan scan;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {
        return stopping_ || !pending_scans_.empty();
      });
      if (pending_scans_.empty()) {
        if (stopping_) {
          break;
        }
        continue;
      }
      scan = std::move(pending_scans_.front());
      pending_scans_.pop_front();
    }
    processScan(std::move(scan));
  }
}

void KeyframeManager::processScan(BufferedScan scan)
{
  const NavState state = scan.state;
  buffered_scans_.push_back(std::move(scan));
  if (buffered_scans_.size() < submap_scan_count_) {
    return;
  }

  if (!shouldCreate(state)) {
    buffered_scans_.erase(buffered_scans_.begin());
    return;
  }

  auto submap = buildSubmap(state);
  if (!submap || submap->empty()) {
    buffered_scans_.clear();
    return;
  }

  LoopKeyframe keyframe;
  keyframe.id = next_keyframe_id_++;
  keyframe.state = state;
  keyframe.cloud = std::move(submap);
  last_keyframe_state_ = state;
  buffered_scans_.clear();

  if (keyframe_callback_) {
    keyframe_callback_(keyframe);
  }
  {
    std::lock_guard<std::mutex> lock(keyframes_mutex_);
    keyframes_.push_back(std::move(keyframe));
  }
}

BASIC::CloudPtr KeyframeManager::buildSubmap(const NavState & state) const
{
  auto merged = std::make_shared<BASIC::PointCloudType>();
  std::size_t point_count = 0;
  for (const auto & scan : buffered_scans_) {
    point_count += scan.cloud->size();
  }
  merged->reserve(point_count);

  const BASIC::M3 world_to_keyframe = state.R.R_.transpose();
  for (const auto & scan : buffered_scans_) {
    Eigen::Matrix4f scan_to_keyframe = Eigen::Matrix4f::Identity();
    scan_to_keyframe.block<3, 3>(0, 0) =
      (world_to_keyframe * scan.state.R.R_).cast<float>();
    scan_to_keyframe.block<3, 1>(0, 3) =
      (world_to_keyframe * (scan.state.p - state.p)).cast<float>();

    BASIC::PointCloudType transformed;
    pcl::transformPointCloud(*scan.cloud, transformed, scan_to_keyframe);
    *merged += transformed;
  }

  if (submap_voxel_size_ <= 0.0) {
    return merged;
  }

  auto filtered = std::make_shared<BASIC::PointCloudType>();
  pcl::VoxelGrid<BASIC::PointType> voxel_filter;
  const float leaf_size = static_cast<float>(submap_voxel_size_);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.setInputCloud(merged);
  voxel_filter.filter(*filtered);
  return filtered;
}

}  // namespace LI2Sup
