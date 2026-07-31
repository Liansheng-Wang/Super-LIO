#include "lio/keyframe_manager.h"

#include <algorithm>
#include <cmath>

namespace LI2Sup
{

KeyframeManager::KeyframeManager(
  double translation_threshold, double rotation_threshold_deg)
: translation_threshold_(std::max(0.0, translation_threshold)),
  rotation_threshold_rad_(
    std::max(0.0, rotation_threshold_deg) * std::acos(-1.0) / 180.0)
{
}

bool KeyframeManager::shouldCreate(const NavState & state) const
{
  if (keyframes_.empty()) {
    return true;
  }

  const auto & previous = keyframes_.back().state;
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

std::optional<LoopKeyframe> KeyframeManager::consider(
  const NavState & state, const BASIC::CloudPtr & body_cloud)
{
  if (!body_cloud || body_cloud->empty() || !shouldCreate(state)) {
    return std::nullopt;
  }

  LoopKeyframe keyframe;
  keyframe.id = static_cast<std::uint32_t>(keyframes_.size());
  keyframe.state = state;
  keyframe.cloud = std::make_shared<BASIC::PointCloudType>(*body_cloud);
  keyframes_.push_back(keyframe);
  return keyframe;
}

}  // namespace LI2Sup
