#ifndef SUPER_LIO_KEYFRAME_MANAGER_H_
#define SUPER_LIO_KEYFRAME_MANAGER_H_

#include <cstdint>
#include <optional>
#include <vector>

#include "basic/alias.h"
#include "common/ds.h"

namespace LI2Sup
{

struct LoopKeyframe
{
  std::uint32_t id = 0;
  NavState state;
  BASIC::CloudPtr cloud;
};

class KeyframeManager
{
public:
  KeyframeManager(double translation_threshold, double rotation_threshold_deg);

  std::optional<LoopKeyframe> consider(
    const NavState & state, const BASIC::CloudPtr & body_cloud);

  const std::vector<LoopKeyframe> & keyframes() const
  {
    return keyframes_;
  }

private:
  bool shouldCreate(const NavState & state) const;

  double translation_threshold_;
  double rotation_threshold_rad_;
  std::vector<LoopKeyframe> keyframes_;
};

}  // namespace LI2Sup

#endif
