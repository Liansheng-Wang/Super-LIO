#ifndef SUPER_LIO_KEYFRAME_MANAGER_H_
#define SUPER_LIO_KEYFRAME_MANAGER_H_

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>
#include <thread>

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
  using KeyframeCallback = std::function<void(const LoopKeyframe &)>;

  KeyframeManager(
    double translation_threshold, double rotation_threshold_deg,
    std::size_t submap_scan_count, double submap_voxel_size,
    KeyframeCallback keyframe_callback);
  ~KeyframeManager();

  void consider(
    const NavState & state, const BASIC::CloudPtr & body_cloud);
  void stop();

  std::vector<LoopKeyframe> keyframes() const;

private:
  struct BufferedScan
  {
    NavState state;
    BASIC::CloudPtr cloud;
  };

  bool shouldCreate(const NavState & state) const;
  BASIC::CloudPtr buildSubmap(const NavState & state) const;
  void workerLoop();
  void processScan(BufferedScan scan);

  double translation_threshold_;
  double rotation_threshold_rad_;
  std::size_t submap_scan_count_;
  double submap_voxel_size_;
  std::size_t max_pending_scans_;
  KeyframeCallback keyframe_callback_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<BufferedScan> pending_scans_;
  bool stopping_ = false;

  std::vector<BufferedScan> buffered_scans_;
  std::optional<NavState> last_keyframe_state_;
  std::uint32_t next_keyframe_id_ = 0;
  mutable std::mutex keyframes_mutex_;
  std::vector<LoopKeyframe> keyframes_;
  std::thread worker_;
};

}  // namespace LI2Sup

#endif
