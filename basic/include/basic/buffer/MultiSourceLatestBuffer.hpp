#ifndef MULTI_SOURCE_LATEST_BUFFER_HPP
#define MULTI_SOURCE_LATEST_BUFFER_HPP


#include <vector>
#include <atomic>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <optional>
#include <unordered_map>

/**
 * @brief A high-responsiveness buffer for latest-state synchronization.
 *        Only the most recent data is kept, with support for multiple sources. 
 * @tparam T 
 * @tparam BUFFER_SIZE 
 */
template <typename T, size_t BUFFER_SIZE = 10>
class MultiSourceLatestBuffer {
public:
  MultiSourceLatestBuffer() = default;

  // Push a state update from a specified source.
  template <typename U>
  void push(int source_id, U&& data) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      size_t index = write_index_map_[source_id] % BUFFER_SIZE;
      buffers_[source_id][index] = std::forward<U>(data);
      latest_index_map_[source_id].store(index, std::memory_order_release);
      write_index_map_[source_id] = (write_index_map_[source_id] + 1) % BUFFER_SIZE;
    }
    cv_.notify_all();  // Notify all waiting threads (multiple sources).
  }

  // Wait for and retrieve the latest state from a given source (with timeout).
  bool waitAndGetLatest(int source_id, T& out, std::chrono::milliseconds timeout = std::chrono::milliseconds(10)) {
    std::unique_lock<std::mutex> lock(mutex_);

    bool got_data = cv_.wait_for(lock, timeout, [&]() {
      return latest_index_map_.count(source_id) > 0 &&
             latest_index_map_[source_id].load(std::memory_order_acquire) != -1;
    });

    if (!got_data) return false;

    int index = latest_index_map_[source_id].load(std::memory_order_acquire);
    out = std::move(buffers_[source_id][index]);
    latest_index_map_[source_id].store(-1, std::memory_order_release);
    return true;
  }

private:
  // Each source maintains its own ring buffer.
  std::unordered_map<int, std::array<T, BUFFER_SIZE>> buffers_;
  std::unordered_map<int, std::atomic<int>> latest_index_map_;
  std::unordered_map<int, size_t> write_index_map_;
  
  std::mutex mutex_;
  std::condition_variable cv_;
};

#endif // MULTI_SOURCE_LATEST_BUFFER_HPP