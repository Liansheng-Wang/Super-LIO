
#ifndef LATEST_ONLY_BUFFER_HPP
#define LATEST_ONLY_BUFFER_HPP


#include <vector>
#include <atomic>
#include <mutex>
#include <chrono>
#include <condition_variable>


template <typename T, size_t BUFFER_SIZE = 10>
class LatestOnlyBuffer {
public:
LatestOnlyBuffer() : write_index(0), latest_index(-1) {}

  template <typename U>
  void push(U&& data) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      buffer[write_index] = std::forward<U>(data);
      latest_index.store(write_index, std::memory_order_release);
      write_index = (write_index + 1) % BUFFER_SIZE;
    }
    cv_.notify_one();
  }

  bool waitAndGetLatest(T& out) {
    std::unique_lock<std::mutex> lock(mutex_);
    /// 100 HZ
    bool got_data = cv_.wait_for(lock, std::chrono::milliseconds(10), [&]() {
      return latest_index.load(std::memory_order_acquire) != -1;
    });

    if (!got_data) return false; // Timeout without receiving new data.

    int index = latest_index.load(std::memory_order_acquire);
    out = std::move(buffer[index]);
    latest_index.store(-1, std::memory_order_release);  // Clear after consumption.
    return true;
  }

private:
  T buffer[BUFFER_SIZE];
  size_t write_index;
  std::atomic<int> latest_index;
  std::mutex mutex_;
  std::condition_variable cv_;
};


#endif // LATEST_ONLY_BUFFER_HPP
