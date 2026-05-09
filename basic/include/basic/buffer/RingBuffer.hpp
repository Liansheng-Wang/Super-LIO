
#ifndef BASIC_BUFFER_RINGBUFFER_HPP
#define BASIC_BUFFER_RINGBUFFER_HPP

#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <optional>


template <typename T, size_t BUFFER_SIZE = 10>
class RingBuffer {
public:
RingBuffer() : buffer_(BUFFER_SIZE), head_(0), tail_(0), full_(false) {}

  void push(const T& data) {
    std::unique_lock<std::mutex> lock(mutex_);

    buffer_[head_] = data;
    head_ = (head_ + 1) % BUFFER_SIZE;

    if (full_) {
      tail_ = (tail_ + 1) % BUFFER_SIZE;
    }

    full_ = head_ == tail_;
    lock.unlock();
    cv_.notify_one();
  }

  void push(T&& data) {
    std::unique_lock<std::mutex> lock(mutex_);

    buffer_[head_] = std::move(data);
    head_ = (head_ + 1) % BUFFER_SIZE;

    /**
     * When the buffer becomes full and new data arrives before old data
     * is consumed, the head index will overtake the tail index.
     *
     * A design question arises: should the tail be advanced to discard
     * old data, or should old data be preserved?
     *
     * Conclusion:
     *   Real-time responsiveness is prioritized over data completeness.
     *   Therefore, the tail is NOT advanced here, and old data may be overwritten.
     *
     * The code below is intentionally commented out to reflect this choice.
     */
    // if (full_) {
    //   tail_ = (tail_ + 1) % BUFFER_SIZE;
    // }

    full_ = head_ == tail_;   /// The newly written data has caught up with old data.
    lock.unlock();
    cv_.notify_one();
  }

  // Block until data is available and pop (no timeout).
  void waitAndPop(T& out) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&]() { return !empty_locked(); });
    out = std::move(buffer_[tail_]);
    tail_ = (tail_ + 1) % BUFFER_SIZE;
    full_ = false;
  }

  // Block with timeout (returns true if data is successfully retrieved).
  bool waitAndPopFor(T& out, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    bool has_data = cv_.wait_for(lock, timeout, [&]() { return !empty_locked(); });
    if (!has_data) return false;
    out = std::move(buffer_[tail_]);
    tail_ = (tail_ + 1) % BUFFER_SIZE;
    full_ = false;
    return true;
  }

  // Non-blocking pop (returns false if the buffer is empty).
  bool tryPop(T& out) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (empty_locked()) return false;
    out = std::move(buffer_[tail_]);
    tail_ = (tail_ + 1) % BUFFER_SIZE;
    full_ = false;
    return true;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (full_) return BUFFER_SIZE;
    if (head_ >= tail_) return head_ - tail_;
    return BUFFER_SIZE + head_ - tail_;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return empty_locked();
  }

private:
  bool empty_locked() const {
    return (!full_ && (head_ == tail_));
  }

  std::vector<T> buffer_;
  size_t head_;
  size_t tail_;
  bool full_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
};


#endif // BASIC_BUFFER_RINGBUFFER_HPP