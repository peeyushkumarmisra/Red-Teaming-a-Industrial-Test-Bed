#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "iiwa_control_law/control_law.hpp"

namespace ids_core {

class ReferenceTrack {
 public:
  static constexpr std::size_t kCapacity = 512;

  void clear() {
    head_ = 0;
    size_ = 0;
  }

  void push(const iiwa_control_law::ReferenceSample& sample) {
    buffer_[head_] = sample;
    head_ = (head_ + 1) % kCapacity;
    if (size_ < kCapacity) {
      ++size_;
    }
  }

  bool empty() const { return size_ == 0; }
  std::size_t size() const { return size_; }

  const iiwa_control_law::ReferenceSample& newest() const { return get(0); }

  const iiwa_control_law::ReferenceSample* at(double t, double latency) const {
    for (std::size_t i = 0; i < size_; ++i) {
      const auto& s = get(i);
      if (s.stamp + latency <= t) {
        return &s;
      }
    }
    return size_ > 0 ? &get(size_ - 1) : nullptr;
  }

  const iiwa_control_law::ReferenceSample* bySeq(std::uint64_t seq) const {
    for (std::size_t i = 0; i < size_; ++i) {
      const auto& s = get(i);
      if (s.seq == seq) {
        return &s;
      }
      if (s.seq < seq) {
        return nullptr;
      }
    }
    return nullptr;
  }

 private:
  const iiwa_control_law::ReferenceSample& get(std::size_t age) const {
    return buffer_[(head_ + kCapacity - 1 - age) % kCapacity];
  }

  std::array<iiwa_control_law::ReferenceSample, kCapacity> buffer_{};
  std::size_t head_ = 0;
  std::size_t size_ = 0;
};

}  // namespace ids_core