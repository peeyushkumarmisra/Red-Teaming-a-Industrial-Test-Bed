#include <gtest/gtest.h>

#include <atomic>
#include <cstdlib>
#include <fstream>
#include <new>
#include <sstream>
#include <vector>

#include "synthetic_cell.hpp"

namespace {

std::atomic<long> g_allocations{0};
std::atomic<bool> g_counting{false};

void* allocate(std::size_t n, std::size_t alignment) {
  if (g_counting.load(std::memory_order_relaxed)) {
    g_allocations.fetch_add(1, std::memory_order_relaxed);
  }
  const std::size_t size = n == 0 ? 1 : n;
  void* p = nullptr;
  if (alignment <= alignof(std::max_align_t)) {
    p = std::malloc(size);
  } else {
    p = std::aligned_alloc(alignment, (size + alignment - 1) / alignment * alignment);
  }
  if (p == nullptr) throw std::bad_alloc();
  return p;
}

std::string urdf() {
  std::ifstream f(ids_core::test::urdfPath());
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

}  // namespace

void* operator new(std::size_t n) { return allocate(n, alignof(std::max_align_t)); }
void* operator new[](std::size_t n) { return allocate(n, alignof(std::max_align_t)); }
void* operator new(std::size_t n, std::align_val_t a) {
  return allocate(n, static_cast<std::size_t>(a));
}
void* operator new[](std::size_t n, std::align_val_t a) {
  return allocate(n, static_cast<std::size_t>(a));
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }

TEST(Realtime, ArmedProcessingAndReferenceIntakeNeverAllocate) {
  ids_core::IdsConfig cfg;
  ids_core::IdsPipeline ids(cfg, urdf());
  ids_core::Calibration cal;
  cal.fingerprint = ids.fingerprint();
  cal.physics_std = ids_core::Vec7::Constant(0.3);
  ids.arm(cal);
  ids_core::test::SyntheticCell cell;
  ids_core::test::Attack attack;
  attack.kind = ids_core::test::AttackKind::kSetpointStep;
  attack.joint = 2;
  attack.magnitude = 0.05;
  attack.start = 5.0;
  cell.setAttack(attack);
  for (int k = 0; k < 400; ++k) ids.process(cell.step(ids));
  g_allocations = 0;
  bool alarmed = false;
  for (int k = 0; k < 400; ++k) {
    const ids_core::Measurement m = cell.step(ids);
    g_counting = true;
    const auto& out = ids.process(m);
    g_counting = false;
    alarmed = alarmed || out.alarm;
  }
  iiwa_control_law::ReferenceSample sample;
  g_counting = true;
  ids.addReference(sample);
  ids.resetAlarm();
  g_counting = false;
  EXPECT_TRUE(alarmed);
  EXPECT_EQ(g_allocations.load(), 0);
}

std::vector<int>* g_sink = nullptr;

TEST(Realtime, AllocationCounterSeesAllocations) {
  g_allocations = 0;
  g_counting = true;
  g_sink = new std::vector<int>(1000);
  g_counting = false;
  delete g_sink;
  EXPECT_GE(g_allocations.load(), 2);
}