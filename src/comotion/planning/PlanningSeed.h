#pragma once

#include <cstdint>
#include <limits>

namespace comotion {

/// Matches benchmark app handling of `std::numeric_limits<uint32_t>::max()`.
inline constexpr std::uint32_t kPlanningSeedPassthroughOmpl =
    std::numeric_limits<std::uint32_t>::max();

/// Root seed for `ompl::RNG::setSeed` (same +1 convention as the benchmark apps).
inline std::uint_fast32_t omplRootSeedFromUserPlanningSeed(std::uint32_t user_seed) {
    if (user_seed == kPlanningSeedPassthroughOmpl)
        return user_seed;
    return static_cast<std::uint_fast32_t>(user_seed) + 1u;
}

/// Per-agent deterministic seed for `ompl::RNG` instances (local / per-tree RNG).
inline std::uint_fast32_t omplLocalSeedFromUserPlanningSeed(std::uint32_t user_seed,
                                                            int agent_index) {
    std::uint64_t x = static_cast<std::uint64_t>(user_seed) +
                      0x9e3779b97f4a7c15ULL *
                          static_cast<std::uint64_t>(agent_index + 1);
    x ^= x >> 33u;
    x *= 0xff51afd7ed558ccdULL;
    x ^= x >> 33u;
    return static_cast<std::uint_fast32_t>(x);
}

/// Salt for `omplLocalSeedFromUserPlanningSeed` reserved for MRdRRT phase-2 (tensor) sampling.
inline constexpr int kOmplLocalSaltMrDrrtTensorPhase = 1000000001;

inline std::uint_fast32_t omplLocalSeedForMrDrrtTensorPhase(std::uint32_t user_seed) {
    return omplLocalSeedFromUserPlanningSeed(user_seed, kOmplLocalSaltMrDrrtTensorPhase);
}

/// Salt base for process-level OR-parallel planner worker seeds.
inline constexpr int kPlanningSeedSaltOrParallelWorkerBase = 1001000000;

/// Deterministic per-worker planner seed for process-level OR-parallel replicas.
inline std::uint32_t orParallelWorkerPlanningSeed(std::uint32_t user_seed,
                                                  int worker_index) {
    return static_cast<std::uint32_t>(omplLocalSeedFromUserPlanningSeed(
        user_seed, kPlanningSeedSaltOrParallelWorkerBase + worker_index));
}

/// Seed for `std::mt19937` in planners that keep their own C++ RNG.
inline std::uint32_t stdRngSeedFromUserPlanningSeed(std::uint32_t user_seed) {
    return user_seed;
}

} // namespace comotion
