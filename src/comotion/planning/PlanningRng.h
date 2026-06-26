#pragma once

/// Trial-boundary helpers for OMPL global RNG setup. Planners must not call
/// `ompl::RNG::setSeed` from nested `solve()`; call these from the driver once
/// per trial before constructing OMPL objects for that trial.

#include "comotion/planning/PlanningSeed.h"
#include <ompl/util/RandomNumbers.h>
#include <mutex>
#include <optional>

namespace comotion {

/// Sets OMPL's global root seed from the user planning seed (`+1` mapping in
/// `omplRootSeedFromUserPlanningSeed`, passthrough for `kPlanningSeedPassthroughOmpl`).
///
/// OMPL only applies a meaningful global root **before** any default-constructed
/// `ompl::RNG` has been created in the process. After that, changing the root does
/// not restore full determinism across already-created RNGs. For multiple trials
/// with different seeds in one long-lived process, prefer subprocess-per-trial or
/// rely on explicit `ompl::RNG(localSeed)` / `setLocalSeed` for streams you control.
///
/// If the same `user_seed` is passed again after the first successful call in this
/// process, this is a no-op so drivers (e.g. `--algorithm all`) do not spam OMPL
/// with ineffective `setSeed` calls between back-to-back planners.
inline void seedOmplGlobalFromUserPlanningSeed(std::uint32_t user_seed) {
    static std::mutex mutex;
    static std::optional<std::uint32_t> last_user_seed;
    std::lock_guard<std::mutex> lock(mutex);
    if (last_user_seed.has_value() && *last_user_seed == user_seed)
        return;
    last_user_seed = user_seed;
    ompl::RNG::setSeed(omplRootSeedFromUserPlanningSeed(user_seed));
}

/// If `user_seed` is set, calls `seedOmplGlobalFromUserPlanningSeed`. If nullopt,
/// leaves OMPL's default (non-reproducible) seeding.
inline void seedOmplGlobalIfSpecified(std::optional<std::uint32_t> user_seed) {
    if (user_seed.has_value())
        seedOmplGlobalFromUserPlanningSeed(*user_seed);
}

/// Explicit local `ompl::RNG` stream derived from `(planning_seed, salt)`.
inline ompl::RNG makeOmplRngLocal(std::uint32_t planning_seed, int salt) {
    return ompl::RNG(omplLocalSeedFromUserPlanningSeed(planning_seed, salt));
}

} // namespace comotion
