#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace comotion::detail {

struct StrrtBatchInflationResult {
    double direct_goal_lower_bound = 0.0;
    std::size_t virtual_expansions = 0;
    unsigned int base_batch_size = 0;
    unsigned int inflated_batch_size = 0;
};

inline std::size_t computeVirtualGoalTimeExpansions(
    double direct_goal_lower_bound, double minimum_goal_time,
    double initial_time_bound_factor, double time_bound_factor_increase) {
    if (!(direct_goal_lower_bound > 0.0) || !(minimum_goal_time > 0.0) ||
        !(initial_time_bound_factor > 0.0) ||
        !(time_bound_factor_increase > 1.0)) {
        return 0;
    }

    const double initial_reachable_upper =
        direct_goal_lower_bound * initial_time_bound_factor;
    if (!(minimum_goal_time > initial_reachable_upper)) {
        return 0;
    }

    const double ratio = minimum_goal_time / initial_reachable_upper;
    if (!(ratio > 1.0)) {
        return 0;
    }

    const double raw =
        std::log(ratio) / std::log(time_bound_factor_increase);
    return static_cast<std::size_t>(std::max(
        0.0, std::ceil(raw - std::numeric_limits<double>::epsilon())));
}

inline unsigned int inflateInitialBatchSize(
    unsigned int base_batch_size, std::size_t virtual_expansions,
    double time_bound_factor_increase,
    unsigned int max_inflated_batch_multiplier) {
    const auto safe_base = std::max(1u, base_batch_size);
    const auto safe_multiplier = std::max(1u, max_inflated_batch_multiplier);
    if (virtual_expansions == 0 || !(time_bound_factor_increase > 1.0)) {
        return safe_base;
    }

    const std::uint64_t cap =
        static_cast<std::uint64_t>(safe_base) * safe_multiplier;
    std::uint64_t batch = safe_base;
    for (std::size_t i = 0; i < virtual_expansions; ++i) {
        const double grown = std::ceil(
            2.0 * (time_bound_factor_increase - 1.0) *
            static_cast<double>(batch));
        const std::uint64_t candidate = static_cast<std::uint64_t>(
            std::max<double>(1.0, grown));
        batch = std::min(cap, candidate);
    }

    return static_cast<unsigned int>(std::min<std::uint64_t>(
        batch, std::numeric_limits<unsigned int>::max()));
}

inline StrrtBatchInflationResult computeStrrtBatchInflation(
    double direct_goal_lower_bound, double minimum_goal_time,
    unsigned int base_batch_size, double initial_time_bound_factor,
    double time_bound_factor_increase,
    unsigned int max_inflated_batch_multiplier) {
    StrrtBatchInflationResult result;
    result.direct_goal_lower_bound = std::max(0.0, direct_goal_lower_bound);
    result.base_batch_size = std::max(1u, base_batch_size);
    result.virtual_expansions = computeVirtualGoalTimeExpansions(
        result.direct_goal_lower_bound, minimum_goal_time,
        initial_time_bound_factor, time_bound_factor_increase);
    result.inflated_batch_size = inflateInitialBatchSize(
        result.base_batch_size, result.virtual_expansions,
        time_bound_factor_increase, max_inflated_batch_multiplier);
    return result;
}

} // namespace comotion::detail
