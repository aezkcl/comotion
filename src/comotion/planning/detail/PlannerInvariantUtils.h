#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace comotion::detail {

inline double maxAbsDifference(const std::vector<double> &a,
                               const std::vector<double> &b) {
    if (a.size() != b.size())
        return std::numeric_limits<double>::infinity();

    double max_diff = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
        max_diff = std::max(max_diff, std::abs(a[i] - b[i]));
    return max_diff;
}

inline void requireConfigNear(const std::vector<double> &actual,
                              const std::vector<double> &expected,
                              double tolerance,
                              const std::string &context) {
    const double error = maxAbsDifference(actual, expected);
    if (error <= tolerance)
        return;

    std::ostringstream msg;
    msg << context << ": max abs diff " << error
        << " exceeds tolerance " << tolerance;
    throw std::runtime_error(msg.str());
}

inline std::int64_t signedTimestepDelta(std::size_t from_timestep,
                                        std::size_t to_timestep,
                                        const std::string &context) {
    constexpr std::size_t kMaxSignedTimestep =
        static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max());

    if (from_timestep > kMaxSignedTimestep || to_timestep > kMaxSignedTimestep) {
        std::ostringstream msg;
        msg << context << ": timestep exceeds signed delta range";
        throw std::runtime_error(msg.str());
    }

    return static_cast<std::int64_t>(to_timestep) -
           static_cast<std::int64_t>(from_timestep);
}

inline std::size_t applySignedTimestepShift(std::size_t timestep,
                                            std::int64_t delta,
                                            const std::string &context) {
    constexpr std::size_t kMaxSignedTimestep =
        static_cast<std::size_t>(std::numeric_limits<std::int64_t>::max());

    if (timestep > kMaxSignedTimestep) {
        std::ostringstream msg;
        msg << context << ": timestep exceeds signed shift range";
        throw std::runtime_error(msg.str());
    }

    const auto signed_timestep = static_cast<std::int64_t>(timestep);
    const auto signed_min = std::numeric_limits<std::int64_t>::min();
    const auto signed_max = std::numeric_limits<std::int64_t>::max();

    if ((delta > 0 && signed_timestep > signed_max - delta) ||
        (delta < 0 && signed_timestep < signed_min - delta)) {
        std::ostringstream msg;
        msg << context << ": signed timestep shift overflow";
        throw std::runtime_error(msg.str());
    }

    const auto shifted = signed_timestep + delta;
    if (shifted < 0) {
        std::ostringstream msg;
        msg << context << ": shifted timestep underflow";
        throw std::runtime_error(msg.str());
    }

    return static_cast<std::size_t>(shifted);
}

} // namespace comotion::detail
