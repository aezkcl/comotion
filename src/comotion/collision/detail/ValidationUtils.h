#pragma once

#include "comotion/planning/Path.h"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace comotion {
namespace detail {

// Composite path scans treat shorter paths as holding their final configuration
// after arrival instead of requiring prior equalizePaths() padding.
inline const std::vector<double> &configAt(const Path &path, std::size_t t) {
    if (t >= path.size())
        return path.back();
    return path[t];
}

inline std::size_t maxPathLength(const std::vector<Path> &paths) {
    std::size_t max_t = 0;
    for (const auto &path : paths)
        max_t = std::max(max_t, path.size());
    return max_t;
}

inline Path makeMotionPath(const std::vector<double> &from,
                           const std::vector<double> &to, int num_checks) {
    const int steps = std::max(1, num_checks);
    Path path;
    path.reserve(static_cast<std::size_t>(steps) + 1);
    for (int step = 0; step <= steps; ++step) {
        const double alpha = static_cast<double>(step) /
                             static_cast<double>(steps);
        path.push_back(interpolateConfig(from, to, alpha));
    }
    return path;
}

inline void interpolateConfigInto(const std::vector<double> &from,
                                  const std::vector<double> &to, double alpha,
                                  std::vector<double> &out) {
    out.resize(from.size());
    for (std::size_t i = 0; i < from.size(); ++i)
        out[i] = from[i] + alpha * (to[i] - from[i]);
}

inline std::vector<Path> makeCompositeMotionPaths(
    const std::vector<std::vector<double>> &from,
    const std::vector<std::vector<double>> &to, int num_checks) {
    std::vector<Path> paths;
    paths.reserve(from.size());
    for (std::size_t i = 0; i < from.size(); ++i)
        paths.push_back(makeMotionPath(from[i], to[i], num_checks));
    return paths;
}

} // namespace detail
} // namespace comotion
