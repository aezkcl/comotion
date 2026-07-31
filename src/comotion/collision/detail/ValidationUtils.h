#pragma once

#include "comotion/planning/Path.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace comotion {
namespace detail {

inline std::size_t pathTimestepCount(const Path &path) {
    if (path.empty())
        return 0;
    return path.arrival_timestep() + 1;
}

// Composite path scans treat shorter paths as holding their final configuration
// after arrival instead of requiring prior equalizePaths() padding. Sparse paths
// are sampled by native timestep without materializing the whole horizon.
inline void configAt(const Path &path, std::size_t t,
                     std::vector<double> &out) {
    path.config_at_timestep(t, out);
}

inline std::vector<double> configAt(const Path &path, std::size_t t) {
    return path.config_at_timestep(t);
}

inline std::size_t maxPathLength(const std::vector<Path> &paths) {
    std::size_t max_t = 0;
    for (const auto &path : paths)
        max_t = std::max(max_t, pathTimestepCount(path));
    return max_t;
}

inline int resolutionAwareMotionCheckCount(
    int spatial_checks, double max_distance, std::size_t resolution,
    double vmax, int minimum_checks = 1) {
    int checks = std::max(minimum_checks, spatial_checks);
    if (max_distance <= 0.0 || resolution == 0 || vmax <= 0.0)
        return checks;

    const double timestep_checks =
        std::ceil(max_distance * static_cast<double>(resolution) / vmax);
    if (timestep_checks >= static_cast<double>(std::numeric_limits<int>::max()))
        return std::numeric_limits<int>::max();
    return std::max(checks, static_cast<int>(timestep_checks));
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
