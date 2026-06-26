#include "comotion/planning/PathSimplification.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace comotion {
namespace detail {

PathSimplificationOptions
normalizePathSimplificationOptions(PathSimplificationOptions options) {
    options.max_shortcut_steps = std::max(1u, options.max_shortcut_steps);
    if (options.max_empty_steps == 0)
        options.max_empty_steps = options.max_shortcut_steps;
    options.max_passes = std::max(1u, options.max_passes);
    return options;
}

bool simplifyPathBounded(
    ompl::geometric::PathGeometric &path,
    const ompl::geometric::PathSimplifierPtr &simplifier,
    PathSimplificationOptions options) {
    if (!simplifier || path.getStateCount() < 3)
        return false;

    options = normalizePathSimplificationOptions(options);
    bool changed = false;

    for (unsigned int pass = 0; pass < options.max_passes; ++pass) {
        if (path.getStateCount() < 3)
            break;

        const bool metric =
            path.getSpaceInformation()->getStateSpace()->isMetricSpace();
        if (metric) {
            changed |= simplifier->partialShortcutPath(
                path, options.max_shortcut_steps, options.max_empty_steps);
        }

        changed |= simplifier->reduceVertices(path, options.max_shortcut_steps,
                                              options.max_empty_steps);
        changed |= simplifier->collapseCloseVertices(
            path, options.max_shortcut_steps, options.max_empty_steps);

        if (metric && options.max_smooth_steps > 0 &&
            path.getStateCount() >= 3) {
            const double min_change = std::max(
                path.length() / 100.0, std::numeric_limits<double>::epsilon());
            simplifier->smoothBSpline(path, options.max_smooth_steps,
                                      min_change);
        }
    }

    return changed;
}

bool simplifySolutionBounded(ompl::geometric::SimpleSetup &setup,
                             PathSimplificationOptions options) {
    const auto &pdef = setup.getProblemDefinition();
    if (!pdef || !pdef->getSolutionPath())
        return false;

    return simplifyPathBounded(
        static_cast<ompl::geometric::PathGeometric &>(
            *pdef->getSolutionPath()),
        setup.getPathSimplifier(), options);
}

} // namespace detail
} // namespace comotion
