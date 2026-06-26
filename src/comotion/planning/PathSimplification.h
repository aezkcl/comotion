#pragma once

#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>

namespace comotion {

struct PathSimplificationOptions {
    unsigned int max_shortcut_steps = 128;
    unsigned int max_empty_steps = 32;
    unsigned int max_smooth_steps = 1;
    unsigned int max_passes = 1;
};

namespace detail {

PathSimplificationOptions
normalizePathSimplificationOptions(PathSimplificationOptions options);

bool simplifyPathBounded(
    ompl::geometric::PathGeometric &path,
    const ompl::geometric::PathSimplifierPtr &simplifier,
    PathSimplificationOptions options = {});

bool simplifySolutionBounded(ompl::geometric::SimpleSetup &setup,
                             PathSimplificationOptions options = {});

} // namespace detail
} // namespace comotion
