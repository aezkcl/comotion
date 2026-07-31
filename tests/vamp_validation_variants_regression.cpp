#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/detail/VampPackingUtils.h"
#include "comotion/planning/Path.h"
#include "comotion/robot/FlyingSphere.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

using comotion::CollisionChecker;
using comotion::CompositeConflict;
using comotion::CompositePathValidationOptions;
using comotion::ConflictScope;
using comotion::FlyingSphere;
using comotion::Path;
using comotion::VampBatchOrdering;
using comotion::VampBatchPacking;
using comotion::VampValidationStrategy;

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "vamp_validation_variants_regression: expected true for "
                  << label << "\n";
        return false;
    }
    return true;
}

bool expectEqual(const std::string &label, std::size_t actual,
                 std::size_t expected) {
    if (actual != expected) {
        std::cerr << "vamp_validation_variants_regression: " << label
                  << " expected " << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

bool expectEqual(const std::string &label, int actual, int expected) {
    if (actual != expected) {
        std::cerr << "vamp_validation_variants_regression: " << label
                  << " expected " << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

bool expectScope(const std::string &label, ConflictScope actual,
                 ConflictScope expected) {
    if (actual != expected) {
        std::cerr << "vamp_validation_variants_regression: " << label
                  << " expected scope " << static_cast<int>(expected)
                  << " got " << static_cast<int>(actual) << "\n";
        return false;
    }
    return true;
}

bool expectVector(const std::string &label, const std::vector<std::size_t> &actual,
                  const std::vector<std::size_t> &expected) {
    if (actual != expected) {
        std::cerr << "vamp_validation_variants_regression: " << label
                  << " mismatch\n";
        return false;
    }
    return true;
}

auto makeRobot() -> std::shared_ptr<FlyingSphere> {
    return std::make_shared<FlyingSphere>(0.6, -10.0, 10.0);
}

auto makeInterRobotConflictPathA() -> Path {
    return Path{
        {-5.0, 0.0, 0.0},
        {-4.0, 0.0, 0.0},
        {-3.0, 0.0, 0.0},
        {-2.0, 0.0, 0.0},
        {-1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        {3.0, 0.0, 0.0},
        {4.0, 0.0, 0.0},
        {5.0, 0.0, 0.0},
        {6.0, 0.0, 0.0},
    };
}

auto makeInterRobotConflictPathB() -> Path {
    return Path{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
    };
}

auto makeValidPathB() -> Path {
    return Path{
        {0.0, 3.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 3.0, 0.0},
        {0.0, 3.0, 0.0},
    };
}

auto makePackOrderingConflictPathA() -> Path {
    Path path(24, {5.0, 0.0, 0.0});
    path[16] = {0.0, 0.0, 0.0};
    return path;
}

auto makePackOrderingConflictPathB() -> Path {
    return Path(24, {0.0, 0.0, 0.0});
}

std::vector<std::size_t> flattenPacks(
    const std::vector<comotion::detail::VampPackingLayout> &packs) {
    std::vector<std::size_t> timesteps;
    for (const auto &pack : packs) {
        for (std::size_t lane = 0; lane < pack.lanes; ++lane)
            timesteps.push_back(pack.timesteps[lane]);
    }
    return timesteps;
}

std::vector<std::size_t> expectedLinearOrder(std::size_t begin,
                                             std::size_t end) {
    std::vector<std::size_t> timesteps;
    for (std::size_t timestep = begin; timestep < end; ++timestep)
        timesteps.push_back(timestep);
    return timesteps;
}

std::vector<std::size_t> expectedRakeOrder(std::size_t begin,
                                           std::size_t end) {
    std::vector<std::size_t> timesteps;
    if (begin >= end)
        return timesteps;

    const std::size_t width = comotion::detail::kVampPackingWidth;
    const std::size_t window = end - begin;
    const std::size_t stride = std::max<std::size_t>(
        1, (window + width - 1) / width);
    for (std::size_t offset = 0; offset < stride; ++offset) {
        for (std::size_t lane = 0; lane < width; ++lane) {
            const std::size_t timestep = begin + offset + lane * stride;
            if (timestep >= end)
                break;
            timesteps.push_back(timestep);
        }
    }
    return timesteps;
}

bool runVariantCase(const std::string &label, const VampValidationStrategy &strategy,
                    const std::string &,
                    const std::vector<const comotion::RobotModel *> &robots,
                    const std::vector<Path> &paths,
                    bool expected_valid,
                    std::optional<std::size_t> expected_timestep,
                    std::optional<ConflictScope> expected_scope) {
    CollisionChecker checker(CollisionChecker::Backend::Vamp);
    checker.setVampValidationStrategy(strategy);

    CompositePathValidationOptions options;
    options.check_environment = true;

    const bool path_valid = checker.validateCompositePaths(paths, robots, options);
    if (path_valid != expected_valid) {
        std::cerr << "vamp_validation_variants_regression: " << label
                  << " path validity expected " << expected_valid << " got "
                  << path_valid << "\n";
        return false;
    }
    CompositePathValidationOptions exhaustive_options = options;
    exhaustive_options.exhaustive = true;
    const bool exhaustive_path_valid =
        checker.validateCompositePaths(paths, robots, exhaustive_options);
    if (exhaustive_path_valid != expected_valid) {
        std::cerr << "vamp_validation_variants_regression: " << label
                  << " exhaustive path validity expected " << expected_valid
                  << " got " << exhaustive_path_valid << "\n";
        return false;
    }

    auto path_conflict = checker.findFirstCompositePathConflict(paths, robots, options);
    if (expected_valid) {
        if (path_conflict) {
            std::cerr << "vamp_validation_variants_regression: " << label
                      << " expected no path conflict\n";
            return false;
        }
    } else {
        if (!path_conflict) {
            std::cerr << "vamp_validation_variants_regression: " << label
                      << " expected a path conflict\n";
            return false;
        }
        if (!expectEqual(label + " path timestep", path_conflict->timestep,
                         *expected_timestep) ||
            !expectScope(label + " path scope", path_conflict->scope,
                         *expected_scope)) {
            return false;
        }
    }

    std::vector<std::vector<double>> from;
    std::vector<std::vector<double>> to;
    from.reserve(paths.size());
    to.reserve(paths.size());
    for (const auto &path : paths) {
        from.push_back(path.front());
        to.push_back(path.back());
    }

    CompositePathValidationOptions motion_options = options;
    motion_options.discrete_num_checks_hint = 11;
    const bool motion_valid =
        checker.isCompositeMotionValid(robots, from, to, motion_options);
    if (motion_valid != expected_valid) {
        std::cerr << "vamp_validation_variants_regression: " << label
                  << " motion validity expected " << expected_valid << " got "
                  << motion_valid << "\n";
        return false;
    }
    motion_options.exhaustive = true;
    const bool exhaustive_motion_valid =
        checker.isCompositeMotionValid(robots, from, to, motion_options);
    if (exhaustive_motion_valid != expected_valid) {
        std::cerr << "vamp_validation_variants_regression: " << label
                  << " exhaustive motion validity expected " << expected_valid
                  << " got " << exhaustive_motion_valid << "\n";
        return false;
    }
    const auto work = checker.lastValidationWorkStats();
    if (!expectEqual(label + " exhaustive timesteps checked",
                     work.motion_timesteps_checked,
                     work.motion_timesteps_possible) ||
        !expectEqual(label + " exhaustive robot-state checks",
                     work.robot_state_checks_completed,
                     work.robot_state_checks_possible) ||
        !expectEqual(label + " exhaustive pair checks",
                     work.robot_pair_checks_completed,
                     work.robot_pair_checks_possible)) {
        return false;
    }

    auto motion_conflict =
        checker.findFirstCompositeMotionConflict(robots, from, to, motion_options);
    if (expected_valid) {
        if (motion_conflict) {
            std::cerr << "vamp_validation_variants_regression: " << label
                      << " expected no motion conflict\n";
            return false;
        }
    } else {
        if (!motion_conflict) {
            std::cerr << "vamp_validation_variants_regression: " << label
                      << " expected a motion conflict\n";
            return false;
        }
        if (!expectEqual(label + " motion timestep", motion_conflict->timestep,
                         *expected_timestep) ||
            !expectScope(label + " motion scope", motion_conflict->scope,
                         *expected_scope)) {
            return false;
        }
    }

    return true;
}

bool runBooleanTraversalCase(
    const std::string &label, const VampValidationStrategy &strategy,
    const std::string &,
    const std::vector<const comotion::RobotModel *> &robots,
    const std::vector<Path> &paths, bool expected_valid) {
    CollisionChecker checker(CollisionChecker::Backend::Vamp);
    checker.setVampValidationStrategy(strategy);

    CompositePathValidationOptions options;
    options.check_environment = false;

    const bool valid = checker.validateCompositePaths(paths, robots, options);
    if (valid != expected_valid) {
        std::cerr << "vamp_validation_variants_regression: " << label
                  << " expected validity " << expected_valid << " got " << valid
                  << "\n";
        return false;
    }

    return true;
}

bool runConflictTraversalCase(
    const std::string &label, const VampValidationStrategy &strategy,
    const std::string &,
    const std::vector<const comotion::RobotModel *> &robots,
    const std::vector<Path> &paths, std::size_t expected_timestep) {
    CollisionChecker checker(CollisionChecker::Backend::Vamp);
    checker.setVampValidationStrategy(strategy);

    CompositePathValidationOptions options;
    options.check_environment = false;

    auto conflict = checker.findFirstCompositePathConflict(paths, robots, options);
    if (!conflict) {
        std::cerr << "vamp_validation_variants_regression: " << label
                  << " expected a conflict\n";
        return false;
    }
    if (!expectEqual(label + " timestep", conflict->timestep, expected_timestep))
        return false;

    return true;
}

}  // namespace

int main() {
    const VampValidationStrategy default_strategy;
    if (!expectTrue(
            "default strategy uses combined ordering",
            default_strategy.ordering == VampBatchOrdering::Combined) ||
        !expectTrue("default strategy uses rake packing",
                    default_strategy.packing == VampBatchPacking::Rake)) {
        return 1;
    }

    CollisionChecker default_checker(CollisionChecker::Backend::Vamp);
    const auto checker_default = default_checker.vampValidationStrategy();
    if (!expectTrue(
            "VAMP checker defaults to combined ordering",
            checker_default.ordering == VampBatchOrdering::Combined) ||
        !expectTrue("VAMP checker defaults to rake packing",
                    checker_default.packing == VampBatchPacking::Rake)) {
        return 1;
    }

    const auto linear_layout =
        comotion::detail::makeVampPackingLayouts(0, 20, VampBatchPacking::Linear);
    const auto rake_layout =
        comotion::detail::makeVampPackingLayouts(0, 20, VampBatchPacking::Rake);
    if (!expectVector("linear pack order", flattenPacks(linear_layout),
                      expectedLinearOrder(0, 20)) ||
        !expectVector("rake pack order", flattenPacks(rake_layout),
                      expectedRakeOrder(0, 20))) {
        return 1;
    }

    auto robot_a = makeRobot();
    auto robot_b = makeRobot();
    std::vector<const comotion::RobotModel *> robots{robot_a.get(), robot_b.get()};

    const std::vector<std::pair<VampValidationStrategy, std::string>> variants = {
        {{VampBatchOrdering::Combined, VampBatchPacking::Rake},
         "combined_rake"},
        {{VampBatchOrdering::Combined, VampBatchPacking::Linear},
         "combined_linear"},
        {{VampBatchOrdering::Hierarchical, VampBatchPacking::Rake},
         "hierarchical_rake"},
        {{VampBatchOrdering::Hierarchical, VampBatchPacking::Linear},
         "hierarchical_linear"},
    };

    const std::vector<Path> conflict_paths{
        makeInterRobotConflictPathA(), makeInterRobotConflictPathB()};
    const std::vector<Path> valid_paths{makeInterRobotConflictPathA(),
                                        makeValidPathB()};
    const std::vector<Path> traversal_paths{
        makePackOrderingConflictPathA(), makePackOrderingConflictPathB()};
    const std::size_t traversal_conflict_timestep = 16;
    for (const auto &[strategy, active_key] : variants) {
        if (!runVariantCase(active_key + " conflict", strategy, active_key,
                            robots, conflict_paths, false, 4,
                            ConflictScope::InterRobot)) {
            return 1;
        }
        if (!runVariantCase(active_key + " valid", strategy, active_key, robots,
                            valid_paths, true, std::nullopt, std::nullopt)) {
            return 1;
        }
    }

    if (!runBooleanTraversalCase(
        "combined_rake traversal", {VampBatchOrdering::Combined,
                                     VampBatchPacking::Rake},
        "combined_rake", robots, traversal_paths, false)) {
        return 1;
    }
    if (!runBooleanTraversalCase(
        "combined_linear traversal", {VampBatchOrdering::Combined,
                                       VampBatchPacking::Linear},
        "combined_linear", robots, traversal_paths, false)) {
        return 1;
    }
    if (!runBooleanTraversalCase(
        "hierarchical_rake traversal", {VampBatchOrdering::Hierarchical,
                                         VampBatchPacking::Rake},
        "hierarchical_rake", robots, traversal_paths, false)) {
        return 1;
    }
    if (!runBooleanTraversalCase(
        "hierarchical_linear traversal", {VampBatchOrdering::Hierarchical,
                                           VampBatchPacking::Linear},
        "hierarchical_linear", robots, traversal_paths, false)) {
        return 1;
    }

    if (!runConflictTraversalCase(
        "combined_rake conflict traversal", {VampBatchOrdering::Combined,
                                             VampBatchPacking::Rake},
        "combined_rake", robots, traversal_paths, traversal_conflict_timestep)) {
        return 1;
    }
    if (!runConflictTraversalCase(
        "combined_linear conflict traversal", {VampBatchOrdering::Combined,
                                               VampBatchPacking::Linear},
        "combined_linear", robots, traversal_paths,
        traversal_conflict_timestep)) {
        return 1;
    }
    if (!runConflictTraversalCase(
            "hierarchical_rake conflict traversal",
            {VampBatchOrdering::Hierarchical, VampBatchPacking::Rake},
            "hierarchical_rake", robots, traversal_paths,
            traversal_conflict_timestep)) {
        return 1;
    }
    if (!runConflictTraversalCase(
            "hierarchical_linear conflict traversal",
            {VampBatchOrdering::Hierarchical, VampBatchPacking::Linear},
            "hierarchical_linear", robots, traversal_paths,
            traversal_conflict_timestep)) {
        return 1;
    }

    std::cout << "vamp_validation_variants_regression: OK\n";
    return 0;
}
