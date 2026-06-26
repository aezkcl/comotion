#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/ConflictChecker.h"
#include "comotion/planning/Path.h"
#include "comotion/robot/FlyingSphere.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "vamp_conflict_scan_regression: " << label << "\n";
        return false;
    }
    return true;
}

comotion::Path makePathWithConflictsAt(std::size_t length,
                                   const std::vector<std::size_t> &timesteps) {
    comotion::Path path(length, {5.0, 0.0, 0.0});
    for (const auto timestep : timesteps) {
        if (timestep < path.size())
            path[timestep] = {0.0, 0.0, 0.0};
    }
    return path;
}

comotion::Path makeConstantPath(std::size_t length,
                            const std::vector<double> &config) {
    return comotion::Path(length, config);
}

std::shared_ptr<comotion::FlyingSphere> makeSphereRobot() {
    return std::make_shared<comotion::FlyingSphere>(0.4, -10.0, 10.0);
}

bool runCase(std::size_t t_begin, std::size_t expected_timestep) {
    auto robot_a = makeSphereRobot();
    auto robot_b = makeSphereRobot();
    std::vector<const comotion::RobotModel *> robots{robot_a.get(), robot_b.get()};
    std::vector<comotion::Path> paths{
        makePathWithConflictsAt(24, {5, 16}),
        makeConstantPath(24, {0.0, 0.0, 0.0}),
    };

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    options.t_begin = t_begin;

    comotion::CollisionChecker pair_checker(comotion::CollisionChecker::Backend::Vamp);
    const auto pair = pair_checker.findFirstPairPathConflict(
        *robot_a, paths[0], *robot_b, paths[1], t_begin, paths[0].size());
    if (!expectTrue("findFirstPair returned a conflict", pair.has_value()))
        return false;
    if (!expectTrue("findFirstPair timestep is chronological first",
                    pair->timestep == expected_timestep)) {
        return false;
    }

    comotion::CollisionChecker first_checker(comotion::CollisionChecker::Backend::Vamp);
    const auto first = first_checker.findFirstCompositePathConflict(
        paths, robots, options);
    if (!expectTrue("findFirst returned a conflict", first.has_value()))
        return false;
    if (!expectTrue("findFirst timestep is chronological first",
                    first->timestep == expected_timestep &&
                        first->robot_i == 0 && first->robot_j == 1)) {
        return false;
    }

    comotion::CollisionChecker conflict_checker_backend(
        comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker conflict_checker(conflict_checker_backend);
    const auto single = conflict_checker.findConflict(
        paths, robots, options, 0);
    if (!expectTrue("ConflictChecker::findConflict returned a conflict",
                    single.has_value()))
        return false;
    if (!expectTrue("findConflict matches findFirst",
                    single->timestep == static_cast<int>(expected_timestep) &&
                        single->robot_i == first->robot_i &&
                        single->robot_j == first->robot_j)) {
        return false;
    }

    comotion::CollisionChecker multi_backend(comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker multi_checker(multi_backend);
    const auto conflicts = multi_checker.findConflicts(
        paths, robots, options, 0, 1, false, {});
    if (!expectTrue("findConflicts N=1 returned one conflict",
                    conflicts.size() == 1))
        return false;
    const auto &conflict = conflicts.front();
    if (!expectTrue("findConflicts N=1 matches findFirst",
                    conflict.conflict_timestep ==
                            static_cast<int>(expected_timestep) &&
                        conflict.seed_robot_i == first->robot_i &&
                        conflict.seed_robot_j == first->robot_j &&
                        conflict.robots == std::vector<int>({0, 1}) &&
                        conflict.window_begin_t ==
                            static_cast<int>(expected_timestep) &&
                        conflict.window_end_t ==
                            static_cast<int>(expected_timestep))) {
        return false;
    }
    return true;
}

bool runCrossPairOrderingCase() {
    auto robot_a = makeSphereRobot();
    auto robot_b = makeSphereRobot();
    auto robot_c = makeSphereRobot();
    std::vector<const comotion::RobotModel *> robots{
        robot_a.get(), robot_b.get(), robot_c.get()};

    auto path_a = makeConstantPath(24, {10.0, 0.0, 0.0});
    auto path_b = makeConstantPath(24, {20.0, 0.0, 0.0});
    auto path_c = makeConstantPath(24, {30.0, 0.0, 0.0});
    path_a[16] = {0.0, 0.0, 0.0};
    path_b[5] = {0.0, 0.0, 0.0};
    path_b[16] = {0.0, 0.0, 0.0};
    path_c[5] = {0.0, 0.0, 0.0};
    std::vector<comotion::Path> paths{path_a, path_b, path_c};

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    options.t_begin = 0;

    comotion::CollisionChecker first_checker(comotion::CollisionChecker::Backend::Vamp);
    const auto first = first_checker.findFirstCompositePathConflict(
        paths, robots, options);
    if (!expectTrue("cross-pair findFirst returned a conflict",
                    first.has_value()))
        return false;
    if (!expectTrue("cross-pair findFirst returns earliest pair",
                    first->timestep == 5 && first->robot_i == 1 &&
                        first->robot_j == 2)) {
        return false;
    }

    comotion::CollisionChecker multi_backend(comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker multi_checker(multi_backend);
    const auto conflicts = multi_checker.findConflicts(
        paths, robots, options, 0, 1, false, {});
    if (!expectTrue("cross-pair findConflicts N=1 returned one conflict",
                    conflicts.size() == 1))
        return false;
    const auto &conflict = conflicts.front();
    if (!expectTrue("cross-pair findConflicts N=1 matches findFirst",
                    conflict.conflict_timestep == 5 &&
                        conflict.seed_robot_i == first->robot_i &&
                        conflict.seed_robot_j == first->robot_j &&
                        conflict.robots == std::vector<int>({1, 2}))) {
        return false;
    }
    return true;
}

bool runExpandedUniqueCase() {
    constexpr std::size_t kLength = 24;
    std::vector<std::shared_ptr<comotion::FlyingSphere>> robot_storage;
    for (int i = 0; i < 4; ++i)
        robot_storage.push_back(makeSphereRobot());

    std::vector<const comotion::RobotModel *> robots;
    for (const auto &robot : robot_storage)
        robots.push_back(robot.get());

    auto path_a = makeConstantPath(kLength, {0.0, 0.0, 0.0});
    auto path_b = makeConstantPath(kLength, {10.0, 0.0, 0.0});
    auto path_c = makeConstantPath(kLength, {20.0, 0.0, 0.0});
    auto path_d = makeConstantPath(kLength, {5.0, 0.0, 0.0});
    path_b[5] = {0.0, 0.0, 0.0};
    path_c[6] = {5.0, 0.0, 0.0};
    std::vector<comotion::Path> paths{path_a, path_b, path_c, path_d};

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;

    comotion::CollisionChecker backend(comotion::CollisionChecker::Backend::Vamp);
    comotion::ConflictChecker checker(backend);
    const auto conflicts = checker.findConflicts(
        paths, robots, options, 0, 0, true,
        [](const comotion::Conflict &conflict) {
            comotion::SubproblemConflict expanded;
            expanded.robots = {conflict.robot_i, conflict.robot_j};
            if (conflict.robot_i == 0 && conflict.robot_j == 1)
                expanded.robots = {0, 1, 2};
            expanded.conflict_timestep = conflict.timestep;
            expanded.window_begin_t = conflict.timestep;
            expanded.window_end_t = conflict.timestep;
            expanded.seed_robot_i = conflict.robot_i;
            expanded.seed_robot_j = conflict.robot_j;
            expanded.alpha = conflict.alpha;
            expanded.kind = conflict.kind;
            expanded.config_i = conflict.config_i;
            expanded.config_j = conflict.config_j;
            return expanded;
        });

    if (!expectTrue("expanded unique conflict count", conflicts.size() == 1))
        return false;
    return expectTrue("expanded unique claims transitive robot",
                      conflicts.front().robots == std::vector<int>({0, 1, 2}) &&
                          conflicts.front().conflict_timestep == 5);
}

} // namespace

int main() {
    if (!runCase(0, 5))
        return 1;
    if (!runCase(6, 16))
        return 1;
    if (!runCrossPairOrderingCase())
        return 1;
    if (!runExpandedUniqueCase())
        return 1;

    std::cout << "vamp_conflict_scan_regression: OK\n";
    return 0;
}
