#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace ob = ompl::base;

namespace {

std::shared_ptr<comotion::FlyingSphere> makeSphereRobot(double radius = 0.25) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{-10.0, -10.0, -10.0},
        std::vector<double>{10.0, 10.0, 10.0});
}

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "composite_rrt_endpoint_regression: " << label
                  << " expected true\n";
        return false;
    }
    return true;
}

double maxAbsDifference(const std::vector<double> &a,
                        const std::vector<double> &b) {
    if (a.size() != b.size())
        return std::numeric_limits<double>::infinity();
    double max_diff = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
        max_diff = std::max(max_diff, std::abs(a[i] - b[i]));
    return max_diff;
}

bool expectNearConfig(const std::string &label,
                      const std::vector<double> &actual,
                      const std::vector<double> &expected,
                      double tolerance = 1e-6) {
    const double error = maxAbsDifference(actual, expected);
    if (error > tolerance) {
        std::cerr << "composite_rrt_endpoint_regression: " << label
                  << " max abs error " << error << " exceeds tolerance "
                  << tolerance << "\n";
        return false;
    }
    return true;
}

bool expectJsonCondition(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "composite_rrt_endpoint_regression: " << label << "\n";
        return false;
    }
    return true;
}

bool testCompositePathConflictHandlesRaggedPaths() {
    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Spheres);
    auto robot_a = makeSphereRobot(0.49);
    auto robot_b = makeSphereRobot(0.49);

    comotion::Path path_a;
    path_a.push_back({-3.0, 0.0, 0.0});
    path_a.push_back({-2.0, 0.0, 0.0});
    path_a.push_back({-1.0, 0.0, 0.0});
    path_a.push_back({0.0, 0.0, 0.0});
    path_a.push_back({1.0, 0.0, 0.0});
    path_a.waypoint_timesteps_ = {0, 1, 2, 3, 4};

    comotion::Path path_b;
    path_b.push_back({0.0, 3.0, 0.0});
    path_b.push_back({0.0, 0.0, 0.0});
    path_b.waypoint_timesteps_ = {0, 1};

    if (!expectTrue("paths are intentionally ragged", path_a.size() != path_b.size()))
        return false;

    std::vector<comotion::Path> paths{path_a, path_b};
    std::vector<const comotion::RobotModel *> robots{robot_a.get(), robot_b.get()};
    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    auto conflict = checker.findFirstCompositePathConflict(paths, robots, options);
    if (!expectTrue("ragged composite path conflict found", conflict.has_value()))
        return false;
    if (!expectJsonCondition(
            "ragged conflict detected after shorter path arrival",
            conflict->timestep == 3))
        return false;
    if (!expectJsonCondition("ragged conflict robot_i == 0", conflict->robot_i == 0))
        return false;
    if (!expectJsonCondition("ragged conflict robot_j == 1", conflict->robot_j == 1))
        return false;

    return true;
}

bool testCompositeRrtMatchesProblemEndpoints() {
    comotion::seedOmplGlobalFromUserPlanningSeed(6);

    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(32);
    problem->setVmax(2.0);

    problem->addRobot(makeSphereRobot(), {-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {0.0, -2.0, 0.0}, {0.0, 2.0, 0.0});

    comotion::CompositeRRT planner;
    planner.setProblem(problem);
    planner.setSimplifySolution(false);

    const auto status = planner.solve(2.0);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "composite_rrt_endpoint_regression: expected exact "
                     "solution, got "
                  << status.asString() << "\n";
        return false;
    }

    const auto paths = planner.getSolutionPaths();
    if (!expectTrue("solution path count == 2", paths.size() == 2))
        return false;

    for (size_t i = 0; i < paths.size(); ++i) {
        const auto &path = paths[i];
        if (!expectTrue("path non-empty", !path.empty()))
            return false;
        if (!expectNearConfig("path starts at robot start",
                              path.front(), problem->robot(static_cast<int>(i)).start))
            return false;
        if (!expectNearConfig("path ends at robot goal",
                              path.back(), problem->robot(static_cast<int>(i)).goal))
            return false;
    }

    return true;
}

} // namespace

int main() {
    if (!testCompositePathConflictHandlesRaggedPaths())
        return 1;
    if (!testCompositeRrtMatchesProblemEndpoints())
        return 1;

    std::cout << "composite_rrt_endpoint_regression: OK\n";
    return 0;
}
