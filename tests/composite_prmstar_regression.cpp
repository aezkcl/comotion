#include "comotion/planning/CompositePRMStar.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

#include <ompl/base/PlannerStatus.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace ob = ompl::base;

namespace {

std::shared_ptr<comotion::FlyingSphere> makeSphereRobot(double radius = 0.2) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{-5.0, -5.0, -5.0},
        std::vector<double>{5.0, 5.0, 5.0});
}

std::shared_ptr<comotion::MultiRobotProblem> makeSeparatedProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(32);
    problem->setVmax(2.0);
    problem->addRobot(makeSphereRobot(), {-2.0, -2.0, 0.75},
                      {-1.0, -2.0, 0.75});
    problem->addRobot(makeSphereRobot(), {2.0, 2.0, 0.75},
                      {1.0, 2.0, 0.75});
    return problem;
}

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "composite_prmstar_regression: " << label << "\n";
        return false;
    }
    return true;
}

double maxAbsDifference(const std::vector<double> &a,
                        const std::vector<double> &b) {
    if (a.size() != b.size())
        return std::numeric_limits<double>::infinity();
    double max_diff = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        max_diff = std::max(max_diff, std::abs(a[i] - b[i]));
    return max_diff;
}

bool expectNearConfig(const std::string &label,
                      const std::vector<double> &actual,
                      const std::vector<double> &expected,
                      double tolerance = 1e-6) {
    const double error = maxAbsDifference(actual, expected);
    if (error > tolerance) {
        std::cerr << "composite_prmstar_regression: " << label
                  << " max abs error " << error << " exceeds tolerance "
                  << tolerance << "\n";
        return false;
    }
    return true;
}

bool expectExactEndpointPaths(const std::vector<comotion::Path> &paths,
                              const comotion::MultiRobotProblem &problem) {
    if (!expectTrue("solution path count == robot count",
                    paths.size() ==
                        static_cast<std::size_t>(problem.numRobots())))
        return false;

    for (std::size_t i = 0; i < paths.size(); ++i) {
        const auto &path = paths[i];
        const auto &robot = problem.robot(static_cast<int>(i));
        if (!expectTrue("path non-empty", !path.empty()))
            return false;
        if (!expectTrue("path has timesteps", path.has_timesteps()))
            return false;
        if (!expectNearConfig("path starts at robot start", path.front(),
                              robot.start))
            return false;
        if (!expectNearConfig("path ends at robot goal", path.back(),
                              robot.goal))
            return false;
    }
    return true;
}

bool solveAndCheck(comotion::CompositePRMStar &planner,
                   const std::shared_ptr<comotion::MultiRobotProblem> &problem,
                   const std::string &expected_metric_mode) {
    planner.setProblem(problem);
    const auto status = planner.solve(1.0);
    if (status != ob::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "composite_prmstar_regression: expected exact solution, got "
                  << status.asString() << "\n";
        return false;
    }
    if (!expectExactEndpointPaths(planner.getSolutionPaths(), *problem))
        return false;

    const auto &stats = planner.plannerStatsJson();
    if (!expectTrue("planner stats include OMPL planner",
                    stats.contains("ompl_planner")))
        return false;
    if (!expectTrue("planner stats OMPL planner is PRMstar",
                    stats["ompl_planner"].get<std::string>() == "PRMstar"))
        return false;
    if (!expectTrue("planner stats include metric mode",
                    stats.contains("metric_mode")))
        return false;
    if (!expectTrue("planner stats metric mode matches",
                    stats["metric_mode"].get<std::string>() ==
                        expected_metric_mode))
        return false;
    if (!expectTrue("planner stats include final solution events",
                    stats.contains("solution_events") &&
                        stats["solution_events"].is_array() &&
                        !stats["solution_events"].empty()))
        return false;
    if (!expectTrue("exact solution flag reported",
                    stats.value("exact_solution_available", false)))
        return false;
    return true;
}

bool testDefaultMakespanMetricAndEndpoints() {
    comotion::CompositePRMStar planner;
    if (!expectTrue("default metric is makespan",
                    planner.metricMode() ==
                        comotion::CompositePRMStar::MetricMode::Makespan))
        return false;
    planner.setSimplifySolution(false);
    comotion::seedOmplGlobalFromUserPlanningSeed(17);
    return solveAndCheck(planner, makeSeparatedProblem(), "makespan");
}

bool testPlainL2MetricModeAndBoundedSimplification() {
    comotion::CompositePRMStar planner;
    planner.setMetricMode(comotion::CompositePRMStar::MetricMode::PlainL2);
    if (!expectTrue("plain L2 metric setter works",
                    planner.metricMode() ==
                        comotion::CompositePRMStar::MetricMode::PlainL2))
        return false;
    comotion::PathSimplificationOptions options;
    options.max_shortcut_steps = 2;
    options.max_empty_steps = 1;
    options.max_smooth_steps = 1;
    planner.setPathSimplificationOptions(options);
    planner.setSimplifySolution(true);

    comotion::seedOmplGlobalFromUserPlanningSeed(23);
    if (!solveAndCheck(planner, makeSeparatedProblem(), "plain_l2"))
        return false;

    const auto &stats = planner.plannerStatsJson();
    if (!expectTrue("path simplification stats exported",
                    stats.contains("path_simplification")))
        return false;
    const auto &path_simplification = stats["path_simplification"];
    if (!expectTrue("bounded simplification enabled",
                    path_simplification["enabled"].get<bool>()))
        return false;
    if (!expectTrue("bounded simplification max steps reported",
                    path_simplification["max_shortcut_steps"]
                            .get<unsigned int>() == 2))
        return false;
    if (!expectTrue("simplification time reported",
                    stats.value("simplify_wall_seconds", -1.0) >= 0.0))
        return false;
    return true;
}

} // namespace

int main() {
    if (!testDefaultMakespanMetricAndEndpoints())
        return 1;
    if (!testPlainL2MetricModeAndBoundedSimplification())
        return 1;

    std::cout << "composite_prmstar_regression: OK\n";
    return 0;
}
