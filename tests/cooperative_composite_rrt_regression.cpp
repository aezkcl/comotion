#include "comotion/planning/CooperativeCompositeRRT.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/robot/FlyingSphere.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace ob = ompl::base;

namespace {

bool expectTrue(const std::string &label, bool condition) {
    if (!condition) {
        std::cerr << "cooperative_composite_rrt_regression: " << label
                  << "\n";
        return false;
    }
    return true;
}

std::shared_ptr<comotion::FlyingSphere> makeSphere(double radius = 0.25) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{-4.0, -4.0, 0.0},
        std::vector<double>{4.0, 4.0, 0.0});
}

bool nearConfig(const std::vector<double> &a, const std::vector<double> &b) {
    if (a.size() != b.size())
        return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (std::fabs(a[i] - b[i]) > 1e-6)
            return false;
    }
    return true;
}

bool runSingleSphere(unsigned threads) {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(16);
    problem->setVmax(2.0);
    problem->addRobot(makeSphere(), {-3.0, 0.0, 0.0}, {3.0, 0.0, 0.0});

    comotion::CooperativeCompositeRRT planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(101);
    planner.setSimplifySolution(false);
    planner.setWorkerThreads(threads);

    const auto status = planner.solve(2.0);
    if (!expectTrue("single sphere exact",
                    status == ob::PlannerStatus::EXACT_SOLUTION))
        return false;

    const auto paths = planner.getSolutionPaths();
    if (!expectTrue("single sphere path count", paths.size() == 1))
        return false;
    if (!expectTrue("single sphere starts at endpoint",
                    nearConfig(paths[0].front(), {-3.0, 0.0, 0.0})))
        return false;
    if (!expectTrue("single sphere reaches endpoint",
                    nearConfig(paths[0].back(), {3.0, 0.0, 0.0})))
        return false;

    const auto &stats = planner.plannerStatsJson();
    if (!expectTrue("stats contains cooperative section",
                    stats.contains("cooperative_composite_rrt")))
        return false;
    const auto &rrt_stats = stats["cooperative_composite_rrt"];
    if (!expectTrue("stats reports configured worker threads",
                    rrt_stats["worker_threads"].get<unsigned>() == threads))
        return false;
    return expectTrue("stats reports iterations",
                      rrt_stats["iterations"].get<std::uint64_t>() > 0);
}

bool runCrossingSpheres() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(24);
    problem->setVmax(2.0);
    problem->addRobot(makeSphere(0.2), {-3.0, 0.0, 0.0}, {3.0, 0.0, 0.0});
    problem->addRobot(makeSphere(0.2), {0.0, -3.0, 0.0}, {0.0, 3.0, 0.0});

    comotion::CooperativeCompositeRRT planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(202);
    planner.setSimplifySolution(false);
    planner.setWorkerThreads(2);

    const auto status = planner.solve(5.0);
    if (!expectTrue("crossing spheres exact",
                    status == ob::PlannerStatus::EXACT_SOLUTION))
        return false;

    const auto paths = planner.getSolutionPaths();
    if (!expectTrue("crossing path count", paths.size() == 2))
        return false;
    auto ptrs = problem->robotModelPtrs();
    comotion::CompositePathValidationOptions options;
    options.check_environment = true;
    const auto conflict = problem->collisionChecker().findFirstCompositePathConflict(
        paths, ptrs, options);
    return expectTrue("crossing paths are conflict-free", !conflict.has_value());
}

} // namespace

int main() {
    if (!runSingleSphere(1))
        return 1;
    if (!runSingleSphere(2))
        return 1;
    if (!runCrossingSpheres())
        return 1;

    std::cout << "cooperative_composite_rrt_regression: OK\n";
    return 0;
}
