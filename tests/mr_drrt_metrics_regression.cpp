#include "comotion/planning/MRdRRT.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

#include <ompl/base/PlannerStatus.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

namespace ob = ompl::base;

namespace {

std::shared_ptr<comotion::FlyingSphere> makeSphere() {
    return std::make_shared<comotion::FlyingSphere>(
        0.1, std::vector<double>{-10.0, -10.0, -10.0},
        std::vector<double>{10.0, 10.0, 10.0});
}

std::shared_ptr<comotion::MultiRobotProblem> makeDirectProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(16);
    problem->setVmax(1.0);
    problem->addRobot(makeSphere(), {0.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    problem->addRobot(makeSphere(), {0.0, 5.0, 0.0}, {0.0, 6.0, 0.0});
    return problem;
}

bool expectTrue(const std::string &label, bool value) {
    if (!value)
        std::cerr << "mr_drrt_metrics_regression: " << label << "\n";
    return value;
}

bool expectNear(const std::string &label, double actual, double expected,
                double tolerance = 1e-9) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << "mr_drrt_metrics_regression: " << label
                  << " expected " << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

bool runMetricCase(comotion::MRdRRT::CostMetric metric,
                   const std::string &metric_name,
                   double expected_cost) {
    comotion::seedOmplGlobalFromUserPlanningSeed(91);
    auto problem = makeDirectProblem();

    comotion::MRdRRTStar planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(91);
    planner.setRoadmapSize(2);
    planner.setIterationsPerBatch(1);
    planner.setCostMetric(metric);

    const auto status = planner.solve(1.0);
    if (!expectTrue(metric_name + " solve exact",
                    status == ob::PlannerStatus::EXACT_SOLUTION))
        return false;

    const auto &stats = planner.plannerStatsJson();
    if (!expectTrue(metric_name + " stats expose selected metric",
                    stats.contains("cost_metric") &&
                        stats["cost_metric"].get<std::string>() == metric_name))
        return false;
    if (!expectTrue(metric_name + " default tensor search is drrt",
                    stats.contains("tensor_search_mode") &&
                        stats["tensor_search_mode"].get<std::string>() == "drrt"))
        return false;
    if (!expectTrue(metric_name + " dRRT* continues after first solution",
                    stats.contains("stop_at_first_solution") &&
                        !stats["stop_at_first_solution"].get<bool>()))
        return false;
    if (!expectTrue(metric_name + " dRRT* rewiring enabled",
                    stats.contains("use_star_rewiring") &&
                        stats["use_star_rewiring"].get<bool>()))
        return false;
    if (!expectTrue(metric_name + " solution cost numeric",
                    stats.contains("solution_cost") &&
                        stats["solution_cost"].is_number()))
        return false;
    if (!expectNear(metric_name + " solution cost",
                    stats["solution_cost"].get<double>(), expected_cost))
        return false;
    if (!expectTrue(metric_name + " makespan metric populated",
                    planner.makespanTimesteps().has_value()))
        return false;

    return true;
}

bool runAStarCase() {
    comotion::seedOmplGlobalFromUserPlanningSeed(92);
    auto problem = makeDirectProblem();

    comotion::MRdRRTStar planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(92);
    planner.setRoadmapSize(2);
    planner.setIterationsPerBatch(1);
    planner.setCostMetric(comotion::MRdRRT::CostMetric::Makespan);
    planner.setTensorSearchMode(comotion::MRdRRT::TensorSearchMode::AStar);

    const auto status = planner.solve(1.0);
    if (!expectTrue("astar solve exact",
                    status == ob::PlannerStatus::EXACT_SOLUTION))
        return false;

    const auto &stats = planner.plannerStatsJson();
    if (!expectTrue("astar stats expose tensor search mode",
                    stats.contains("tensor_search_mode") &&
                        stats["tensor_search_mode"].get<std::string>() == "astar"))
        return false;
    if (!expectTrue("astar stats expose astar block",
                    stats.contains("astar") && stats["astar"].is_object()))
        return false;
    if (!expectTrue("astar heuristic name",
                    stats["astar"].contains("heuristic") &&
                        stats["astar"]["heuristic"].get<std::string>() ==
                            "max_individual_graph_cost_to_go"))
        return false;
    if (!expectNear("astar solution cost",
                    stats["solution_cost"].get<double>(), 2.0))
        return false;
    if (!expectTrue("astar one path per robot",
                    planner.getSolutionPaths().size() == 2))
        return false;
    if (!expectTrue("astar makespan populated",
                    planner.makespanTimesteps().has_value()))
        return false;
    if (!expectTrue("astar synchronized makespan",
                    *planner.makespanTimesteps() == 32))
        return false;

    return true;
}

bool runLazyAStarCase() {
    comotion::seedOmplGlobalFromUserPlanningSeed(93);
    auto problem = makeDirectProblem();

    comotion::MRdRRTStar planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(93);
    planner.setRoadmapSize(2);
    planner.setIterationsPerBatch(1);
    planner.setCostMetric(comotion::MRdRRT::CostMetric::Makespan);
    planner.setTensorSearchMode(comotion::MRdRRT::TensorSearchMode::LazyAStar);

    const auto status = planner.solve(1.0);
    if (!expectTrue("lazy astar solve exact",
                    status == ob::PlannerStatus::EXACT_SOLUTION))
        return false;

    const auto &stats = planner.plannerStatsJson();
    if (!expectTrue("lazy astar stats expose tensor search mode",
                    stats.contains("tensor_search_mode") &&
                        stats["tensor_search_mode"].get<std::string>() ==
                            "lazy_astar"))
        return false;
    if (!expectTrue("lazy astar stats expose lazy astar block",
                    stats.contains("lazy_astar") &&
                        stats["lazy_astar"].is_object()))
        return false;
    if (!expectTrue("lazy astar validates only candidate path",
                    stats["lazy_astar"].contains("validated_edges") &&
                        stats["lazy_astar"]["validated_edges"].get<std::uint64_t>() ==
                            1u))
        return false;
    if (!expectNear("lazy astar solution cost",
                    stats["solution_cost"].get<double>(), 2.0))
        return false;
    if (!expectTrue("lazy astar synchronized makespan",
                    planner.makespanTimesteps().has_value() &&
                        *planner.makespanTimesteps() == 32))
        return false;

    return true;
}

} // namespace

int main() {
    bool ok = true;
    ok &= runMetricCase(comotion::MRdRRT::CostMetric::SumOfCosts,
                        "sum_of_costs", 3.0);
    ok &= runMetricCase(comotion::MRdRRT::CostMetric::Makespan,
                        "makespan", 2.0);
    ok &= runAStarCase();
    ok &= runLazyAStarCase();
    return ok ? 0 : 1;
}
