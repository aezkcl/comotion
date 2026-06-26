#include "comotion/planning/CompositeRRT.h"
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/ScopedState.h>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace comotion {

namespace {

using Clock = std::chrono::steady_clock;

inline std::uint64_t elapsedNanoseconds(Clock::time_point start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - start)
            .count());
}

} // namespace

ompl::base::PlannerStatus CompositeRRT::solve(double timeLimit) {
    resetPlannerRunMetrics();
    solution_paths_.clear();

    std::vector<int> indices = robot_indices_;
    if (indices.empty()) {
        for (int i = 0; i < problem_->numRobots(); ++i)
            indices.push_back(i);
    }

    auto si = use_makespan_metric_
                  ? problem_->createMakespanCompositeSpaceInfo(indices)
                  : problem_->createCompositeSpaceInfo(indices);
    auto space = si->getStateSpace();

    og::SimpleSetup setup(si);
    auto planner = std::make_shared<og::RRTConnect>(si);
    if (range_ && *range_ > 0.0)
        planner->setRange(*range_);
    setup.setPlanner(planner);

    // Build composite start and goal states
    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    int offset = 0;
    for (int idx : indices) {
        auto &r = problem_->robot(idx);
        int ndof = r.model->numJoints();
        for (int d = 0; d < ndof; ++d) {
            start->as<ompl::base::RealVectorStateSpace::StateType>()
                ->values[offset + d] = r.start[d];
            goal->as<ompl::base::RealVectorStateSpace::StateType>()
                ->values[offset + d] = r.goal[d];
        }
        offset += ndof;
    }

    setup.setStartAndGoalStates(start, goal);

    const auto solve_wall_start = Clock::now();
    ob::PlannerStatus raw_status;
    if (max_rrt_connect_iterations_ == 0 && !cancel_requested_) {
        raw_status = setup.solve(timeLimit);
    } else {
        const unsigned cap = max_rrt_connect_iterations_;
        std::size_t outer_iters = 0;
        auto time_ptc = ob::timedPlannerTerminationCondition(timeLimit);
        auto ptc = ob::PlannerTerminationCondition([&, cap, time_ptc]() {
            return time_ptc || cancellationRequested() ||
                   (cap > 0 && ++outer_iters > static_cast<std::size_t>(cap));
        });
        raw_status = setup.solve(ptc);
    }
    const auto solve_wall_ns = elapsedNanoseconds(solve_wall_start);
    auto exported_status = raw_status;
    if (raw_status == ob::PlannerStatus::APPROXIMATE_SOLUTION)
        exported_status = ob::PlannerStatus::TIMEOUT;
    std::uint64_t simplify_wall_ns = 0;

    if (raw_status == ob::PlannerStatus::EXACT_SOLUTION) {
        if (simplify_solution_) {
            const auto simplify_start = Clock::now();
            setup.simplifySolution();
            simplify_wall_ns = elapsedNanoseconds(simplify_start);
        }
        auto &path = setup.getSolutionPath();
        path.interpolate();

        solution_paths_ = splitCompositePathToRobotPaths(path, *problem_,
                                                         indices,
                                                         "CompositeRRT");
        setSolutionMetricsFromPaths(solution_paths_);
    }

    nlohmann::json stats = nlohmann::json::object();
    stats["solve_wall_seconds"] = static_cast<double>(solve_wall_ns) * 1e-9;
    stats["simplify_wall_seconds"] =
        static_cast<double>(simplify_wall_ns) * 1e-9;
    stats["simplify_solution"] = simplify_solution_;
    stats["rrt_connect_range"] = planner->getRange();
    stats["rrt_connect_range_explicit"] =
        static_cast<bool>(range_ && *range_ > 0.0);
    stats["use_makespan_metric"] = use_makespan_metric_;
    stats["state_space_extent"] = space->getMaximumExtent();
    stats["solution_events"] = nlohmann::json::array();
    if (exported_status == ob::PlannerStatus::EXACT_SOLUTION &&
        makespanTimesteps()) {
        stats["solution_events"].push_back({
            {"elapsed_seconds", static_cast<double>(solve_wall_ns) * 1e-9},
            {"makespan_timesteps", *makespanTimesteps()},
            {"sum_of_cost_timesteps",
             sumOfCostTimesteps() ? nlohmann::json(*sumOfCostTimesteps())
                                  : nlohmann::json(nullptr)},
            {"kind", "first_solution"},
        });
    }
    stats["num_solution_events"] = stats["solution_events"].size();
    setPlannerStatsJson(std::move(stats));

    return exported_status;
}

std::vector<Path> CompositeRRT::getSolutionPaths() const {
    return solution_paths_;
}

} // namespace comotion
