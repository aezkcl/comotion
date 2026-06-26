#include "comotion/planning/CompositePRMStar.h"

#include "comotion/planning/detail/PlannerInvariantUtils.h"

#include <nlohmann/json.hpp>
#include <ompl/base/PlannerData.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace comotion {
namespace {

using Clock = std::chrono::steady_clock;

struct RobotBlock {
    int robot_index = -1;
    int offset = 0;
    int ndof = 0;
};

std::uint64_t elapsedNanoseconds(Clock::time_point start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - start)
            .count());
}

std::vector<int> defaultRobotIndices(const MultiRobotProblem &problem,
                                     std::vector<int> robot_indices) {
    if (!robot_indices.empty())
        return robot_indices;
    robot_indices.reserve(static_cast<std::size_t>(problem.numRobots()));
    for (int i = 0; i < problem.numRobots(); ++i)
        robot_indices.push_back(i);
    return robot_indices;
}

std::vector<RobotBlock> makeBlocks(const MultiRobotProblem &problem,
                                   const std::vector<int> &robot_indices) {
    std::vector<RobotBlock> blocks;
    blocks.reserve(robot_indices.size());
    int offset = 0;
    for (const int idx : robot_indices) {
        const int ndof = problem.robot(idx).model->numJoints();
        blocks.push_back({idx, offset, ndof});
        offset += ndof;
    }
    return blocks;
}

std::vector<double> configFromState(const ob::State *state, int offset,
                                    int ndof) {
    const auto *rv = state->as<ob::RealVectorStateSpace::StateType>();
    std::vector<double> config(static_cast<std::size_t>(ndof));
    for (int d = 0; d < ndof; ++d)
        config[static_cast<std::size_t>(d)] = rv->values[offset + d];
    return config;
}

void setCompositeState(const MultiRobotProblem &problem,
                       const std::vector<RobotBlock> &blocks, bool use_goal,
                       ob::State *state) {
    auto *rv = state->as<ob::RealVectorStateSpace::StateType>();
    for (const auto &block : blocks) {
        const auto &robot = problem.robot(block.robot_index);
        const auto &config = use_goal ? robot.goal : robot.start;
        for (int d = 0; d < block.ndof; ++d)
            rv->values[block.offset + d] = config[static_cast<std::size_t>(d)];
    }
}

double maxAbsConfigDiff(const std::vector<double> &a,
                        const std::vector<double> &b) {
    if (a.size() != b.size())
        return std::numeric_limits<double>::infinity();
    double diff = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i)
        diff = std::max(diff, std::abs(a[i] - b[i]));
    return diff;
}

void normalizeCompositeStateOrder(const MultiRobotProblem &problem,
                                  const std::vector<RobotBlock> &blocks,
                                  std::vector<const ob::State *> &states) {
    if (states.size() < 2)
        return;

    double forward_error = 0.0;
    double reversed_error = 0.0;
    for (const auto &block : blocks) {
        const auto first =
            configFromState(states.front(), block.offset, block.ndof);
        const auto last =
            configFromState(states.back(), block.offset, block.ndof);
        const auto &robot = problem.robot(block.robot_index);
        forward_error += maxAbsConfigDiff(first, robot.start) +
                         maxAbsConfigDiff(last, robot.goal);
        reversed_error += maxAbsConfigDiff(first, robot.goal) +
                          maxAbsConfigDiff(last, robot.start);
    }

    if (reversed_error < forward_error)
        std::reverse(states.begin(), states.end());
}

std::vector<Path> splitCompositeStates(const MultiRobotProblem &problem,
                                       const std::vector<RobotBlock> &blocks,
                                       std::vector<const ob::State *> states) {
    normalizeCompositeStateOrder(problem, blocks, states);
    std::vector<Path> paths(blocks.size());
    for (const ob::State *state : states) {
        for (std::size_t bi = 0; bi < blocks.size(); ++bi) {
            const auto &block = blocks[bi];
            paths[bi].push_back(
                configFromState(state, block.offset, block.ndof));
        }
    }

    std::vector<double> segment_times_sec;
    if (states.size() >= 2) {
        segment_times_sec.reserve(states.size() - 1);
        for (std::size_t seg = 0; seg + 1 < states.size(); ++seg) {
            double max_robot_dist = 0.0;
            for (const auto &block : blocks) {
                const auto a =
                    configFromState(states[seg], block.offset, block.ndof);
                const auto b =
                    configFromState(states[seg + 1], block.offset, block.ndof);
                double dist_sq = 0.0;
                for (std::size_t d = 0; d < a.size(); ++d) {
                    const double diff = b[d] - a[d];
                    dist_sq += diff * diff;
                }
                max_robot_dist = std::max(max_robot_dist, std::sqrt(dist_sq));
            }
            segment_times_sec.push_back(max_robot_dist / problem.vmax());
        }
    }

    for (auto &path : paths) {
        if (!segment_times_sec.empty())
            path.setTimestepsFromSegmentTimes(segment_times_sec,
                                              problem.resolution());
        path.interpolate_to_timesteps(problem.resolution(), problem.vmax());
    }
    return paths;
}

std::vector<Path> splitCompositeGeometric(const MultiRobotProblem &problem,
                                          const std::vector<RobotBlock> &blocks,
                                          const og::PathGeometric &gpath) {
    std::vector<const ob::State *> states;
    states.reserve(gpath.getStateCount());
    for (std::size_t s = 0; s < gpath.getStateCount(); ++s)
        states.push_back(gpath.getState(s));
    return splitCompositeStates(problem, blocks, states);
}

std::pair<std::uint64_t, std::uint64_t>
metricsFromPaths(const std::vector<Path> &paths) {
    std::uint64_t sum = 0;
    std::uint64_t makespan = 0;
    for (const auto &path : paths) {
        const auto arrival =
            static_cast<std::uint64_t>(path.arrival_timestep());
        sum += arrival;
        makespan = std::max(makespan, arrival);
    }
    return {sum, makespan};
}

nlohmann::json makeEventJson(const Clock::time_point &solve_start,
                             const std::vector<Path> &paths, double ompl_cost,
                             const char *kind) {
    const auto [sum, makespan] = metricsFromPaths(paths);
    return {
        {"elapsed_seconds",
         std::chrono::duration<double>(Clock::now() - solve_start).count()},
        {"ompl_cost", ompl_cost},
        {"sum_of_cost_timesteps", sum},
        {"makespan_timesteps", makespan},
        {"kind", kind},
    };
}

void requireExactEndpoints(const MultiRobotProblem &problem,
                           const std::vector<int> &robot_indices,
                           const std::vector<Path> &paths,
                           const char *planner_name) {
    constexpr double kEndpointTolerance = 1e-6;
    if (paths.size() != robot_indices.size())
        throw std::runtime_error(std::string(planner_name) +
                                 " returned wrong path count");
    for (std::size_t local_idx = 0; local_idx < paths.size(); ++local_idx) {
        const int robot_idx = robot_indices[local_idx];
        const auto &path = paths[local_idx];
        if (path.empty()) {
            std::ostringstream msg;
            msg << planner_name << " exact solution path missing for robot "
                << robot_idx;
            throw std::runtime_error(msg.str());
        }

        std::ostringstream start_context;
        start_context << planner_name
                      << " exact solution start mismatch for robot "
                      << robot_idx;
        comotion::detail::requireConfigNear(path.front(),
                                        problem.robot(robot_idx).start,
                                        kEndpointTolerance,
                                        start_context.str());

        std::ostringstream goal_context;
        goal_context << planner_name
                     << " exact solution goal mismatch for robot " << robot_idx;
        comotion::detail::requireConfigNear(path.back(),
                                        problem.robot(robot_idx).goal,
                                        kEndpointTolerance,
                                        goal_context.str());
    }
}

} // namespace

const char *toString(CompositePRMStar::MetricMode mode) {
    switch (mode) {
    case CompositePRMStar::MetricMode::Makespan:
        return "makespan";
    case CompositePRMStar::MetricMode::PlainL2:
        return "plain_l2";
    }
    return "makespan";
}

ompl::base::PlannerStatus CompositePRMStar::solve(double timeLimit) {
    resetPlannerRunMetrics();
    solution_paths_.clear();

    const auto indices = defaultRobotIndices(*problem_, robot_indices_);
    const auto blocks = makeBlocks(*problem_, indices);
    auto si = metric_mode_ == MetricMode::Makespan
                  ? problem_->createMakespanCompositeSpaceInfo(indices)
                  : problem_->createCompositeSpaceInfo(indices);
    auto space = si->getStateSpace();

    og::SimpleSetup setup(si);
    auto objective = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    setup.getProblemDefinition()->setOptimizationObjective(objective);

    auto planner = std::make_shared<og::PRMstar>(si);
    setup.setPlanner(planner);

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    setCompositeState(*problem_, blocks, false, start.get());
    setCompositeState(*problem_, blocks, true, goal.get());
    setup.setStartAndGoalStates(start, goal);

    const auto solve_start = Clock::now();
    nlohmann::json events = nlohmann::json::array();
    const auto solve_wall_start = Clock::now();
    const auto raw_status = setup.solve(timeLimit);
    const auto solve_wall_ns = elapsedNanoseconds(solve_wall_start);
    std::uint64_t simplify_wall_ns = 0;

    auto exported_status = raw_status;
    const bool has_exact_solution =
        setup.getProblemDefinition()->hasExactSolution();
    if (!has_exact_solution &&
        raw_status == ob::PlannerStatus::APPROXIMATE_SOLUTION) {
        exported_status = ob::PlannerStatus::TIMEOUT;
    }

    if (has_exact_solution) {
        if (simplify_solution_) {
            const auto simplify_start = Clock::now();
            detail::simplifySolutionBounded(setup, simplification_options_);
            simplify_wall_ns = elapsedNanoseconds(simplify_start);
        }

        auto &gpath = setup.getSolutionPath();
        gpath.interpolate();
        solution_paths_ = splitCompositeGeometric(*problem_, blocks, gpath);
        requireExactEndpoints(*problem_, indices, solution_paths_,
                              "CompositePRMStar");
        setSolutionMetricsFromPaths(solution_paths_);

        const double final_cost = gpath.cost(objective).value();
        events.push_back(makeEventJson(solve_start, solution_paths_, final_cost,
                                       "final_solution"));
        exported_status = ob::PlannerStatus::EXACT_SOLUTION;
    }

    ob::PlannerData planner_data(si);
    planner->getPlannerData(planner_data);

    nlohmann::json stats = nlohmann::json::object();
    stats["ompl_planner"] = "PRMstar";
    stats["metric_mode"] = toString(metric_mode_);
    stats["optimization_objective"] = "PathLengthOptimizationObjective";
    stats["raw_status"] = raw_status.asString();
    stats["exact_solution_available"] = has_exact_solution;
    stats["solve_wall_seconds"] = static_cast<double>(solve_wall_ns) * 1e-9;
    stats["simplify_wall_seconds"] =
        static_cast<double>(simplify_wall_ns) * 1e-9;
    stats["num_vertices"] = planner_data.numVertices();
    stats["num_edges"] = planner_data.numEdges();
    stats["path_simplification"] = {
        {"enabled", simplify_solution_},
        {"max_shortcut_steps", simplification_options_.max_shortcut_steps},
        {"max_empty_steps", simplification_options_.max_empty_steps},
        {"max_smooth_steps", simplification_options_.max_smooth_steps},
        {"max_passes", simplification_options_.max_passes},
    };
    stats["solution_events"] = std::move(events);
    stats["num_solution_events"] = stats["solution_events"].size();
    setPlannerStatsJson(std::move(stats));

    return exported_status;
}

std::vector<Path> CompositePRMStar::getSolutionPaths() const {
    return solution_paths_;
}

} // namespace comotion
