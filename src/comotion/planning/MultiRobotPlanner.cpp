#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/detail/PlannerInvariantUtils.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace comotion {

void MultiRobotPlanner::resetPlannerRunMetrics() {
    clearSolutionMetrics();
    planner_stats_json_ = nlohmann::json::object();
}

void MultiRobotPlanner::clearSolutionMetrics() {
    solution_metrics_.sum_of_cost_timesteps.reset();
    solution_metrics_.makespan_timesteps.reset();
}

void MultiRobotPlanner::setSolutionMetrics(
    const std::uint64_t sum_of_cost_timesteps,
    const std::uint64_t makespan_timesteps) {
    solution_metrics_.sum_of_cost_timesteps = sum_of_cost_timesteps;
    solution_metrics_.makespan_timesteps = makespan_timesteps;
}

void MultiRobotPlanner::setSolutionMetricsFromPaths(
    const std::vector<Path> &paths) {
    const auto [sum_of_cost_timesteps, makespan_timesteps] =
        computeSolutionMetrics(paths);
    setSolutionMetrics(sum_of_cost_timesteps, makespan_timesteps);
}

void MultiRobotPlanner::setPlannerStatsJson(nlohmann::json stats) {
    if (!stats.is_object())
        stats = nlohmann::json::object();
    planner_stats_json_ = std::move(stats);
}

std::pair<std::uint64_t, std::uint64_t>
MultiRobotPlanner::computeSolutionMetrics(const std::vector<Path> &paths) {
    std::uint64_t sum_of_cost_timesteps = 0;
    std::uint64_t makespan_timesteps = 0;
    for (const auto &path : paths) {
        const auto arrival_timestep =
            static_cast<std::uint64_t>(path.arrival_timestep());
        sum_of_cost_timesteps += arrival_timestep;
        makespan_timesteps = std::max(makespan_timesteps, arrival_timestep);
    }
    return {sum_of_cost_timesteps, makespan_timesteps};
}

Path MultiRobotPlanner::omplPathToPath(
    const ompl::geometric::PathGeometric &gpath, int ndof,
    size_t resolution, double vmax) {
    Path result;
    for (size_t i = 0; i < gpath.getStateCount(); ++i) {
        result.push_back(stateToConfig(gpath.getState(i), ndof));
    }
    if (result.size() >= 2 && vmax > 0.0 && resolution >= 1)
        result.computeTimestepsFromDistance(resolution, vmax);
    return result;
}

std::vector<Path> MultiRobotPlanner::splitCompositePathToRobotPaths(
    const ompl::geometric::PathGeometric &path,
    const MultiRobotProblem &problem, const std::vector<int> &robot_indices,
    const std::string &planner_label) {
    int offset = 0;
    std::vector<Path> robot_paths;
    robot_paths.reserve(robot_indices.size());
    for (int idx : robot_indices) {
        const int ndof = problem.robot(idx).model->numJoints();
        Path robot_path;
        for (size_t s = 0; s < path.getStateCount(); ++s) {
            const auto *rv =
                path.getState(s)
                    ->as<ompl::base::RealVectorStateSpace::StateType>();
            std::vector<double> cfg(static_cast<std::size_t>(ndof));
            for (int d = 0; d < ndof; ++d)
                cfg[static_cast<std::size_t>(d)] = rv->values[offset + d];
            robot_path.push_back(std::move(cfg));
        }
        robot_paths.push_back(std::move(robot_path));
        offset += ndof;
    }

    const double vmax = problem.vmax();
    const size_t resolution = problem.resolution();
    std::vector<double> segment_times_sec;
    if (!robot_paths.empty() && robot_paths[0].size() >= 2) {
        const size_t n_segments = robot_paths[0].size() - 1;
        segment_times_sec.reserve(n_segments);
        for (size_t seg = 0; seg < n_segments; ++seg) {
            double max_seg_dist = 0.0;
            for (const auto &rp : robot_paths) {
                const auto &a = rp[seg];
                const auto &b = rp[seg + 1];
                double d = 0.0;
                for (size_t i = 0; i < a.size(); ++i) {
                    const double dd = b[i] - a[i];
                    d += dd * dd;
                }
                max_seg_dist = std::max(max_seg_dist, std::sqrt(d));
            }
            segment_times_sec.push_back(max_seg_dist / vmax);
        }
    }
    if (segment_times_sec.empty() && vmax > 0.0) {
        double fallback = 0.0;
        for (const auto &rp : robot_paths)
            fallback = std::max(fallback, rp.path_cost() / vmax);
        if (fallback > 0.0)
            segment_times_sec.push_back(fallback);
    }

    for (auto &rp : robot_paths) {
        if (!segment_times_sec.empty())
            rp.setTimestepsFromSegmentTimes(segment_times_sec, resolution);
    }

    for (auto &rp : robot_paths)
        rp.interpolate_to_timesteps(resolution, vmax);

    constexpr double kEndpointTolerance = 1e-6;
    for (size_t local_idx = 0; local_idx < robot_paths.size(); ++local_idx) {
        const int robot_idx = robot_indices[local_idx];
        const auto &solution_path = robot_paths[local_idx];
        const auto &robot = problem.robot(robot_idx);
        if (solution_path.empty()) {
            std::ostringstream msg;
            msg << planner_label << " exact solution path missing for robot "
                << robot_idx << " (local index " << local_idx << ")";
            throw std::runtime_error(msg.str());
        }
        std::ostringstream start_context;
        start_context << planner_label
                      << " exact solution start mismatch for robot "
                      << robot_idx;
        comotion::detail::requireConfigNear(solution_path.front(), robot.start,
                                        kEndpointTolerance,
                                        start_context.str());

        std::ostringstream goal_context;
        goal_context << planner_label
                     << " exact solution goal mismatch for robot " << robot_idx;
        comotion::detail::requireConfigNear(solution_path.back(), robot.goal,
                                        kEndpointTolerance,
                                        goal_context.str());
    }

    return robot_paths;
}

} // namespace comotion
