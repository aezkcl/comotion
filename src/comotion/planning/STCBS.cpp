#include "comotion/planning/STCBS.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace comotion {

namespace {

std::string summarizeConfig(const std::vector<double> &config,
                            std::size_t max_dims = 4) {
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < config.size() && i < max_dims; ++i) {
        if (i > 0)
            out << ", ";
        out << config[i];
    }
    if (config.size() > max_dims)
        out << ", ... (" << config.size() << " dims)";
    out << "]";
    return out.str();
}

int motionIndexAtTimestep(const USTRRTstar::Result &result, int timestep) {
    if (result.dense_timestep_to_motion_index.empty())
        return -1;
    if (timestep < 0)
        return result.dense_timestep_to_motion_index.front();
    const std::size_t idx = std::min<std::size_t>(
        static_cast<std::size_t>(timestep),
        result.dense_timestep_to_motion_index.size() - 1);
    return result.dense_timestep_to_motion_index[idx];
}

std::size_t arrivalTimestep(const USTRRTstar::Result &result,
                            std::size_t resolution) {
    if (!std::isfinite(result.arrival_time_seconds))
        return std::numeric_limits<std::size_t>::max();
    return static_cast<std::size_t>(std::max(
        0, static_cast<int>(std::llround(
               result.arrival_time_seconds *
               static_cast<double>(std::max<std::size_t>(1, resolution))))));
}

double constraintTimeEpsilon(std::size_t resolution) {
    return 0.5 /
           static_cast<double>(std::max<std::size_t>(1, resolution));
}

std::vector<double> configAtTimeOnEdge(const std::vector<double> &from,
                                       const std::vector<double> &to,
                                       double t0, double t1,
                                       double query_time) {
    if (std::abs(t1 - t0) <= 1e-12)
        return to;
    const double alpha =
        std::clamp((query_time - t0) / (t1 - t0), 0.0, 1.0);
    return interpolateConfig(from, to, alpha);
}

std::vector<double> configAtTimeOnResult(const USTRRTstar::Result &result,
                                         double query_time, double time_eps) {
    if (result.raw_path.empty())
        return {};
    if (result.raw_path.size() == 1 || result.raw_times_seconds.size() <= 1)
        return result.raw_path.back();

    if (query_time <= result.raw_times_seconds.front() + time_eps)
        return result.raw_path.front();

    for (std::size_t i = 0; i + 1 < result.raw_times_seconds.size(); ++i) {
        const double t0 = result.raw_times_seconds[i];
        const double t1 = result.raw_times_seconds[i + 1];
        if (query_time <= t1 + time_eps) {
            return configAtTimeOnEdge(result.raw_path[i], result.raw_path[i + 1],
                                      t0, t1, query_time);
        }
    }

    // Hold the goal config after arrival time, matching composite path checks.
    return result.raw_path.back();
}

bool hasDuplicateConstraint(const USTRRTstar::TreeSnapshot &tree,
                            const USTRRTstar::BranchConstraint &constraint) {
    for (const auto &existing : tree.constraints) {
        if (existing.constrained_agent_id == constraint.constrained_agent_id &&
            existing.other_agent_id == constraint.other_agent_id &&
            existing.timestep == constraint.timestep) {
            return true;
        }
    }
    return false;
}

void throwIfDuplicateConstraint(const USTRRTstar::TreeSnapshot &tree,
                                const USTRRTstar::BranchConstraint &constraint,
                                int ct_node_index) {
    if (!hasDuplicateConstraint(tree, constraint))
        return;

    std::ostringstream msg;
    msg << "STCBS: duplicate constraint in CT node " << ct_node_index
        << " for constrained_robot=" << constraint.constrained_agent_id
        << ", other_robot=" << constraint.other_agent_id
        << ", timestep=" << constraint.timestep
        << ", existing_constraints=" << tree.constraints.size();
    throw std::runtime_error(msg.str());
}

void throwIfResultViolatesConstraints(
    const USTRRTstar::TreeSnapshot &tree,
    const USTRRTstar::Result &result, int ct_node_index, int agent_idx) {
    if (!result.exact)
        return;
    if (result.raw_path.size() != result.raw_times_seconds.size()) {
        std::ostringstream msg;
        msg << "STCBS: invalid exact result shape for robot " << agent_idx
            << " in CT node " << ct_node_index << ": raw_path="
            << result.raw_path.size() << ", raw_times="
            << result.raw_times_seconds.size();
        throw std::runtime_error(msg.str());
    }

    const double time_eps = constraintTimeEpsilon(tree.resolution);
    for (const auto &constraint : tree.constraints) {
        if (constraint.other_model == nullptr)
            continue;
        const auto config = configAtTimeOnResult(
            result, constraint.time_seconds, time_eps);
        if (config.empty())
            continue;
        if (!tree.collision_checker->isValidPair(
                *tree.model, config, *constraint.other_model,
                constraint.other_config)) {
            const double arrival_time =
                result.raw_times_seconds.empty()
                    ? std::numeric_limits<double>::quiet_NaN()
                    : result.raw_times_seconds.back();
            const bool after_arrival =
                !result.raw_times_seconds.empty() &&
                constraint.time_seconds > arrival_time + time_eps;
            std::ostringstream msg;
            msg << "STCBS: constrained path for robot " << agent_idx
                << " violates active constraint in CT node " << ct_node_index
                << " against robot " << constraint.other_agent_id
                << " at timestep " << constraint.timestep
                << " (time=" << constraint.time_seconds
                << ", arrival=" << arrival_time
                << ", after_arrival=" << (after_arrival ? "true" : "false")
                << ", sampled_config=" << summarizeConfig(config)
                << ", other_config="
                << summarizeConfig(constraint.other_config)
                << ")";
            throw std::runtime_error(msg.str());
        }
    }
}

void throwIfResultViolatesStructuralState(
    const USTRRTstar::TreeSnapshot &tree,
    const USTRRTstar::Result &result, int ct_node_index, int agent_idx) {
    if (!result.exact)
        return;

    if (tree.goal_permanently_blocked) {
        std::ostringstream msg;
        msg << "STCBS: exact result returned for permanently blocked robot "
            << agent_idx << " in CT node " << ct_node_index;
        throw std::runtime_error(msg.str());
    }

    const auto arrival_ts = arrivalTimestep(result, tree.resolution);
    if (arrival_ts < tree.min_safe_arrival_timestep) {
        std::ostringstream msg;
        msg << "STCBS: exact result for robot " << agent_idx
            << " violates min_safe_arrival_timestep in CT node "
            << ct_node_index << " (arrival_timestep=" << arrival_ts
            << ", min_safe_arrival_timestep="
            << tree.min_safe_arrival_timestep << ")";
        throw std::runtime_error(msg.str());
    }

    for (int motion_index : result.motion_indices) {
        if (motion_index < 0 ||
            static_cast<std::size_t>(motion_index) >= tree.motions.size()) {
            continue;
        }
        const auto *motion =
            tree.motions[static_cast<std::size_t>(motion_index)].get();
        if (motion != nullptr && motion->marked) {
            std::ostringstream msg;
            msg << "STCBS: exact result for robot " << agent_idx
                << " includes marked motion " << motion_index
                << " in CT node " << ct_node_index;
            throw std::runtime_error(msg.str());
        }
    }
}

} // namespace

USTRRTstar::Params STCBS::makeParams() const {
    USTRRTstar::Params params;
    params.range = range_;
    params.max_iterations = max_iterations_;
    params.max_samples = max_samples_;
    params.goal_threshold = goal_threshold_;
    params.rewire_mode = rewire_mode_;
    params.rewire_radius = rewire_radius_;
    params.rewire_k = rewire_k_;
    params.layer_dt_seconds = layer_dt_seconds_;
    return params;
}

std::optional<STCBSConflict>
STCBS::getFirstConflict(const STCBSNode &node) const {
    std::vector<Path> dense_paths;
    dense_paths.reserve(node.results.size());
    for (const auto &result : node.results)
        dense_paths.push_back(result.dense_path);

    CompositePathValidationOptions options;
    options.check_environment = false;
    auto ptrs = problem_->robotModelPtrs();
    auto conflict = problem_->collisionChecker().findFirstCompositePathConflict(
        dense_paths, ptrs, options);
    if (!conflict)
        return std::nullopt;

    return STCBSConflict{
        conflict->robot_i,
        conflict->robot_j,
        static_cast<int>(conflict->timestep),
        motionIndexAtTimestep(node.results[static_cast<std::size_t>(conflict->robot_i)],
                              static_cast<int>(conflict->timestep)),
        motionIndexAtTimestep(node.results[static_cast<std::size_t>(conflict->robot_j)],
                              static_cast<int>(conflict->timestep)),
        conflict->config_i,
        conflict->config_j,
    };
}

double STCBS::computeTotalSTCost(const STCBSNode &node) const {
    double total = 0.0;
    for (const auto &result : node.results) {
        if (!result.exact)
            return std::numeric_limits<double>::infinity();
        total += result.st_cost;
    }
    return total;
}

bool STCBS::applyBranchGoalHold(USTRRTstar::TreeSnapshot &tree,
                                   int constrained_robot, int other_robot,
                                   const Path &other_path) const {
    if (!goal_hold_enabled_)
        return true;

    const auto constraint = problem_->collisionChecker().computeGoalHoldConstraint(
        *problem_->robot(constrained_robot).model,
        problem_->robot(constrained_robot).goal,
        *problem_->robot(other_robot).model, other_path);

    if (constraint.permanently_blocked) {
        tree.goal_permanently_blocked = true;
        return false;
    }

    if (!USTRRTstar::updateMinSafeArrivalTimestep(
            tree, constraint.min_safe_arrival_timestep)) {
        return false;
    }

    return true;
}

ompl::base::PlannerStatus STCBS::solve(double timeLimit) {
    resetPlannerRunMetrics();
    solution_paths_.clear();
    conflict_count_ = 0;
    ct_nodes_expanded_ = 0;
    ust_rrt_calls_total_ = 0;
    ust_rrt_calls_successes_ = 0;
    ust_rrt_solve_times_seconds_.clear();

    const int n = problem_->numRobots();
    const auto params = makeParams();
    const auto start_time = std::chrono::steady_clock::now();
    const auto deadline =
        start_time + std::chrono::duration_cast<
                         std::chrono::steady_clock::duration>(
                         std::chrono::duration<double>(timeLimit));
    const auto finalizePlannerStats = [&]() {
        nlohmann::json stats = nlohmann::json::object();
        stats["num_conflicts"] = conflict_count_;
        stats["ct_nodes_expanded"] = ct_nodes_expanded_;
        stats["ust_rrt_calls_total"] = ust_rrt_calls_total_;
        stats["ust_rrt_calls_successes"] = ust_rrt_calls_successes_;
        stats["ust_rrt_success_rate"] =
            ust_rrt_calls_total_ > 0
                ? static_cast<double>(ust_rrt_calls_successes_) /
                      static_cast<double>(ust_rrt_calls_total_)
                : 0.0;
        stats["ust_rrt_solve_times_seconds"] = ust_rrt_solve_times_seconds_;
        setPlannerStatsJson(std::move(stats));
    };

    auto root = std::make_shared<STCBSNode>();
    root->trees.reserve(n);
    root->results.reserve(n);

    for (int i = 0; i < n; ++i) {
        const auto &robot = problem_->robot(i);
        const auto low_level_start = std::chrono::steady_clock::now();
        auto [tree, result] = USTRRTstar::buildTree(
            i, robot.start, robot.goal, *robot.model,
            problem_->collisionChecker(), params, problem_->resolution(),
            problem_->vmax(), lambda_, planning_seed_, deadline);
        ++ust_rrt_calls_total_;
        const double low_level_elapsed_seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          low_level_start)
                .count();
        ust_rrt_solve_times_seconds_.push_back(low_level_elapsed_seconds);
        if (result.exact)
            ++ust_rrt_calls_successes_;
        if (!result.exact) {
            finalizePlannerStats();
            return ompl::base::PlannerStatus::TIMEOUT;
        }
        root->trees.push_back(std::move(tree));
        root->results.push_back(std::move(result));
    }

    root->st_cost = computeTotalSTCost(*root);

    auto cmp = [](const std::shared_ptr<STCBSNode> &lhs,
                  const std::shared_ptr<STCBSNode> &rhs) {
        return lhs->st_cost > rhs->st_cost;
    };
    std::priority_queue<std::shared_ptr<STCBSNode>,
                        std::vector<std::shared_ptr<STCBSNode>>,
                        decltype(cmp)>
        open(cmp);
    open.push(root);

    int expanded = 0;
    while (!open.empty() && expanded < max_ct_nodes_) {
        if (std::chrono::steady_clock::now() >= deadline)
            break;

        auto node = open.top();
        open.pop();
        ++expanded;
        ct_nodes_expanded_ = static_cast<std::uint64_t>(expanded);

        auto conflict = getFirstConflict(*node);
        if (!conflict) {
            solution_paths_.reserve(node->results.size());
            for (const auto &result : node->results)
                solution_paths_.push_back(result.dense_path);
            setSolutionMetricsFromPaths(solution_paths_);
            finalizePlannerStats();
            return ompl::base::PlannerStatus::EXACT_SOLUTION;
        }
        ++conflict_count_;

        const std::array<std::tuple<int, int, int>, 2> branches = {{
            {conflict->robot_i, conflict->robot_j, conflict->motion_index_i},
            {conflict->robot_j, conflict->robot_i, conflict->motion_index_j},
        }};

        for (const auto &[agent_idx, other_idx, conflict_motion_index] :
             branches) {
            auto child = std::make_shared<STCBSNode>(*node);
            child->trees[static_cast<std::size_t>(agent_idx)] =
                node->trees[static_cast<std::size_t>(agent_idx)]->clone();

            auto &tree = *child->trees[static_cast<std::size_t>(agent_idx)];

            if (conflict_motion_index < 0 ||
                static_cast<std::size_t>(conflict_motion_index) >=
                    tree.motions.size()) {
                continue;
            }

            USTRRTstar::pruneNeighbors(tree, conflict_motion_index,
                                          occupied_radius_);
            USTRRTstar::pruneDescendants(tree, conflict_motion_index);
            USTRRTstar::markMotion(tree, conflict_motion_index);
            tree.rebuildNearestNeighbors();

            if (!applyBranchGoalHold(
                    tree, agent_idx, other_idx,
                    child->results[static_cast<std::size_t>(other_idx)]
                        .dense_path)) {
                continue;
            }

            if (!tree.isSearchFeasible()) {
                continue;
            }

            auto result = USTRRTstar::extractBestPath(tree, lambda_);
            if (!result.exact)
            {
                const auto low_level_start = std::chrono::steady_clock::now();
                result = USTRRTstar::extendTree(tree, lambda_,
                                               max_iterations_, deadline);
                ++ust_rrt_calls_total_;
                const double low_level_elapsed_seconds =
                    std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - low_level_start)
                        .count();
                ust_rrt_solve_times_seconds_.push_back(
                    low_level_elapsed_seconds);
                if (result.exact)
                    ++ust_rrt_calls_successes_;
            }
            if (!result.exact) {
                continue;
            }
            throwIfResultViolatesStructuralState(tree, result, expanded,
                                                 agent_idx);
            throwIfResultViolatesConstraints(tree, result, expanded, agent_idx);

            child->results[static_cast<std::size_t>(agent_idx)] =
                std::move(result);
            child->st_cost = computeTotalSTCost(*child);
            open.push(std::move(child));
        }
    }

    ct_nodes_expanded_ = static_cast<std::uint64_t>(expanded);
    finalizePlannerStats();
    return ompl::base::PlannerStatus::TIMEOUT;
}

std::vector<Path> STCBS::getSolutionPaths() const {
    return solution_paths_;
}

} // namespace comotion
