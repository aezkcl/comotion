#include "comotion/planning/GuidedARC.h"
#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/PlanningSeed.h"
#include "comotion/planning/PrioritizedSTRRT.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace comotion {

namespace {

// Return the inclusive final timestep across every global path.
std::optional<int> globalEndT(const std::vector<Path> &paths) {
    // A global path set must contain at least one path.
    if (paths.empty())
        return std::nullopt;

    std::size_t global_end_t = 0;

    for (const auto &path : paths) {
        // The global horizon is undefined while any global path is empty.
        if (path.empty())
            return std::nullopt;

        global_end_t = std::max(global_end_t, path.arrival_timestep());
    }

    // MRMPSubproblem stores timestep fields as int.
    if (global_end_t >
        static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::overflow_error(
            "Subproblem timestep exceeds the supported integer range");
    }

    return static_cast<int>(global_end_t);
}

// Check the two composite configurations that bound a local attempt.
std::pair<bool, bool>
endpointValidity(const MRMPSubproblem &subproblem) {
    if (!subproblem.problem)
        throw std::logic_error("Resolution attempt has no local problem");

    const auto robot_models = subproblem.problem->robotModelPtrs();
    std::vector<std::vector<double>> starts;
    std::vector<std::vector<double>> goals;
    starts.reserve(static_cast<std::size_t>(subproblem.problem->numRobots()));
    goals.reserve(static_cast<std::size_t>(subproblem.problem->numRobots()));

    for (int local_index = 0;
         local_index < subproblem.problem->numRobots(); ++local_index) {
        starts.push_back(subproblem.problem->robot(local_index).start);
        goals.push_back(subproblem.problem->robot(local_index).goal);
    }

    const auto &checker = subproblem.problem->collisionChecker();
    return {
        robot_models.empty() || checker.isValidComposite(robot_models, starts),
        robot_models.empty() || checker.isValidComposite(robot_models, goals),
    };
}

// Compute ARC-style configuration-coordinate bounds from one path window.
RobotCspaceRegion configurationRegion(
    int global_robot_index, const RobotModel &robot, const Path &path,
    int window_start_t, int window_end_t, double margin,
    double minimum_range) {
    const int dimensions = robot.numJoints();
    RobotCspaceRegion region;
    region.global_robot_index = global_robot_index;
    region.lower.assign(static_cast<std::size_t>(dimensions),
                        std::numeric_limits<double>::max());
    region.upper.assign(static_cast<std::size_t>(dimensions),
                        std::numeric_limits<double>::lowest());

    const auto include_config = [&](const std::vector<double> &config) {
        if (config.size() < static_cast<std::size_t>(dimensions)) {
            throw std::logic_error(
                "Path configuration has fewer coordinates than its robot");
        }
        for (int dimension = 0; dimension < dimensions; ++dimension) {
            const auto coordinate =
                config[static_cast<std::size_t>(dimension)];
            region.lower[static_cast<std::size_t>(dimension)] = std::min(
                region.lower[static_cast<std::size_t>(dimension)], coordinate);
            region.upper[static_cast<std::size_t>(dimension)] = std::max(
                region.upper[static_cast<std::size_t>(dimension)], coordinate);
        }
    };

    // Always include the exact interpolated configurations at both endpoints.
    include_config(path.config_at_timestep(
        static_cast<std::size_t>(window_start_t)));
    include_config(path.config_at_timestep(
        static_cast<std::size_t>(window_end_t)));

    // Include every stored waypoint that lies inside the inclusive window.
    for (std::size_t waypoint = 0; waypoint < path.size(); ++waypoint) {
        const std::size_t waypoint_t = path.timestep_at(waypoint);
        if (waypoint_t >= static_cast<std::size_t>(window_start_t) &&
            waypoint_t <= static_cast<std::size_t>(window_end_t)) {
            include_config(path[waypoint]);
        }
    }

    // Expand each observed range, clamp it to the robot's global limits, and
    // enforce ARC's configured minimum range where the global limits permit it.
    for (int dimension = 0; dimension < dimensions; ++dimension) {
        const auto index = static_cast<std::size_t>(dimension);
        const double observed_range = region.upper[index] - region.lower[index];
        const double expansion = margin * observed_range;
        const double global_lower = robot.jointLower(dimension);
        const double global_upper = robot.jointUpper(dimension);

        region.lower[index] =
            std::max(region.lower[index] - expansion, global_lower);
        region.upper[index] =
            std::min(region.upper[index] + expansion, global_upper);

        if (region.lower[index] > region.upper[index]) {
            const double midpoint =
                0.5 * (region.lower[index] + region.upper[index]);
            region.lower[index] = midpoint;
            region.upper[index] = midpoint;
        }

        if (minimum_range > 0.0 &&
            region.upper[index] - region.lower[index] < minimum_range) {
            const double global_range = global_upper - global_lower;
            if (global_range >= minimum_range) {
                const double half_range = 0.5 * minimum_range;
                const double midpoint = std::clamp(
                    0.5 * (region.lower[index] + region.upper[index]),
                    global_lower + half_range, global_upper - half_range);
                region.lower[index] = midpoint - half_range;
                region.upper[index] = midpoint + half_range;
            } else {
                region.lower[index] = global_lower;
                region.upper[index] = global_upper;
            }
        }
    }

    return region;
}

// Return the unrestricted configuration-coordinate limits for one robot.
RobotCspaceRegion globalConfigurationRegion(int global_robot_index,
                                             const RobotModel &robot) {
    RobotCspaceRegion region;
    region.global_robot_index = global_robot_index;
    region.lower.reserve(static_cast<std::size_t>(robot.numJoints()));
    region.upper.reserve(static_cast<std::size_t>(robot.numJoints()));

    for (int dimension = 0; dimension < robot.numJoints(); ++dimension) {
        region.lower.push_back(robot.jointLower(dimension));
        region.upper.push_back(robot.jointUpper(dimension));
    }

    return region;
}

} // namespace

std::optional<MRMPSubproblem> GuidedARC::createSubProblem(
    const SubproblemConflict &conflict, const std::vector<Path> &paths,
    ResolutionAttempts &pending_history) const {
    switch (expansion_mode_) {
    case SubproblemExpansionMode::Temporal:
        return createTemporalSubProblem(
            conflict, paths, pending_history);

    case SubproblemExpansionMode::CSpace:
        throw std::logic_error(
            "GuidedARC CSpace mode is not implemented yet");
    }

    throw std::logic_error(
        "GuidedARC has an unknown subproblem expansion mode");
}

std::optional<MRMPSubproblem> GuidedARC::createTemporalSubProblem(
    const SubproblemConflict &conflict, const std::vector<Path> &paths,
    ResolutionAttempts &pending_history) const {
    if (!problem_)
        throw std::logic_error("GuidedARC has no planning problem");

    // A subproblem without participating robots cannot be constructed.
    if (conflict.robots.empty())
        return std::nullopt;

    // Validate the local-to-global mapping before indexing either collection.
    for (const int robot_index : conflict.robots) {
        if (robot_index < 0 ||
            static_cast<std::size_t>(robot_index) >= paths.size() ||
            robot_index >= problem_->numRobots()) {
            throw std::out_of_range(
                "Subproblem robot index is outside the global problem");
        }
    }

    // The inclusive horizon is the latest arrival across all global paths.
    const auto global_end_t = globalEndT(paths);
    if (!global_end_t)
        return std::nullopt;

    // The first attempt uses the conflict window produced by baseline ARC.
    int window_start_t =
        std::clamp(conflict.window_begin_t, 0, *global_end_t);
    int window_end_t =
        std::clamp(conflict.window_end_t, window_start_t, *global_end_t);

    if (!pending_history.attempts.empty()) {
        const auto &last_attempt = pending_history.attempts.back();
        if (!last_attempt.subproblem)
            throw std::logic_error(
                "Pending resolution attempt has no subproblem");

        // A failed attempt over the complete time interval cannot expand again.
        if (last_attempt.subproblem->spansGlobalTime())
            return std::nullopt;

        if (pending_history.expansion_attempts_applied >
            pending_history.attempts.size()) {
            throw std::logic_error(
                "Expansion cache is ahead of resolution history");
        }

        if (pending_history.expansion_attempts_applied <
            pending_history.attempts.size()) {
            if (pending_history.expansion_attempts_applied + 1 !=
                pending_history.attempts.size()) {
                throw std::logic_error(
                    "Expansion cache missed a resolution attempt");
            }

            const auto [start_valid, goal_valid] =
                endpointValidity(*last_attempt.subproblem);
            pending_history.next_temporal_window =
                nextExpansionWindowAfterAttempt(
                    last_attempt.subproblem->window_start_t,
                    last_attempt.subproblem->window_end_t,
                    static_cast<std::size_t>(*global_end_t), start_valid,
                    goal_valid, pending_history.expansion_schedule_state);
            pending_history.expansion_attempts_applied =
                pending_history.attempts.size();
        }

        if (!pending_history.next_temporal_window) {
            throw std::logic_error(
                "Expansion cache has no temporal window");
        }

        window_start_t = pending_history.next_temporal_window->first;
        window_end_t = pending_history.next_temporal_window->second;
    }

    // Every attempt owns a fresh local problem so expansion cannot mutate an
    // MRMPSubproblem already retained in ResolutionHistory.
    auto local_problem = std::make_shared<MultiRobotProblem>(
        problem_->collisionChecker().backend());
    local_problem->setObstacles(std::vector<ObstacleSphere>(
        problem_->collisionChecker().obstacles().begin(),
        problem_->collisionChecker().obstacles().end()));
    local_problem->setCylinderObstacles(std::vector<ObstacleCylinder>(
        problem_->collisionChecker().cylinders().begin(),
        problem_->collisionChecker().cylinders().end()));
    local_problem->setResolution(problem_->resolution());
    local_problem->setVmax(problem_->vmax());

    const bool spans_global_time =
        window_start_t == 0 && window_end_t >= *global_end_t;
    const bool uses_global_cspace = spans_global_time || !use_cspace_bounds_;
    std::vector<RobotCspaceRegion> regions;
    regions.reserve(conflict.robots.size());

    for (std::size_t local_index = 0;
         local_index < conflict.robots.size(); ++local_index) {
        const int global_robot_index = conflict.robots[local_index];
        const auto global_index =
            static_cast<std::size_t>(global_robot_index);
        const auto &global_robot = problem_->robot(global_robot_index);
        const auto &path = paths[global_index];

        // Path::config_at_timestep holds a shorter path at its final
        // configuration, which gives unequal path lengths defined semantics.
        local_problem->addRobot(
            global_robot.model,
            path.config_at_timestep(static_cast<std::size_t>(window_start_t)),
            path.config_at_timestep(static_cast<std::size_t>(window_end_t)));

        RobotCspaceRegion region = uses_global_cspace
            ? globalConfigurationRegion(global_robot_index,
                                        *global_robot.model)
            : configurationRegion(
                  global_robot_index, *global_robot.model, path,
                  window_start_t, window_end_t,
                  static_cast<double>(cspace_bound_margin_),
                  min_cspace_bound_range_);

        if (!uses_global_cspace) {
            local_problem->setCspaceBoundsForRobot(
                static_cast<int>(local_index), region.lower, region.upper);
        }
        regions.push_back(std::move(region));
    }

    MRMPSubproblem subproblem;
    subproblem.problem = std::move(local_problem);
    subproblem.global_robot_indices = conflict.robots;
    subproblem.window_start_t = window_start_t;
    subproblem.window_end_t = window_end_t;
    subproblem.global_end_t = *global_end_t;
    subproblem.cspace_regions = std::move(regions);
    subproblem.uses_global_cspace = uses_global_cspace;
    return subproblem;
}

GuidedARC::SubproblemSolveResult GuidedARC::solveSubProblem(
    MRMPSubproblem subproblem, double time_limit,
    std::uint32_t attempt_root_seed,
    ResolutionAttempts &resolution_attempts) const {
    const auto solve_start = std::chrono::steady_clock::now();
    SubproblemSolveResult result;

    const auto remainingWallTime = [&]() {
        const double elapsed =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - solve_start)
                .count();
        return std::max(0.0, time_limit - elapsed);
    };

    const auto isCancelled = [&]() {
        return cancel_requested_ && cancel_requested_();
    };

    if (global_makespan_bound_timesteps_) {
        throw std::logic_error(
            "GuidedARC does not yet support global makespan bounds");
    }

    if (!subproblem.problem) {
        throw std::logic_error(
            "GuidedARC subproblem has no local problem");
    }

    const auto finishAttempt =
        [&](SubproblemSolveResult finished_result) {
            ResolutionAttempt attempt;
            attempt.conflict = resolution_attempts.conflict;
            attempt.subproblem =
                std::make_shared<const MRMPSubproblem>(
                    std::move(subproblem));
            attempt.attempt_index =
                resolution_attempts.attempts.size();
            attempt.attempt_root_seed = attempt_root_seed;
            attempt.solver = finished_result.solver;
            attempt.outcome = finished_result.outcome;

            resolution_attempts.append(std::move(attempt));
            return finished_result;
        };

    const auto validateReturnedPaths =
        [&](std::vector<Path> paths, const char *solver_name) {
            if (paths.size() !=
                subproblem.global_robot_indices.size()) {
                throw std::logic_error(
                    std::string(solver_name) +
                    " returned the wrong number of robot paths");
            }

            if (std::any_of(
                    paths.begin(), paths.end(),
                    [](const Path &path) { return path.empty(); })) {
                throw std::logic_error(
                    std::string(solver_name) +
                    " returned an empty robot path");
            }

            return paths;
        };

    if (isCancelled())
        return finishAttempt(std::move(result));

    const auto [start_valid, goal_valid] =
        endpointValidity(subproblem);

    if (!start_valid || !goal_valid || isCancelled())
        return finishAttempt(std::move(result));

    const bool use_prioritized =
        local_solver_mode_ != LocalSolverMode::CompositeRrtOnly;
    const bool use_composite =
        local_solver_mode_ != LocalSolverMode::PrioritizedStrrtOnly;

    if (use_prioritized) {
        const std::size_t resolution =
            subproblem.problem->resolution();
        const int window_span_t =
            std::max(0, subproblem.window_end_t -
                            subproblem.window_start_t);
        const double window_span_seconds =
            static_cast<double>(window_span_t) /
            static_cast<double>(resolution);
        const double space_time_upper_bound =
            std::max(
                window_span_seconds *
                    strrt_space_time_span_factor_,
                1.0 / static_cast<double>(resolution));

        PrioritizedSTRRT planner;
        planner.setProblem(subproblem.problem);
        planner.setPlanningSeed(
            arcRepairPrioritizedPlanningSeed(
                attempt_root_seed));
        planner.setPersistAtGoal(
            local_prioritized_strrt_persist_at_goal_);
        planner.setEqualizePaths(false);
        planner.setUseUnboundedTime(false);
        planner.setSpaceTimeUpperBound(
            space_time_upper_bound);
        planner.setSimplifyAfterPlan(
            simplify_conflict_solutions_);
        planner.setPathSimplificationOptions(
            conflict_simplification_options_.value_or(
                simplification_options_));
        planner.setStrrtRewiring(
            local_prioritized_strrt_rewiring_);
        planner.setReturnFirstSolution(
            local_prioritized_strrt_return_first_solution_);
        planner.setStrrtMaxIterations(
            local_prioritized_strrt_max_iterations_);
        planner.setCancellationCallback(cancel_requested_);

        if (!isCancelled()) {
            const double remaining_time =
                remainingWallTime();

            if (remaining_time > 0.0) {
                result.solver =
                    ResolutionSolver::PrioritizedSTRRT;

                const auto status =
                    planner.solve(remaining_time);

                if (status ==
                    ompl::base::PlannerStatus::EXACT_SOLUTION) {
                    result.paths = validateReturnedPaths(
                        planner.getSolutionPaths(),
                        "PrioritizedSTRRT");
                    result.outcome =
                        ResolutionOutcome::Success;
                    return finishAttempt(std::move(result));
                }
            }
        }
    }

    if (isCancelled())
        return finishAttempt(std::move(result));

    if (use_composite) {
        CompositeRRT planner;
        planner.setProblem(subproblem.problem);
        planner.setPlanningSeed(
            arcRepairCompositePlanningSeed(
                attempt_root_seed));
        planner.setSimplifySolution(
            simplify_conflict_solutions_);
        planner.setPathSimplificationOptions(
            conflict_simplification_options_.value_or(
                simplification_options_));

        if (local_composite_rrt_range_) {
            planner.setRange(
                *local_composite_rrt_range_);
        }

        planner.setUseMakespanMetric(
            local_composite_rrt_use_makespan_metric_);
        planner.setCancellationCallback(cancel_requested_);
        planner.setMaxRrtConnectIterations(
            subproblem.spansGlobalTime()
                ? 0
                : local_composite_rrt_max_samples_);

        if (!isCancelled()) {
            const double remaining_time =
                remainingWallTime();

            if (remaining_time > 0.0) {
                result.solver =
                    ResolutionSolver::CompositeRRT;

                const auto status =
                    planner.solve(remaining_time);

                if (status ==
                    ompl::base::PlannerStatus::EXACT_SOLUTION) {
                    result.paths = validateReturnedPaths(
                        planner.getSolutionPaths(),
                        "CompositeRRT");
                    result.outcome =
                        ResolutionOutcome::Success;
                    return finishAttempt(std::move(result));
                }
            }
        }
    }

    return finishAttempt(std::move(result));
}

ompl::base::PlannerStatus GuidedARC::solve(double time_limit) {
    if (expansion_mode_ == SubproblemExpansionMode::CSpace) {
        throw std::logic_error(
            "GuidedARC CSpace mode is not implemented yet");
    }

    if (global_makespan_bound_timesteps_) {
        throw std::logic_error(
            "GuidedARC does not yet support global makespan bounds");
    }

    if (!problem_) {
        throw std::logic_error(
            "GuidedARC requires a planning problem before solve()");
    }

    const double overall_time_limit =
        std::max(0.0, time_limit);
    const auto solve_start = Clock::now();

    const auto cancellationRequested = [&]() {
        return cancel_requested_ && cancel_requested_();
    };

    const auto deadlineReached = [&]() {
        const double elapsed_seconds =
            std::chrono::duration<double>(
                Clock::now() - solve_start)
                .count();
        return elapsed_seconds >= overall_time_limit;
    };

    resetArcSolveState();
    resolved_conflicts_.clear();

    const std::uint32_t solve_root_seed = planning_seed_;

    if (cancellationRequested() || deadlineReached()) {
        return ompl::base::PlannerStatus::TIMEOUT;
    }

    if (!planIndividualPaths(
            solve_start,
            overall_time_limit,
            solution_paths_)) {
        return ompl::base::PlannerStatus::TIMEOUT;
    }

    if (cancellationRequested() || deadlineReached()) {
        return ompl::base::PlannerStatus::TIMEOUT;
    }

    initializeConflictScanStarts(solution_paths_.size());

    const auto robot_models = problem_->robotModelPtrs();
    ConflictChecker conflict_checker(
        problem_->collisionChecker());

    std::optional<EncounteredConflictNeighbors> encountered_neighbors;
    if (robotSelectionPolicy() ==
        RobotSelectionPolicy::HistoricalDirectNeighbors) {
        encountered_neighbors.emplace(solution_paths_.size());
    }

    std::optional<SubproblemConflict> current_conflict;
    std::optional<ResolutionAttempts> current_attempts;
    std::uint64_t current_repair_id = 0;

    while (true) {
        if (cancellationRequested() || deadlineReached()) {
            return ompl::base::PlannerStatus::TIMEOUT;
        }

        if (!current_conflict) {
            startVisualizationIteration(solution_paths_);

            std::vector<SubproblemConflict> conflicts;
            if (robotSelectionPolicy() ==
                RobotSelectionPolicy::CurrentConflictComponent) {
                CompositePathValidationOptions scan_options;
                scan_options.check_environment = false;
                scan_options.stop_requested = [&]() {
                    return cancellationRequested() ||
                           deadlineReached();
                };

                CurrentConflictSnapshot snapshot;
                snapshot.neighbors.resize(solution_paths_.size());
                std::optional<Conflict> selected;
                const bool scan_complete =
                    conflict_checker.visitInterRobotConflicts(
                        solution_paths_, robot_models, scan_options,
                        [&](const CompositeConflict &found) {
                            const int timestep =
                                static_cast<int>(found.timestep);
                            if (!selected) {
                                selected.emplace(
                                    Conflict{
                                        found.robot_i,
                                        found.robot_j,
                                        timestep,
                                        found.alpha,
                                        found.kind,
                                        found.config_i,
                                        found.config_j,
                                    });
                            }
                    recordCurrentConflict(
                        Conflict{
                            found.robot_i,
                            found.robot_j,
                            timestep,
                            found.alpha,
                            found.kind,
                        },
                        snapshot);
                        });

                if (!scan_complete) {
                    return ompl::base::PlannerStatus::TIMEOUT;
                }
                snapshot.complete = true;

                if (selected) {
                    auto expanded =
                        makeInitialSubproblemConflict(*selected);
                    applyCurrentConflictComponent(
                        *selected, snapshot, initial_window_, expanded);
                    conflicts.push_back(std::move(expanded));
                }
            } else {
                CompositePathValidationOptions scan_options =
                    conflictScanOptions();

                scan_options.stop_requested = [&]() {
                    return cancellationRequested() ||
                           deadlineReached();
                };

                std::vector<std::size_t> next_t_begin_by_pair;

                conflicts = conflict_checker.findConflicts(
                    solution_paths_,
                    robot_models,
                    scan_options,
                    0,
                    1,
                    true,
                    [this, &encountered_neighbors](
                        const Conflict &conflict) {
                        if (encountered_neighbors) {
                            auto expanded =
                                makeInitialSubproblemConflict(conflict);
                            applyHistoricalDirectNeighbors(
                                conflict,
                                *encountered_neighbors,
                                initial_window_,
                                expanded);
                            return expanded;
                        }
                        return expandConflictForSubproblem(conflict);
                    },
                    nullptr,
                    &next_t_begin_by_pair);

                applyConflictScanProgress(
                    next_t_begin_by_pair);
            }

            setVisualizationConflicts(conflicts);

            if (cancellationRequested() ||
                deadlineReached()) {
                return ompl::base::PlannerStatus::TIMEOUT;
            }

            if (conflicts.empty()) {
                return ompl::base::PlannerStatus::EXACT_SOLUTION;
            }

            current_conflict = conflicts.front();

            if (encountered_neighbors) {
                recordHistoricalConflict(
                    Conflict{
                        current_conflict->seed_robot_i,
                        current_conflict->seed_robot_j,
                        current_conflict->conflict_timestep,
                        current_conflict->alpha,
                        current_conflict->kind,
                        current_conflict->config_i,
                        current_conflict->config_j,
                    },
                    *encountered_neighbors);
            }

            Conflict history_conflict{
                current_conflict->seed_robot_i,
                current_conflict->seed_robot_j,
                current_conflict->conflict_timestep,
                current_conflict->alpha,
                current_conflict->kind,
                current_conflict->config_i,
                current_conflict->config_j,
            };

            current_attempts.emplace(
                ResolutionAttempts{
                    std::move(history_conflict), {}});

            current_repair_id =
                next_repair_attempt_id_++;

            ++num_conflicts_;
        }

        auto subproblem = createSubProblem(
            *current_conflict,
            solution_paths_,
            *current_attempts);

        if (!subproblem) {
            return ompl::base::PlannerStatus::TIMEOUT;
        }

        const std::vector<int> repair_robots =
            subproblem->global_robot_indices;
        const int repair_start_t =
            subproblem->window_start_t;
        const int repair_end_t =
            subproblem->window_end_t;

        const std::uint64_t attempt_index =
            current_attempts->attempts.size();

        const std::uint32_t attempt_root_seed =
            arcRepairAttemptPlanningSeed(
                solve_root_seed,
                current_repair_id,
                attempt_index);

        const double elapsed_seconds =
            std::chrono::duration<double>(
                Clock::now() - solve_start)
                .count();

        const double remaining_seconds =
            overall_time_limit - elapsed_seconds;

        if (remaining_seconds <= 0.0 ||
            cancellationRequested()) {
            return ompl::base::PlannerStatus::TIMEOUT;
        }

        ++num_subproblem_attempts_;

        SubproblemSolveResult solve_result =
            solveSubProblem(
                std::move(*subproblem),
                remaining_seconds,
                attempt_root_seed,
                *current_attempts);

        if (cancellationRequested() ||
            deadlineReached()) {
            return ompl::base::PlannerStatus::TIMEOUT;
        }

        if (solve_result.outcome ==
            ResolutionOutcome::Failure) {
            continue;
        }

        spliceSolutionIntoPaths(
            repair_robots,
            repair_start_t,
            repair_end_t,
            solve_result.paths,
            solution_paths_);

        appendVisualizationRepair(
            0,
            repair_robots,
            repair_start_t,
            repair_end_t,
            solve_result.paths);

        resetConflictScanStartsForRobots(
            repair_robots,
            repair_start_t);

        recordAppliedRepairHistory(
            repair_robots,
            repair_start_t,
            repair_end_t);

        resolved_conflicts_.append(
            std::move(*current_attempts));

        current_attempts.reset();
        current_conflict.reset();
    }
}

} // namespace comotion
