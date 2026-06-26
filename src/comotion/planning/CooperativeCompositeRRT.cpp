#include "comotion/planning/CooperativeCompositeRRT.h"

#include "comotion/planning/PlanningSeed.h"

#include <ompl/base/ScopedState.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace comotion {

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kDefaultRangeFraction = 0.2;

std::uint64_t elapsedNanoseconds(const Clock::time_point &start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                             start)
            .count());
}

struct Motion {
    ob::State *state = nullptr;
    Motion *parent = nullptr;
};

struct SharedTree {
    std::unique_ptr<ompl::NearestNeighbors<Motion *>> nearest_neighbors;
    std::vector<std::unique_ptr<Motion>> motions;
    mutable std::shared_mutex mutex;
};

enum class GrowState { Trapped, Advanced, Reached };

struct GrowResult {
    GrowState state = GrowState::Trapped;
    Motion *motion = nullptr;
};

double distanceFunction(const ob::SpaceInformationPtr &si, const Motion *a,
                        const Motion *b) {
    return si->distance(a->state, b->state);
}

void initializeTree(SharedTree &tree, const ob::SpaceInformationPtr &si) {
    tree.nearest_neighbors =
        std::make_unique<ompl::NearestNeighborsGNATNoThreadSafety<Motion *>>();
    tree.nearest_neighbors->setDistanceFunction(
        [&si](const Motion *a, const Motion *b) {
            return distanceFunction(si, a, b);
        });
}

Motion *addMotion(SharedTree &tree, const ob::SpaceInformationPtr &si,
                  const ob::State *state, Motion *parent) {
    auto motion = std::make_unique<Motion>();
    motion->state = si->allocState();
    si->copyState(motion->state, state);
    motion->parent = parent;
    Motion *raw = motion.get();
    tree.motions.push_back(std::move(motion));
    tree.nearest_neighbors->add(raw);
    return raw;
}

void freeTreeStates(SharedTree &tree, const ob::SpaceInformationPtr &si) {
    for (auto &motion : tree.motions) {
        if (motion && motion->state) {
            si->freeState(motion->state);
            motion->state = nullptr;
        }
    }
}

Motion *nearestMotion(const SharedTree &tree, const Motion &query) {
    // The GNATNoThreadSafety backend can mutate internal scratch state while
    // answering nearest() queries, so serialize queries as well as inserts.
    std::unique_lock<std::shared_mutex> lock(tree.mutex);
    return tree.nearest_neighbors->nearest(const_cast<Motion *>(&query));
}

GrowResult growTree(SharedTree &tree, bool tree_starts_at_start,
                    const ob::State *target,
                    const ob::SpaceInformationPtr &owner_si,
                    const ob::SpaceInformationPtr &worker_si,
                    ob::State *scratch_state, double max_distance) {
    Motion query;
    query.state = const_cast<ob::State *>(target);
    Motion *nearest = nearestMotion(tree, query);
    if (!nearest)
        return {};

    bool reach = true;
    const ob::State *state_to_add = target;
    const double distance = owner_si->distance(nearest->state, target);
    if (distance > max_distance) {
        owner_si->getStateSpace()->interpolate(
            nearest->state, target, max_distance / distance, scratch_state);
        if (owner_si->equalStates(nearest->state, scratch_state))
            return {};
        state_to_add = scratch_state;
        reach = false;
    }

    const bool valid_motion =
        tree_starts_at_start
            ? worker_si->checkMotion(nearest->state, state_to_add)
            : worker_si->isValid(state_to_add) &&
                  worker_si->checkMotion(state_to_add, nearest->state);
    if (!valid_motion)
        return {};

    std::unique_lock<std::shared_mutex> lock(tree.mutex);
    Motion *added = addMotion(tree, owner_si, state_to_add, nearest);
    return {reach ? GrowState::Reached : GrowState::Advanced, added};
}

std::vector<ob::State *> traceToRoot(Motion *motion) {
    std::vector<ob::State *> out;
    while (motion) {
        out.push_back(motion->state);
        motion = motion->parent;
    }
    return out;
}

std::unique_ptr<og::PathGeometric> buildSolutionPath(
    const ob::SpaceInformationPtr &si, Motion *start_motion,
    Motion *goal_motion) {
    if (!start_motion || !goal_motion)
        return nullptr;

    if (start_motion->parent)
        start_motion = start_motion->parent;
    else if (goal_motion->parent)
        goal_motion = goal_motion->parent;

    const auto start_branch = traceToRoot(start_motion);
    const auto goal_branch = traceToRoot(goal_motion);

    auto path = std::make_unique<og::PathGeometric>(si);
    path->getStates().reserve(start_branch.size() + goal_branch.size());
    for (auto it = start_branch.rbegin(); it != start_branch.rend(); ++it)
        path->append(*it);
    for (ob::State *state : goal_branch)
        path->append(state);
    return path;
}

void fillCompositeState(ob::State *state,
                        const ob::SpaceInformationPtr &si,
                        const ob::RealVectorBounds &bounds,
                        ompl::RNG &rng) {
    auto *rv = state->as<ob::RealVectorStateSpace::StateType>();
    const unsigned int dim = si->getStateDimension();
    for (unsigned int i = 0; i < dim; ++i)
        rv->values[i] = rng.uniformReal(bounds.low[i], bounds.high[i]);
}

} // namespace

ompl::base::PlannerStatus CooperativeCompositeRRT::solve(double timeLimit) {
    resetPlannerRunMetrics();
    solution_paths_.clear();

    if (!problem_)
        throw std::runtime_error(
            "CooperativeCompositeRRT requires setProblem() before solve()");
    if (timeLimit <= 0.0)
        return ob::PlannerStatus::TIMEOUT;

    std::vector<int> indices = robot_indices_;
    if (indices.empty()) {
        for (int i = 0; i < problem_->numRobots(); ++i)
            indices.push_back(i);
    }
    if (indices.empty())
        return ob::PlannerStatus::INVALID_START;

    auto owner_problem = std::make_shared<MultiRobotProblem>(*problem_);
    auto owner_si = owner_problem->createCompositeSpaceInfo(indices);
    auto space = owner_si->getStateSpace();
    const auto *rv_space =
        space->as<ob::RealVectorStateSpace>();
    if (!rv_space)
        throw std::runtime_error(
            "CooperativeCompositeRRT requires a RealVector composite space");
    const ob::RealVectorBounds bounds = rv_space->getBounds();

    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    int offset = 0;
    for (int idx : indices) {
        const auto &robot = problem_->robot(idx);
        const int ndof = robot.model->numJoints();
        for (int d = 0; d < ndof; ++d) {
            start->as<ob::RealVectorStateSpace::StateType>()
                ->values[offset + d] = robot.start[static_cast<std::size_t>(d)];
            goal->as<ob::RealVectorStateSpace::StateType>()
                ->values[offset + d] = robot.goal[static_cast<std::size_t>(d)];
        }
        offset += ndof;
    }

    if (!owner_si->isValid(start.get()))
        return ob::PlannerStatus::INVALID_START;
    if (!owner_si->isValid(goal.get()))
        return ob::PlannerStatus::INVALID_GOAL;

    double max_distance = range_;
    if (max_distance <= 0.0)
        max_distance = owner_si->getMaximumExtent() * kDefaultRangeFraction;
    if (max_distance <= 0.0)
        return ob::PlannerStatus::TIMEOUT;

    SharedTree start_tree;
    SharedTree goal_tree;
    initializeTree(start_tree, owner_si);
    initializeTree(goal_tree, owner_si);
    {
        std::unique_lock<std::shared_mutex> lock(start_tree.mutex);
        addMotion(start_tree, owner_si, start.get(), nullptr);
    }
    {
        std::unique_lock<std::shared_mutex> lock(goal_tree.mutex);
        addMotion(goal_tree, owner_si, goal.get(), nullptr);
    }

    const auto solve_start = Clock::now();
    std::atomic<bool> done{false};
    std::atomic<bool> exact_found{false};
    std::atomic<std::uint64_t> global_iterations{0};
    std::atomic<int> winning_thread{-1};
    std::mutex solution_mutex;
    Motion *winning_start_motion = nullptr;
    Motion *winning_goal_motion = nullptr;
    const unsigned worker_count = std::max(1u, worker_threads_);
    std::exception_ptr worker_exception;
    std::mutex exception_mutex;

    auto worker = [&](unsigned int thread_index) {
        try {
            auto worker_problem =
                std::make_shared<MultiRobotProblem>(*problem_);
            auto worker_si = worker_problem->createCompositeSpaceInfo(indices);

            ob::State *random_state = owner_si->allocState();
            ob::State *scratch_state = owner_si->allocState();
            ompl::RNG rng(omplLocalSeedFromUserPlanningSeed(
                planning_seed_, 2002000000 + static_cast<int>(thread_index)));

            while (!done.load(std::memory_order_acquire)) {
                const double elapsed =
                    std::chrono::duration<double>(Clock::now() - solve_start)
                        .count();
                if (elapsed >= timeLimit) {
                    done.store(true, std::memory_order_release);
                    break;
                }

                const std::uint64_t iter =
                    global_iterations.fetch_add(1, std::memory_order_relaxed);
                if (max_rrt_connect_iterations_ > 0 &&
                    iter >= max_rrt_connect_iterations_) {
                    done.store(true, std::memory_order_release);
                    break;
                }

                fillCompositeState(random_state, owner_si, bounds, rng);

                const bool primary_is_start = (iter % 2u) == 0u;
                SharedTree &primary_tree =
                    primary_is_start ? start_tree : goal_tree;
                SharedTree &other_tree =
                    primary_is_start ? goal_tree : start_tree;

                GrowResult primary = growTree(
                    primary_tree, primary_is_start, random_state, owner_si,
                    worker_si, scratch_state, max_distance);
                if (primary.state == GrowState::Trapped)
                    continue;

                GrowResult connect = growTree(
                    other_tree, !primary_is_start, primary.motion->state,
                    owner_si, worker_si, scratch_state, max_distance);
                while (connect.state == GrowState::Advanced &&
                       !done.load(std::memory_order_acquire)) {
                    const double inner_elapsed =
                        std::chrono::duration<double>(Clock::now() -
                                                      solve_start)
                            .count();
                    if (inner_elapsed >= timeLimit) {
                        done.store(true, std::memory_order_release);
                        break;
                    }
                    connect = growTree(
                        other_tree, !primary_is_start, primary.motion->state,
                        owner_si, worker_si, scratch_state, max_distance);
                }

                if (connect.state != GrowState::Reached)
                    continue;

                bool expected = false;
                if (exact_found.compare_exchange_strong(
                        expected, true, std::memory_order_acq_rel)) {
                    done.store(true, std::memory_order_release);
                    Motion *start_motion =
                        primary_is_start ? primary.motion : connect.motion;
                    Motion *goal_motion =
                        primary_is_start ? connect.motion : primary.motion;
                    {
                        std::lock_guard<std::mutex> lock(solution_mutex);
                        winning_start_motion = start_motion;
                        winning_goal_motion = goal_motion;
                        winning_thread.store(static_cast<int>(thread_index),
                                             std::memory_order_release);
                    }
                }
                break;
            }

            owner_si->freeState(random_state);
            owner_si->freeState(scratch_state);
        } catch (const std::exception &ex) {
            done.store(true, std::memory_order_release);
            std::lock_guard<std::mutex> lock(exception_mutex);
            if (!worker_exception)
                worker_exception = std::current_exception();
        } catch (...) {
            done.store(true, std::memory_order_release);
            std::lock_guard<std::mutex> lock(exception_mutex);
            if (!worker_exception)
                worker_exception = std::current_exception();
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(worker_count);
    for (unsigned int i = 0; i < worker_count; ++i)
        threads.emplace_back(worker, i);
    for (auto &thread : threads)
        thread.join();

    if (worker_exception)
        std::rethrow_exception(worker_exception);

    const auto solve_wall_ns = elapsedNanoseconds(solve_start);

    auto makeStatsJson = [&](bool exact) {
        nlohmann::json stats;
        stats["cooperative_composite_rrt"] = {
            {"worker_threads", worker_count},
            {"max_rrt_connect_iterations", max_rrt_connect_iterations_},
            {"iterations", global_iterations.load()},
            {"range", max_distance},
            {"solve_wall_seconds",
             static_cast<double>(solve_wall_ns) * 1e-9},
            {"exact_solution", exact},
            {"winning_thread", winning_thread.load()},
            {"start_tree_states", start_tree.motions.size()},
            {"goal_tree_states", goal_tree.motions.size()},
        };
        return stats;
    };

    std::unique_ptr<og::PathGeometric> solution_path;
    if (exact_found.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lock(solution_mutex);
        solution_path =
            buildSolutionPath(owner_si, winning_start_motion,
                              winning_goal_motion);
    }

    if (!exact_found.load(std::memory_order_acquire) || !solution_path) {
        setPlannerStatsJson(makeStatsJson(false));
        freeTreeStates(start_tree, owner_si);
        freeTreeStates(goal_tree, owner_si);
        return ob::PlannerStatus::TIMEOUT;
    }

    if (simplify_solution_) {
        og::PathSimplifier simplifier(owner_si);
        simplifier.simplify(*solution_path, 0.0);
    }
    solution_path->interpolate();
    solution_paths_ = splitCompositePathToRobotPaths(*solution_path, *problem_,
                                                     indices,
                                                     "CooperativeCompositeRRT");
    setSolutionMetricsFromPaths(solution_paths_);
    setPlannerStatsJson(makeStatsJson(true));

    freeTreeStates(start_tree, owner_si);
    freeTreeStates(goal_tree, owner_si);
    return ob::PlannerStatus::EXACT_SOLUTION;
}

} // namespace comotion
