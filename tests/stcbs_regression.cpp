#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/STCBS.h"
#include "comotion/planning/USTRRTstar.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

#include <ompl/base/PlannerStatus.h>

#include <cstdint>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

double vectorDistance(const std::vector<double> &a,
                      const std::vector<double> &b) {
    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

std::shared_ptr<comotion::FlyingSphere> makeSphereRobot(double radius = 0.4) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{-10.0, -10.0, -10.0},
        std::vector<double>{10.0, 10.0, 10.0});
}

std::shared_ptr<comotion::FlyingSphere> makeLineRobot(double radius = 0.4) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{-10.0, -0.01, -0.01},
        std::vector<double>{10.0, 0.01, 0.01});
}

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "stcbs_regression: " << label
                  << " expected true\n";
        return false;
    }
    return true;
}

bool expectFalse(const std::string &label, bool value) {
    if (value) {
        std::cerr << "stcbs_regression: " << label
                  << " expected false\n";
        return false;
    }
    return true;
}

bool expectEq(const std::string &label, std::size_t actual,
              std::size_t expected) {
    if (actual != expected) {
        std::cerr << "stcbs_regression: " << label << " expected "
                  << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

bool expectNear(const std::string &label, double actual, double expected,
                double tol = 1e-6) {
    if (std::abs(actual - expected) > tol) {
        std::cerr << "stcbs_regression: " << label << " expected "
                  << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

template <typename Fn>
bool expectRuntimeError(const std::string &label, Fn &&fn) {
    try {
        fn();
    } catch (const std::runtime_error &) {
        return true;
    } catch (const std::exception &ex) {
        std::cerr << "stcbs_regression: " << label
                  << " expected std::runtime_error, got different exception: "
                  << ex.what() << "\n";
        return false;
    }

    std::cerr << "stcbs_regression: " << label
              << " expected std::runtime_error\n";
    return false;
}

comotion::USTRRTstar::BranchConstraint makeMidEdgeConstraint(
    const comotion::USTRRTstar::Result &result, const comotion::RobotModel &other_model,
    std::size_t constrained_segment, std::size_t resolution) {
    const double t0 = result.raw_times_seconds[constrained_segment - 1];
    const double t1 = result.raw_times_seconds[constrained_segment];
    const double query_time = 0.5 * (t0 + t1);
    const double alpha = (query_time - t0) / (t1 - t0);

    comotion::USTRRTstar::BranchConstraint constraint;
    constraint.constrained_agent_id = 0;
    constraint.other_agent_id = 1;
    constraint.timestep = static_cast<int>(
        std::llround(query_time * static_cast<double>(resolution)));
    constraint.time_seconds = query_time;
    constraint.other_model = &other_model;
    constraint.other_config = comotion::interpolateConfig(
        result.raw_path[constrained_segment - 1],
        result.raw_path[constrained_segment], alpha);
    return constraint;
}

std::size_t findMovingSegment(const comotion::USTRRTstar::Result &result) {
    for (std::size_t i = 1; i < result.raw_path.size(); ++i) {
        if (vectorDistance(result.raw_path[i - 1], result.raw_path[i]) > 1e-4)
            return i;
    }
    return result.raw_path.size();
}

std::size_t countActiveDescendants(const comotion::USTRRTstar::Motion *root) {
    if (root == nullptr)
        return 0;
    std::size_t count = 0;
    std::vector<const comotion::USTRRTstar::Motion *> stack(root->children.begin(),
                                                           root->children.end());
    while (!stack.empty()) {
        const auto *motion = stack.back();
        stack.pop_back();
        if (motion->active)
            ++count;
        for (const auto *child : motion->children)
            stack.push_back(child);
    }
    return count;
}

std::vector<double> motionConfig(const comotion::USTRRTstar::Motion &motion,
                                 int ndof) {
    const auto *compound =
        motion.state->as<ompl::base::CompoundState>();
    const auto *rv =
        compound->as<ompl::base::RealVectorStateSpace::StateType>(0);
    std::vector<double> config(static_cast<std::size_t>(ndof));
    for (int i = 0; i < ndof; ++i)
        config[static_cast<std::size_t>(i)] = rv->values[i];
    return config;
}

std::optional<std::pair<int, int>> findSameLayerNeighborPair(
    const comotion::USTRRTstar::TreeSnapshot &tree, double occupied_radius) {
    const double layer_tol =
        std::max(1e-9, tree.params.layer_dt_seconds * 0.5);
    const int ndof = tree.model->numJoints();
    for (const auto &lhs_ptr : tree.motions) {
        const auto *lhs = lhs_ptr.get();
        if (lhs == nullptr || !lhs->active || lhs->marked)
            continue;
        const auto lhs_cfg = motionConfig(*lhs, ndof);
        const double lhs_time =
            ompl::base::SpaceTimeStateSpace::getStateTime(lhs->state);
        for (const auto &rhs_ptr : tree.motions) {
            const auto *rhs = rhs_ptr.get();
            if (rhs == nullptr || rhs == lhs || !rhs->active || rhs->marked)
                continue;
            const double rhs_time =
                ompl::base::SpaceTimeStateSpace::getStateTime(rhs->state);
            if (std::abs(lhs_time - rhs_time) > layer_tol)
                continue;
            const auto rhs_cfg = motionConfig(*rhs, ndof);
            if (vectorDistance(lhs_cfg, rhs_cfg) <= occupied_radius)
                return std::make_pair(lhs->index, rhs->index);
        }
    }
    return std::nullopt;
}

std::shared_ptr<comotion::MultiRobotProblem> makeRigidBodyConflictProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(128);
    problem->setVmax(2.0);
    problem->setObstacles(
        {comotion::ObstacleSphere{Eigen::Vector3d(0.0, 0.0, 0.75), 1.5}});

    const std::vector<double> env_min{-12.0, -12.0, 0.0};
    const std::vector<double> env_max{12.0, 12.0, 1.5};
    problem->addRobot(std::make_shared<comotion::FlyingSphere>(1.0, env_min, env_max),
                      {10.0, 0.0, 0.75}, {-10.0, 0.0, 0.75});
    problem->addRobot(std::make_shared<comotion::FlyingSphere>(1.0, env_min, env_max),
                      {0.0, 10.0, 0.75}, {0.0, -10.0, 0.75});
    problem->addRobot(std::make_shared<comotion::FlyingSphere>(1.0, env_min, env_max),
                      {-10.0, 0.0, 0.75}, {10.0, 0.0, 0.75});
    problem->addRobot(std::make_shared<comotion::FlyingSphere>(1.0, env_min, env_max),
                      {0.0, -10.0, 0.75}, {0.0, 10.0, 0.75});
    return problem;
}

bool testNewUstrrtTimeSemantics() {
    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Spheres);
    auto robot = makeSphereRobot();

    comotion::USTRRTstar::Params params;
    params.range = 6.0;
    params.max_iterations = 2000;
    params.max_samples = 30000;
    params.goal_threshold = 0.2;
    params.rewire_mode = comotion::USTRRTstar::RewireMode::KNearest;
    params.rewire_k = 12;
    params.layer_dt_seconds = 1.0;

    auto [tree, result] = comotion::USTRRTstar::buildTree(
        0, {-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, *robot, checker, params, 8, 1.0,
        0.5);

    if (!expectTrue("low-level exact solution", result.exact))
        return false;
    if (!expectTrue("tree exists", static_cast<bool>(tree)))
        return false;
    if (!expectTrue("dense path has timesteps", result.dense_path.has_timesteps()))
        return false;
    if (!expectEq("raw path count matches raw time count", result.raw_path.size(),
                  result.raw_times_seconds.size()))
        return false;
    if (!expectEq("dense mapping size matches dense path size",
                  result.dense_timestep_to_motion_index.size(),
                  result.dense_path.size()))
        return false;
    if (!expectNear("arrival time matches final raw time",
                    result.arrival_time_seconds, result.raw_times_seconds.back()))
        return false;

    for (std::size_t i = 1; i < result.raw_times_seconds.size(); ++i) {
        const double prev_t = result.raw_times_seconds[i - 1];
        const double cur_t = result.raw_times_seconds[i];
        if (!expectTrue("raw times strictly increase", cur_t > prev_t))
            return false;

        const double delta_t = cur_t - prev_t;
        const double delta_x =
            vectorDistance(result.raw_path[i - 1], result.raw_path[i]);
        if (!expectTrue("speed bound respected", delta_x <= delta_t * 1.0 + 1e-6))
            return false;
    }

    if (!expectEq("dense path final timestep matches dense path length",
                  result.dense_path.arrival_timestep() + 1,
                  result.dense_path.size()))
        return false;

    return true;
}

bool testNewUstrrtExactTimeConstraintEnforcement() {
    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Spheres);
    auto robot = makeSphereRobot();

    comotion::USTRRTstar::Params params;
    params.range = 6.0;
    params.max_iterations = 2500;
    params.max_samples = 40000;
    params.goal_threshold = 0.2;
    params.rewire_mode = comotion::USTRRTstar::RewireMode::KNearest;
    params.rewire_k = 12;
    params.layer_dt_seconds = 1.0;

    auto [tree, result] = comotion::USTRRTstar::buildTree(
        0, {-3.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, *robot, checker, params, 128, 1.0,
        0.5);

    if (!expectTrue("exact-time setup exact solution", result.exact))
        return false;
    if (!expectTrue("exact-time path has enough waypoints",
                    result.motion_indices.size() >= 3))
        return false;

    const std::size_t constrained_segment = findMovingSegment(result);
    if (!expectTrue("found moving segment",
                    constrained_segment < result.motion_indices.size()))
        return false;
    const int blocked_motion_index = result.motion_indices[constrained_segment];
    auto *blocked_motion =
        tree->motions[static_cast<std::size_t>(blocked_motion_index)].get();
    if (!expectTrue("blocked motion active before prune", blocked_motion->active))
        return false;

    const auto constraint =
        makeMidEdgeConstraint(result, *robot, constrained_segment, 128);
    if (!expectFalse("constraint time should be off layer boundary",
                     constraint.timestep % 128 == 0)) {
        return false;
    }

    auto motion_check_tree = tree->clone();
    motion_check_tree->constraints.push_back(constraint);
    auto *motion_check_parent =
        motion_check_tree->motions[static_cast<std::size_t>(
            result.motion_indices[constrained_segment - 1])]
            .get();
    auto *motion_check_child =
        motion_check_tree->motions[static_cast<std::size_t>(blocked_motion_index)]
            .get();
    if (!expectFalse("exact-time constraint should reject edge motion",
                     motion_check_tree->si->checkMotion(
                         motion_check_parent->state, motion_check_child->state)))
        return false;

    auto prune_tree = tree->clone();
    auto *pruned_blocked_motion =
        prune_tree->motions[static_cast<std::size_t>(blocked_motion_index)].get();
    const auto active_before = prune_tree->activeMotionCount();

    if (!expectTrue("pruneWithConstraint succeeded",
                    comotion::USTRRTstar::pruneWithConstraint(*prune_tree,
                                                             constraint)))
        return false;
    if (!expectFalse("blocked motion active after prune",
                     pruned_blocked_motion->active))
        return false;
    if (!expectTrue("start motion still active after prune",
                    prune_tree->start_motion != nullptr &&
                        prune_tree->start_motion->active))
        return false;
    if (!expectTrue("constraint recorded", !prune_tree->constraints.empty()))
        return false;
    if (!expectTrue("tree retains some active motions",
                    prune_tree->activeMotionCount() > 0))
        return false;
    if (!expectTrue("prune removes at least one motion",
                    prune_tree->activeMotionCount() < active_before))
        return false;

    auto pruned_best = comotion::USTRRTstar::extractBestPath(*prune_tree, 0.5);
    if (pruned_best.exact) {
        for (int motion_index : pruned_best.motion_indices) {
            if (!expectTrue("pruned solution excludes blocked motion",
                            motion_index != blocked_motion_index)) {
                return false;
            }
        }
    }

    return true;
}

bool testNewUstrrtDuplicateConstraintThrows() {
    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Spheres);
    auto robot = makeSphereRobot();

    comotion::USTRRTstar::Params params;
    params.range = 6.0;
    params.max_iterations = 2500;
    params.max_samples = 40000;
    params.goal_threshold = 0.2;
    params.rewire_mode = comotion::USTRRTstar::RewireMode::KNearest;
    params.rewire_k = 12;
    params.layer_dt_seconds = 1.0;

    auto [tree, result] = comotion::USTRRTstar::buildTree(
        0, {-3.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, *robot, checker, params, 128,
        1.0, 0.5);
    if (!expectTrue("duplicate constraint setup exact solution", result.exact))
        return false;

    const std::size_t constrained_segment = findMovingSegment(result);
    if (!expectTrue("duplicate constraint found moving segment",
                    constrained_segment < result.motion_indices.size()))
        return false;

    const auto constraint =
        makeMidEdgeConstraint(result, *robot, constrained_segment, 128);
    if (!expectTrue("first pruneWithConstraint succeeds",
                    comotion::USTRRTstar::pruneWithConstraint(*tree, constraint)))
        return false;

    return expectRuntimeError("duplicate constraint throws", [&]() {
        comotion::USTRRTstar::pruneWithConstraint(*tree, constraint);
    });
}

bool testDenseConflictConstraintChangesTree() {
    auto problem = makeRigidBodyConflictProblem();

    comotion::USTRRTstar::Params params;
    params.range = 10.0;
    params.max_iterations = 1000;
    params.max_samples = 100000;
    params.goal_threshold = 0.1;
    params.rewire_mode = comotion::USTRRTstar::RewireMode::KNearest;
    params.rewire_radius = 1.0;
    params.rewire_k = 10;
    params.layer_dt_seconds = 1.0;

    std::vector<std::shared_ptr<comotion::USTRRTstar::TreeSnapshot>> trees;
    std::vector<comotion::USTRRTstar::Result> results;
    trees.reserve(static_cast<std::size_t>(problem->numRobots()));
    results.reserve(static_cast<std::size_t>(problem->numRobots()));
    for (int robot_idx = 0; robot_idx < problem->numRobots(); ++robot_idx) {
        auto [tree, result] = comotion::USTRRTstar::buildTree(
            robot_idx, problem->robot(robot_idx).start,
            problem->robot(robot_idx).goal, *problem->robot(robot_idx).model,
            problem->collisionChecker(), params, problem->resolution(),
            problem->vmax(), 0.5);
        if (!expectTrue("dense conflict setup exact path", result.exact))
            return false;
        trees.push_back(std::move(tree));
        results.push_back(std::move(result));
    }

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    auto ptrs = problem->robotModelPtrs();
    std::vector<comotion::Path> dense_paths;
    dense_paths.reserve(results.size());
    for (const auto &result : results)
        dense_paths.push_back(result.dense_path);
    auto conflict = problem->collisionChecker().findFirstCompositePathConflict(
        dense_paths, ptrs, options);
    if (!expectTrue("dense conflict exists", conflict.has_value()))
        return false;
    if (!expectFalse("dense conflict should be mid-edge",
                     conflict->timestep % problem->resolution() == 0))
        return false;

    auto branch_changes_tree = [&](int constrained_robot, int other_robot,
                                   const std::vector<double> &other_config) {
        auto constrained_tree =
            trees[static_cast<std::size_t>(constrained_robot)]->clone();
        const auto active_before = constrained_tree->activeMotionCount();

        comotion::USTRRTstar::BranchConstraint constraint;
        constraint.constrained_agent_id = constrained_robot;
        constraint.other_agent_id = other_robot;
        constraint.timestep = static_cast<int>(conflict->timestep);
        constraint.time_seconds =
            static_cast<double>(conflict->timestep) /
            static_cast<double>(problem->resolution());
        constraint.other_model = problem->robot(other_robot).model.get();
        constraint.other_config = other_config;

        if (!comotion::USTRRTstar::pruneWithConstraint(*constrained_tree,
                                                      constraint)) {
            return true;
        }
        if (constrained_tree->activeMotionCount() < active_before)
            return true;

        const auto pruned_best =
            comotion::USTRRTstar::extractBestPath(*constrained_tree, 0.5);
        return !pruned_best.exact ||
               pruned_best.motion_indices !=
                   results[static_cast<std::size_t>(constrained_robot)]
                       .motion_indices;
    };

    if (!expectTrue(
            "dense conflict changes at least one constrained branch",
            branch_changes_tree(conflict->robot_i, conflict->robot_j,
                                conflict->config_j) ||
                branch_changes_tree(conflict->robot_j, conflict->robot_i,
                                    conflict->config_i))) {
        return false;
    }

    return true;
}

bool testNewUstrrtStructuralPruneMarksBranch() {
    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Spheres);
    auto robot = makeSphereRobot();

    comotion::USTRRTstar::Params params;
    params.range = 6.0;
    params.max_iterations = 2500;
    params.max_samples = 40000;
    params.goal_threshold = 0.2;
    params.rewire_mode = comotion::USTRRTstar::RewireMode::KNearest;
    params.rewire_k = 12;
    params.layer_dt_seconds = 1.0;

    auto [tree, result] = comotion::USTRRTstar::buildTree(
        0, {-3.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, *robot, checker, params, 128,
        1.0, 0.5);
    if (!expectTrue("structural prune setup exact solution", result.exact))
        return false;

    const std::size_t constrained_segment = findMovingSegment(result);
    if (!expectTrue("structural prune found moving segment",
                    constrained_segment < result.motion_indices.size()))
        return false;

    const int blocked_motion_index = result.motion_indices[constrained_segment];
    auto prune_tree = tree->clone();
    auto *blocked_motion =
        prune_tree->motions[static_cast<std::size_t>(blocked_motion_index)].get();
    const auto usable_before = prune_tree->usableMotionCount();
    if (!expectTrue("blocked motion has descendants before prune",
                    countActiveDescendants(blocked_motion) > 0)) {
        return false;
    }

    comotion::USTRRTstar::pruneDescendants(*prune_tree, blocked_motion_index);
    comotion::USTRRTstar::markMotion(*prune_tree, blocked_motion_index);
    prune_tree->rebuildNearestNeighbors();

    if (!expectTrue("blocked motion stays active after mark",
                    blocked_motion->active))
        return false;
    if (!expectTrue("blocked motion marked after structural prune",
                    blocked_motion->marked))
        return false;
    if (!expectEq("descendants pruned after structural prune",
                  countActiveDescendants(blocked_motion), std::size_t{0}))
        return false;
    if (!expectTrue("usable motions drop after structural prune",
                    prune_tree->usableMotionCount() < usable_before))
        return false;

    const auto pruned_best =
        comotion::USTRRTstar::extractBestPath(*prune_tree, 0.5);
    if (pruned_best.exact) {
        for (int motion_index : pruned_best.motion_indices) {
            if (!expectTrue("marked motion excluded from best path",
                            motion_index != blocked_motion_index)) {
                return false;
            }
        }
    }

    return true;
}

bool testNewUstrrtNeighborPruneSameLayer() {
    auto problem = makeRigidBodyConflictProblem();

    comotion::USTRRTstar::Params params;
    params.range = 10.0;
    params.max_iterations = 1000;
    params.max_samples = 100000;
    params.goal_threshold = 0.1;
    params.rewire_mode = comotion::USTRRTstar::RewireMode::KNearest;
    params.rewire_radius = 1.0;
    params.rewire_k = 10;
    params.layer_dt_seconds = 1.0;

    auto [tree, result] = comotion::USTRRTstar::buildTree(
        0, problem->robot(0).start, problem->robot(0).goal,
        *problem->robot(0).model, problem->collisionChecker(), params,
        problem->resolution(), problem->vmax(), 0.5);
    if (!expectTrue("neighbor prune setup exact solution", result.exact))
        return false;

    constexpr double kOccupiedRadius = 3.0;
    const auto pair = findSameLayerNeighborPair(*tree, kOccupiedRadius);
    if (!expectTrue("found same-layer neighbor pair", pair.has_value()))
        return false;

    auto prune_tree = tree->clone();
    auto *root_motion =
        prune_tree->motions[static_cast<std::size_t>(pair->first)].get();
    auto *neighbor_motion =
        prune_tree->motions[static_cast<std::size_t>(pair->second)].get();
    const auto active_before = prune_tree->activeMotionCount();

    comotion::USTRRTstar::pruneNeighbors(*prune_tree, pair->first,
                                        kOccupiedRadius);
    prune_tree->rebuildNearestNeighbors();

    if (!expectTrue("root motion remains active after neighbor prune",
                    root_motion->active))
        return false;
    if (!expectFalse("neighbor motion pruned by neighbor prune",
                     neighbor_motion->active))
        return false;
    if (!expectTrue("neighbor prune reduces active motion count",
                    prune_tree->activeMotionCount() < active_before))
        return false;

    return true;
}

bool testNewStcbsPostArrivalGoalHoldScenario() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(128);
    problem->setVmax(1.0);

    problem->addRobot(makeSphereRobot(), {-0.25, 0.0, 0.0}, {0.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {2.0, 0.0, 0.0}, {-1.0, 0.0, 0.0});

    comotion::STCBS planner;
    planner.setProblem(problem);
    planner.setRange(1.0);
    planner.setMaxIterations(1500);
    planner.setMaxSamples(8000);
    planner.setGoalThreshold(0.05);
    planner.setLayerDtSeconds(1.0);
    planner.setRewireMode(comotion::USTRRTstar::RewireMode::KNearest);
    planner.setRewireK(8);
    planner.setMaxCTNodes(1024);
    planner.setOccupiedRadius(0.1);
    planner.setPlanningSeed(7);

    // Legacy `STCBS::solve` re-seeded OMPL with `omplRootSeed(7)` (=8) before planning.
    comotion::seedOmplGlobalFromUserPlanningSeed(7);

    const auto status = planner.solve(20.0);
    if (status != ompl::base::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "stcbs_regression: expected exact post-arrival "
                     "STCBS solution, got "
                  << status.asString() << "\n";
        return false;
    }

    auto paths = planner.getSolutionPaths();
    if (!expectEq("post-arrival STCBS path count", paths.size(), 2))
        return false;

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    auto ptrs = problem->robotModelPtrs();
    auto conflict = problem->collisionChecker().findFirstCompositePathConflict(
        paths, ptrs, options);
    if (!expectFalse("post-arrival STCBS returned conflict-free solution",
                     conflict.has_value())) {
        return false;
    }

    const auto goal_hold = problem->collisionChecker().computeGoalHoldConstraint(
        *problem->robot(0).model, problem->robot(0).goal,
        *problem->robot(1).model, paths[1]);
    if (!expectFalse("post-arrival goal hold is not permanently blocked",
                     goal_hold.permanently_blocked))
        return false;
    if (!expectTrue("post-arrival goal hold requires delayed arrival",
                    goal_hold.min_safe_arrival_timestep > 0))
        return false;

    const auto arrival_ts = paths[0].arrival_timestep();
    if (!expectTrue("STCBS respects post-arrival minimum safe arrival",
                    arrival_ts >= goal_hold.min_safe_arrival_timestep))
        return false;

    return true;
}

bool testNewStcbsCrossingScenario() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(4);
    problem->setVmax(1.0);

    problem->addRobot(makeSphereRobot(), {-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0});
    problem->addRobot(makeSphereRobot(), {0.0, -2.0, 0.0}, {0.0, 2.0, 0.0});

    comotion::STCBS planner;
    planner.setProblem(problem);
    planner.setRange(6.0);
    planner.setMaxIterations(3000);
    planner.setMaxSamples(50000);
    planner.setGoalThreshold(0.25);
    planner.setLayerDtSeconds(1.0);
    planner.setRewireMode(comotion::USTRRTstar::RewireMode::KNearest);
    planner.setRewireK(12);
    planner.setMaxCTNodes(256);

    const auto status = planner.solve(10.0);
    if (status != ompl::base::PlannerStatus::EXACT_SOLUTION) {
        std::cerr << "stcbs_regression: expected exact STCBS solution, "
                     "got "
                  << status.asString() << "\n";
        return false;
    }

    auto paths = planner.getSolutionPaths();
    if (!expectEq("STCBS path count", paths.size(), 2))
        return false;
    for (const auto &path : paths) {
        if (!expectTrue("STCBS output path has timesteps",
                        path.has_timesteps())) {
            return false;
        }
    }

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    auto ptrs = problem->robotModelPtrs();
    auto conflict = problem->collisionChecker().findFirstCompositePathConflict(
        paths, ptrs, options);
    if (!expectFalse("STCBS returned conflict-free solution",
                     conflict.has_value())) {
        return false;
    }

    if (!expectTrue("STCBS sum_of_cost metric populated",
                    planner.sumOfCostTimesteps().has_value()))
        return false;
    if (!expectTrue("STCBS makespan metric populated",
                    planner.makespanTimesteps().has_value()))
        return false;

    const auto &planner_stats = planner.plannerStatsJson();
    if (!expectTrue("planner stats include num_conflicts",
                    planner_stats.contains("num_conflicts")))
        return false;
    if (!expectTrue("planner stats include ct_nodes_expanded",
                    planner_stats.contains("ct_nodes_expanded")))
        return false;
    if (!expectTrue("planner stats include ust_rrt_calls_total",
                    planner_stats.contains("ust_rrt_calls_total")))
        return false;
    if (!expectTrue("planner stats include ust_rrt_calls_successes",
                    planner_stats.contains("ust_rrt_calls_successes")))
        return false;
    if (!expectTrue("planner stats include ust_rrt_success_rate",
                    planner_stats.contains("ust_rrt_success_rate")))
        return false;
    if (!expectTrue("ct nodes expanded >= 1",
                    planner_stats["ct_nodes_expanded"]
                            .get<std::uint64_t>() >= 1))
        return false;
    const auto ust_calls_total =
        planner_stats["ust_rrt_calls_total"].get<std::uint64_t>();
    const auto ust_calls_successes =
        planner_stats["ust_rrt_calls_successes"].get<std::uint64_t>();
    if (!expectTrue("UST-RRT successes do not exceed total calls",
                    ust_calls_total >= ust_calls_successes))
        return false;
    const double ust_success_rate =
        planner_stats["ust_rrt_success_rate"].get<double>();
    if (!expectTrue("UST-RRT success rate in [0, 1]",
                    ust_success_rate >= 0.0 && ust_success_rate <= 1.0))
        return false;

    return true;
}

} // namespace

int main() {
    comotion::seedOmplGlobalFromUserPlanningSeed(6);
    if (!testNewUstrrtTimeSemantics())
        return 1;
    if (!testNewUstrrtExactTimeConstraintEnforcement())
        return 1;
    if (!testNewUstrrtDuplicateConstraintThrows())
        return 1;
    if (!testDenseConflictConstraintChangesTree())
        return 1;
    if (!testNewUstrrtStructuralPruneMarksBranch())
        return 1;
    if (!testNewUstrrtNeighborPruneSameLayer())
        return 1;
    if (!testNewStcbsPostArrivalGoalHoldScenario())
        return 1;
    if (!testNewStcbsCrossingScenario())
        return 1;
    return 0;
}
