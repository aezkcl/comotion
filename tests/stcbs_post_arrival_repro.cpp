#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/USTRRTstar.h"
#include "comotion/planning/Path.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

std::shared_ptr<comotion::FlyingSphere> makeSphereRobot(double radius = 0.4) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{-10.0, -10.0, -10.0},
        std::vector<double>{10.0, 10.0, 10.0});
}

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "stcbs_post_arrival_repro: " << label
                  << " expected true\n";
        return false;
    }
    return true;
}

bool expectFalse(const std::string &label, bool value) {
    if (value) {
        std::cerr << "stcbs_post_arrival_repro: " << label
                  << " expected false\n";
        return false;
    }
    return true;
}

template <typename T>
bool expectEq(const std::string &label, const T &actual, const T &expected) {
    if (actual != expected) {
        std::cerr << "stcbs_post_arrival_repro: " << label
                  << " expected " << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

std::vector<double> configAtTimeOnResult(
    const comotion::USTRRTstar::Result &result, double query_time,
    double time_eps) {
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
            const double alpha =
                (std::abs(t1 - t0) <= 1e-12)
                    ? 1.0
                    : std::clamp((query_time - t0) / (t1 - t0), 0.0, 1.0);
            return comotion::interpolateConfig(result.raw_path[i],
                                           result.raw_path[i + 1], alpha);
        }
    }

    return result.raw_path.back();
}

bool testPostArrivalConstraintReuseGap() {
    constexpr std::size_t kResolution = 128;
    constexpr double kVmax = 1.0;
    constexpr double kLambda = 0.5;
    constexpr double kTimeEps = 0.5 / static_cast<double>(kResolution);

    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Spheres);
    auto constrained_robot = makeSphereRobot(0.4);
    auto other_robot = makeSphereRobot(0.4);

    comotion::USTRRTstar::Params params;
    params.range = 1.0;
    params.max_iterations = 512;
    params.max_samples = 4000;
    params.goal_threshold = 0.05;
    params.rewire_mode = comotion::USTRRTstar::RewireMode::KNearest;
    params.rewire_k = 8;
    params.layer_dt_seconds = 1.0;

    auto [tree, result] = comotion::USTRRTstar::buildTree(
        0, {-0.25, 0.0, 0.0}, {0.0, 0.0, 0.0}, *constrained_robot, checker,
        params, kResolution, kVmax, kLambda);

    if (!expectTrue("constrained robot exact path", result.exact))
        return false;

    const std::size_t arrival_ts = static_cast<std::size_t>(
        std::llround(result.arrival_time_seconds *
                     static_cast<double>(kResolution)));
    if (!expectTrue("constrained arrival is positive", arrival_ts > 0))
        return false;

    comotion::Path other_path;
    other_path.push_back({2.0, 0.0, 0.0});
    other_path.push_back({1.0, 0.0, 0.0});
    other_path.push_back({0.0, 0.0, 0.0});
    other_path.push_back({-1.0, 0.0, 0.0});
    other_path.set_waypoint_timesteps(
        {0, arrival_ts, arrival_ts + kResolution,
         arrival_ts + 2 * kResolution});
    other_path.interpolate_to_timesteps(kResolution, kVmax);

    comotion::CompositePathValidationOptions options;
    options.check_environment = false;
    std::vector<comotion::Path> dense_paths{result.dense_path, other_path};
    std::vector<const comotion::RobotModel *> robots{constrained_robot.get(),
                                                 other_robot.get()};
    auto conflict =
        checker.findFirstCompositePathConflict(dense_paths, robots, options);
    if (!expectTrue("post-arrival conflict exists", conflict.has_value()))
        return false;

    if (!expectTrue("conflict occurs after constrained arrival",
                    conflict->timestep > arrival_ts))
        return false;

    const auto goal_hold = checker.computeGoalHoldConstraint(
        *constrained_robot, std::vector<double>{0.0, 0.0, 0.0},
        *other_robot, other_path);
    if (!expectFalse("goal hold is not permanently blocked",
                     goal_hold.permanently_blocked))
        return false;
    if (!expectTrue("goal hold requires later arrival",
                    goal_hold.min_safe_arrival_timestep > arrival_ts))
        return false;

    auto pruned_tree = tree->clone();
    const auto active_before = pruned_tree->activeMotionCount();

    comotion::USTRRTstar::BranchConstraint constraint;
    constraint.constrained_agent_id = 0;
    constraint.other_agent_id = 1;
    constraint.timestep = static_cast<int>(conflict->timestep);
    constraint.time_seconds =
        static_cast<double>(conflict->timestep) /
        static_cast<double>(kResolution);
    constraint.other_model = other_robot.get();
    constraint.other_config =
        (conflict->robot_i == 0) ? conflict->config_j : conflict->config_i;

    if (!expectTrue("post-arrival prune succeeds",
                    comotion::USTRRTstar::pruneWithConstraint(*pruned_tree,
                                                             constraint)))
        return false;
    if (!expectEq("post-arrival prune leaves active count unchanged",
                  pruned_tree->activeMotionCount(), active_before))
        return false;

    const auto pruned_best =
        comotion::USTRRTstar::extractBestPath(*pruned_tree, kLambda);
    if (!expectTrue("pruned best path remains exact", pruned_best.exact))
        return false;
    if (!expectTrue("pruned best path reuses original motion chain",
                    pruned_best.motion_indices == result.motion_indices))
        return false;

    const auto sampled_config = configAtTimeOnResult(
        pruned_best, constraint.time_seconds, kTimeEps);
    if (!expectFalse("reused path still violates post-arrival constraint",
                     checker.isValidPair(*constrained_robot, sampled_config,
                                         *other_robot,
                                         constraint.other_config)))
        return false;

    std::cout << "stcbs_post_arrival_repro: conflict_timestep="
              << conflict->timestep << " arrival_timestep=" << arrival_ts
              << " safe_arrival_timestep="
              << goal_hold.min_safe_arrival_timestep << "\n";
    return true;
}

} // namespace

int main() {
    comotion::seedOmplGlobalFromUserPlanningSeed(6);
    return testPostArrivalConstraintReuseGap() ? 0 : 1;
}
