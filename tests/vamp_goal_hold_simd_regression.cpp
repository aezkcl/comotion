#include "comotion/collision/CollisionChecker.h"
#include "comotion/robot/FlyingSphere.h"

#include <iostream>
#include <memory>
#include <vector>

namespace {

std::shared_ptr<comotion::FlyingSphere> makeSphereRobot() {
    return std::make_shared<comotion::FlyingSphere>(
        0.6, std::vector<double>{-10.0, -10.0, -10.0},
        std::vector<double>{10.0, 10.0, 10.0});
}

} // namespace

int main() {
    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Vamp);

    auto goal_robot = makeSphereRobot();
    auto prior_robot = makeSphereRobot();

    comotion::Path prior_path;
    prior_path.push_back({0.0, 0.0, 0.0});
    prior_path.push_back({1.0, 0.0, 0.0});
    prior_path.push_back({2.0, 0.0, 0.0});
    prior_path.push_back({3.0, 0.0, 0.0});
    prior_path.push_back({4.0, 0.0, 0.0});

    auto constraint = checker.computeGoalHoldConstraint(
        *goal_robot, std::vector<double>{2.0, 0.0, 0.0}, *prior_robot,
        prior_path);
    if (constraint.permanently_blocked ||
        constraint.min_safe_arrival_timestep != 4) {
        std::cerr << "vamp_goal_hold_simd_regression: unexpected constraint "
                     "result\n";
        return 1;
    }

    std::cout << "vamp_goal_hold_simd_regression: OK\n";
    return 0;
}
