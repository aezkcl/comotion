#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/robot/FlyingSphere.h"

#include <iostream>
#include <memory>
#include <vector>

int main() {
    auto robot = std::make_shared<comotion::FlyingSphere>(
        0.25, std::vector<double>{-1.0, -1.0, 0.0},
        std::vector<double>{1.0, 1.0, 0.0});

    comotion::MultiRobotProblem problem(comotion::CollisionChecker::Backend::Spheres);
    problem.addRobot(robot, {-0.5, 0.0, 0.0}, {0.5, 0.0, 0.0});
    problem.setResolution(16);

    const auto &instance = problem.robot(0);
    const bool start_valid =
        problem.collisionChecker().isValidSingleFull(*instance.model,
                                                     instance.start);

    std::cout << "CoMotion library smoke: robots=" << problem.numRobots()
              << " start_valid=" << (start_valid ? "true" : "false")
              << "\n";

    return start_valid && problem.numRobots() == 1 ? 0 : 1;
}
