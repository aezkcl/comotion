#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/FlyingSphere.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace ob = ompl::base;

namespace {

std::shared_ptr<comotion::FlyingSphere> makeSphereRobot(double radius = 1.0) {
    return std::make_shared<comotion::FlyingSphere>(
        radius, std::vector<double>{-12.0, -12.0, 0.0},
        std::vector<double>{12.0, 12.0, 1.5});
}

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr << "composite_rrt_approximate_rejection: " << label
                  << " expected true\n";
        return false;
    }
    return true;
}

std::shared_ptr<comotion::MultiRobotProblem> makeApproximateCompositeProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(128);
    problem->setVmax(2.0);
    problem->setObstacles({comotion::ObstacleSphere{{0.0, 0.0, 0.75}, 1.5}});

    const std::vector<std::vector<double>> starts = {
        {10.0, 0.0, 0.75},
        {7.07, 7.07, 0.75},
        {0.0, 10.0, 0.75},
        {-7.07, 7.07, 0.75},
        {-10.0, 0.0, 0.75},
        {-7.07, -7.07, 0.75},
        {0.0, -10.0, 0.75},
        {7.07, -7.07, 0.75},
    };
    const std::vector<std::vector<double>> goals = {
        {-10.0, 0.0, 0.75},
        {-7.07, -7.07, 0.75},
        {0.0, -10.0, 0.75},
        {7.07, -7.07, 0.75},
        {10.0, 0.0, 0.75},
        {7.07, 7.07, 0.75},
        {0.0, 10.0, 0.75},
        {-7.07, 7.07, 0.75},
    };

    for (size_t i = 0; i < starts.size(); ++i)
        problem->addRobot(makeSphereRobot(), starts[i], goals[i]);

    return problem;
}

bool testCompositeRrtRejectsApproximateSolutions() {
    comotion::seedOmplGlobalFromUserPlanningSeed(6);
    auto problem = makeApproximateCompositeProblem();
    comotion::CompositeRRT planner;
    planner.setPlanningSeed(6);
    planner.setProblem(problem);
    planner.setSimplifySolution(false);

    // Tight enough that the 8-robot composite rarely reaches exact within the cap
    // (OMPL may still succeed on very fast machines if this is too loose).
    const auto status = planner.solve(0.38);

    if (status != ob::PlannerStatus::TIMEOUT) {
        std::cerr << "composite_rrt_approximate_rejection: expected TIMEOUT "
                     "after rejecting approximate solution, got "
                  << status.asString() << "\n";
        return false;
    }

    if (!expectTrue("approximate rejection clears solution paths",
                    planner.getSolutionPaths().empty())) {
        return false;
    }

    return true;
}

} // namespace

int main() {
    if (!testCompositeRrtRejectsApproximateSolutions())
        return 1;

    std::cout << "composite_rrt_approximate_rejection: OK\n";
    return 0;
}
