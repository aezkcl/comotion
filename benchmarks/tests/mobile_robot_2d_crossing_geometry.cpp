#include "mobile_robot_2d_crossing_scenarios.hpp"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/robot/FlyingSphere.h"

#include <ompl/base/ScopedState.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace crossing = comotion::benchmark_apps::mobile_robot_2d;

namespace {

bool near(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

bool expect(bool condition, const std::string &message) {
    if (condition)
        return true;
    std::cerr << "mobile_robot_2d_crossing_geometry: " << message << "\n";
    return false;
}

bool samePoint(const std::vector<double> &a, const std::vector<double> &b) {
    return a.size() == 3 && b.size() == 3 && near(a[0], b[0]) &&
           near(a[1], b[1]) && near(a[2], b[2]);
}

bool expectPoint(const std::vector<double> &point, double x, double y,
                 const std::string &message) {
    return expect(point.size() == 3 && near(point[0], x) &&
                      near(point[1], y) && near(point[2], 0.0),
                  message);
}

std::shared_ptr<comotion::FlyingSphere>
makeSphere(const crossing::GeneratedScenario &generated) {
    return std::make_shared<comotion::FlyingSphere>(
        generated.robot_radius, generated.env_min, generated.env_max);
}

std::vector<comotion::CollisionChecker::Backend> contactTestBackends() {
    return {comotion::CollisionChecker::Backend::Spheres,
            comotion::CollisionChecker::Backend::Fcl,
            comotion::CollisionChecker::Backend::Vamp};
}

std::string backendName(comotion::CollisionChecker::Backend backend) {
    switch (backend) {
    case comotion::CollisionChecker::Backend::Spheres:
        return "sphere";
    case comotion::CollisionChecker::Backend::Fcl:
        return "fcl";
    case comotion::CollisionChecker::Backend::Vamp:
        return "vamp";
    }
    return "unknown";
}

bool checkSingleValidity(const crossing::GeneratedScenario &generated,
                         const std::vector<double> &config,
                         bool expected,
                         const std::string &message) {
    bool ok = true;
    const auto robot = makeSphere(generated);
    for (const auto backend : contactTestBackends()) {
        comotion::CollisionChecker checker(backend);
        checker.setCylinderObstacles(generated.cylinder_obstacles);
        ok &= expect(checker.isValidSingle(*robot, config) == expected,
                     message + " (" + backendName(backend) + ")");
    }
    return ok;
}

bool checkSingleTouchValidity(const crossing::GeneratedScenario &generated,
                              const std::vector<double> &config,
                              const std::string &message) {
    bool ok = true;
    const auto robot = makeSphere(generated);
    for (const auto backend : contactTestBackends()) {
        comotion::CollisionChecker checker(backend);
        checker.setCylinderObstacles(generated.cylinder_obstacles);
        const bool valid = checker.isValidSingle(*robot, config);
        if (backend != comotion::CollisionChecker::Backend::Spheres)
            continue;
        ok &= expect(valid, message + " (" + backendName(backend) + ")");
    }
    return ok;
}

bool checkPairValidity(const crossing::GeneratedScenario &generated,
                       const std::vector<double> &a,
                       const std::vector<double> &b,
                       bool expected,
                       const std::string &message) {
    bool ok = true;
    const auto robot_a = makeSphere(generated);
    const auto robot_b = makeSphere(generated);
    for (const auto backend : contactTestBackends()) {
        comotion::CollisionChecker checker(backend);
        checker.setCylinderObstacles(generated.cylinder_obstacles);
        ok &= expect(checker.isValidPair(*robot_a, a, *robot_b, b) == expected,
                     message + " (" + backendName(backend) + ")");
    }
    return ok;
}

bool checkPairTouchValidity(const crossing::GeneratedScenario &generated,
                            const std::vector<double> &a,
                            const std::vector<double> &b,
                            const std::string &message) {
    bool ok = true;
    const auto robot_a = makeSphere(generated);
    const auto robot_b = makeSphere(generated);
    for (const auto backend : contactTestBackends()) {
        comotion::CollisionChecker checker(backend);
        checker.setCylinderObstacles(generated.cylinder_obstacles);
        const bool valid = checker.isValidPair(*robot_a, a, *robot_b, b);
        if (backend != comotion::CollisionChecker::Backend::Spheres)
            continue;
        ok &= expect(valid, message + " (" + backendName(backend) + ")");
    }
    return ok;
}

bool checkAllEndpointsValid(const crossing::GeneratedScenario &generated) {
    bool ok = true;
    for (std::size_t i = 0; i < generated.starts.size(); ++i) {
        ok &= checkSingleValidity(
            generated, generated.starts[i], true,
            "start " + std::to_string(i) + " must not touch an obstacle");
    }
    for (std::size_t i = 0; i < generated.goals.size(); ++i) {
        ok &= checkSingleValidity(
            generated, generated.goals[i], true,
            "goal " + std::to_string(i) + " must not touch an obstacle");
    }
    return ok;
}

bool checkBoundaryCylinders(const crossing::GeneratedScenario &generated) {
    bool ok = true;
    ok &= expect(generated.cylinder_obstacles.size() >= 4,
                 "scenario must include visual boundary cylinders");
    if (generated.cylinder_obstacles.size() < 4)
        return ok;

    const double radius = crossing::boundaryCylinderRadius(
        generated.robot_radius);
    const double offset = crossing::boundaryCylinderOffset(
        generated.robot_radius);
    const double min_x = generated.env_min[0];
    const double max_x = generated.env_max[0];
    const double min_y = generated.env_min[1];
    const double max_y = generated.env_max[1];
    const double center_x = 0.5 * (min_x + max_x);
    const double center_y = 0.5 * (min_y + max_y);
    const double half_width = 0.5 * (max_x - min_x) + offset;
    const double half_height = 0.5 * (max_y - min_y) + offset;
    const std::size_t first = generated.cylinder_obstacles.size() - 4;

    const auto &left = generated.cylinder_obstacles[first];
    const auto &right = generated.cylinder_obstacles[first + 1];
    const auto &bottom = generated.cylinder_obstacles[first + 2];
    const auto &top = generated.cylinder_obstacles[first + 3];
    ok &= expect(near(left.center.x(), min_x - offset) &&
                     near(left.center.y(), center_y) &&
                     near(right.center.x(), max_x + offset) &&
                     near(right.center.y(), center_y),
                 "vertical boundary cylinder positions mismatch");
    ok &= expect(near(bottom.center.x(), center_x) &&
                     near(bottom.center.y(), min_y - offset) &&
                     near(top.center.x(), center_x) &&
                     near(top.center.y(), max_y + offset),
                 "horizontal boundary cylinder positions mismatch");
    ok &= expect(near(left.center.x() + left.radius, min_x) &&
                     near(right.center.x() - right.radius, max_x) &&
                     near(bottom.center.y() + bottom.radius, min_y) &&
                     near(top.center.y() - top.radius, max_y),
                 "boundary cylinder inner edges must touch environment bounds");
    ok &= expect(near(left.axis.y(), 1.0) && near(right.axis.y(), 1.0) &&
                     near(bottom.axis.x(), 1.0) && near(top.axis.x(), 1.0),
                 "boundary cylinder axes mismatch");
    ok &= expect(near(left.radius, radius) && near(right.radius, radius) &&
                     near(bottom.radius, radius) && near(top.radius, radius),
                 "boundary cylinder radius mismatch");
    ok &= expect(near(left.half_height, half_height) &&
                     near(right.half_height, half_height) &&
                     near(bottom.half_height, half_width) &&
                     near(top.half_height, half_width),
                 "boundary cylinder half-height mismatch");
    return ok;
}

bool checkCommon(const crossing::GeneratedScenario &generated,
                 int expected_robots, bool expect_square_bounds = true) {
    bool ok = true;
    ok &= expect(generated.num_robots == expected_robots,
                 "robot count mismatch");
    ok &= expect(generated.starts.size() ==
                     static_cast<std::size_t>(expected_robots),
                 "start count mismatch");
    ok &= expect(generated.goals.size() ==
                     static_cast<std::size_t>(expected_robots),
                 "goal count mismatch");
    ok &= expect(generated.env_min.size() == 3 && generated.env_max.size() == 3,
                 "environment bounds must be 3D");
    ok &= expect(near(generated.env_min[2], 0.0) &&
                     near(generated.env_max[2], 0.0),
                 "z bounds must be fixed at zero");
    if (expect_square_bounds) {
        ok &= expect(near(generated.env_max[0] - generated.env_min[0],
                          generated.env_max[1] - generated.env_min[1]),
                     "environment bounds must be square in x/y");
    }
    for (const auto &p : generated.starts)
        ok &= expect(p.size() == 3 && near(p[2], 0.0),
                     "start must be 3D and z=0");
    for (const auto &p : generated.goals)
        ok &= expect(p.size() == 3 && near(p[2], 0.0),
                     "goal must be 3D and z=0");
    ok &= checkBoundaryCylinders(generated);
    ok &= checkAllEndpointsValid(generated);
    return ok;
}

bool checkCircle() {
    const auto generated = crossing::generateScenario(
        crossing::CrossingScenario::Circle, 8, 1.0, 5.0);
    bool ok = checkCommon(generated, 8);
    for (int i = 0; i < 8; ++i) {
        ok &= expect(samePoint(generated.goals[static_cast<std::size_t>(i)],
                               generated.starts[static_cast<std::size_t>(
                                   (i + 4) % 8)]),
                     "circle goals must be opposite starts");
    }
    return ok;
}

bool checkParallel() {
    const auto generated = crossing::generateScenario(
        crossing::CrossingScenario::Parallel, 6, 1.0, 5.0);
    bool ok = checkCommon(generated, 6);
    for (int i = 0; i < 6; i += 2) {
        const auto &a_start = generated.starts[static_cast<std::size_t>(i)];
        const auto &a_goal = generated.goals[static_cast<std::size_t>(i)];
        const auto &b_start = generated.starts[static_cast<std::size_t>(i + 1)];
        const auto &b_goal = generated.goals[static_cast<std::size_t>(i + 1)];
        ok &= expect(near(a_start[1], a_goal[1]) &&
                         near(b_start[1], b_goal[1]),
                     "parallel swaps must preserve y");
        ok &= expect(near(a_start[1], b_start[1]) &&
                         near(a_goal[1], b_goal[1]),
                     "parallel pair must share one line");
        ok &= expect(near(a_start[0], b_goal[0]) &&
                         near(a_goal[0], b_start[0]),
                     "parallel pair must swap x only");
    }
    return ok;
}

bool checkParallelDirectCompositeMotionRejected() {
    const auto generated = crossing::generateScenario(
        crossing::CrossingScenario::Parallel, 16, 0.5, 5.0);
    bool ok = true;
    for (const auto backend : contactTestBackends()) {
        comotion::MultiRobotProblem problem(backend);
        for (int i = 0; i < generated.num_robots; ++i) {
            problem.addRobot(
                makeSphere(generated),
                generated.starts[static_cast<std::size_t>(i)],
                generated.goals[static_cast<std::size_t>(i)]);
        }
        problem.setCylinderObstacles(generated.cylinder_obstacles);
        problem.setResolution(128);

        std::vector<int> robot_indices;
        for (int i = 0; i < generated.num_robots; ++i)
            robot_indices.push_back(i);
        auto si = problem.createMakespanCompositeSpaceInfo(robot_indices);
        auto space = si->getStateSpace();
        ompl::base::ScopedState<> start(space);
        ompl::base::ScopedState<> goal(space);
        auto *start_values =
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        auto *goal_values =
            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        int offset = 0;
        for (int i = 0; i < generated.num_robots; ++i) {
            const auto &s = generated.starts[static_cast<std::size_t>(i)];
            const auto &g = generated.goals[static_cast<std::size_t>(i)];
            for (std::size_t d = 0; d < s.size(); ++d) {
                start_values[offset + static_cast<int>(d)] = s[d];
                goal_values[offset + static_cast<int>(d)] = g[d];
            }
            offset += static_cast<int>(s.size());
        }

        ok &= expect(!si->checkMotion(start.get(), goal.get()),
                     "parallel n=16 direct composite swap must collide (" +
                         backendName(backend) + ")");
    }
    return ok;
}

bool checkPerpendicular() {
    const auto generated = crossing::generateScenario(
        crossing::CrossingScenario::Perpendicular, 6, 1.0, 5.0);
    bool ok = checkCommon(generated, 6);
    for (int i = 0; i < 6; i += 2) {
        const auto &horizontal_start =
            generated.starts[static_cast<std::size_t>(i)];
        const auto &horizontal_goal =
            generated.goals[static_cast<std::size_t>(i)];
        const auto &vertical_start =
            generated.starts[static_cast<std::size_t>(i + 1)];
        const auto &vertical_goal =
            generated.goals[static_cast<std::size_t>(i + 1)];

        ok &= expect(near(horizontal_start[1], horizontal_goal[1]),
                     "horizontal perpendicular robot must preserve y");
        ok &= expect(near(vertical_start[0], vertical_goal[0]),
                     "vertical perpendicular robot must preserve x");
        ok &= expect(near(vertical_start[0], horizontal_start[1]) &&
                         near(vertical_goal[0], horizontal_goal[1]),
                     "perpendicular pair must be mirrored across x=y");
    }
    return ok;
}

bool checkHallwaysDefault() {
    const auto generated = crossing::generateScenario(
        crossing::CrossingScenario::Hallways, 8, 0.5, 5.0);
    bool ok = checkCommon(generated, 8, false);
    ok &= expect(crossing::parseScenarioName("hallways") ==
                     crossing::CrossingScenario::Hallways,
                 "hallways scenario name must parse");
    ok &= expect(generated.scenario_name == "hallways",
                 "hallways scenario name mismatch");
    ok &= expect(generated.vertical_hallways == 3,
                 "default vertical hallway count mismatch");
    ok &= expect(generated.horizontal_hallways == 0,
                 "default horizontal hallway count mismatch");
    ok &= expect(near(generated.hallway_radius, 1.0),
                 "default hallway radius mismatch");
    ok &= expect(generated.cylinder_obstacles.size() == 8,
                 "default hallway cylinder count mismatch");
    ok &= expect(near(generated.env_min[0], -11.0) &&
                     near(generated.env_min[1], -21.0) &&
                     near(generated.env_max[0], 11.0) &&
                     near(generated.env_max[1], 21.0),
                 "default hallways bounds mismatch");

    const int pairs = 4;
    const double top_y = 20.0;
    for (int i = 0; i < pairs; ++i) {
        const double expected_x = (static_cast<double>(i) - 1.5) * 5.0;
        const auto &top_start = generated.starts[static_cast<std::size_t>(i)];
        const auto &top_goal = generated.goals[static_cast<std::size_t>(i)];
        const auto &bottom_start =
            generated.starts[static_cast<std::size_t>(i + pairs)];
        const auto &bottom_goal =
            generated.goals[static_cast<std::size_t>(i + pairs)];

        ok &= expect(near(top_start[0], expected_x) &&
                         near(bottom_start[0], expected_x),
                     "hallways robot columns must be evenly spaced");
        ok &= expect(near(top_start[1], top_y) &&
                         near(top_goal[1], -top_y) &&
                         near(bottom_start[1], -top_y) &&
                         near(bottom_goal[1], top_y),
                     "hallways robots must swap vertically");
        ok &= expect(near(top_start[0], top_goal[0]) &&
                         near(bottom_start[0], bottom_goal[0]),
                     "hallways vertical swaps must preserve x");
    }

    if (generated.cylinder_obstacles.size() >= 4) {
        for (int i = 0; i < 4; ++i) {
            const double expected_x = (static_cast<double>(i) - 1.5) * 5.0;
            ok &= expect(near(generated.cylinder_obstacles[static_cast<std::size_t>(i)]
                                  .center.x(),
                              expected_x),
                         "default hallway cylinders must align with robot columns");
        }
    }
    for (std::size_t i = 0; i < std::min<std::size_t>(
                                generated.cylinder_obstacles.size(), 4);
         ++i) {
        const auto &cylinder = generated.cylinder_obstacles[i];
        ok &= expect(near(cylinder.center.z(), 0.0),
                     "rack cylinder center z must be zero");
        ok &= expect(near(cylinder.axis.x(), 0.0) &&
                         near(cylinder.axis.y(), 1.0) &&
                         near(cylinder.axis.z(), 0.0),
                     "rack cylinders must be y-axis aligned");
        ok &= expect(near(cylinder.radius, 1.5),
                     "rack cylinder radius mismatch");
        ok &= expect(near(cylinder.half_height, 19.0),
                     "rack cylinder half height mismatch");
    }
    return ok;
}

bool checkHallwaysHorizontalSplit() {
    crossing::HallwayOptions options;
    options.vertical_hallways = 4;
    options.horizontal_hallways = 2;
    options.hallway_radius = 1.0;
    const auto generated = crossing::generateScenario(
        crossing::CrossingScenario::Hallways, 8, 0.5, 5.0, options);
    bool ok = checkCommon(generated, 8, false);
    ok &= expect(generated.vertical_hallways == 4,
                 "explicit vertical hallway count mismatch");
    ok &= expect(generated.horizontal_hallways == 2,
                 "explicit horizontal hallway count mismatch");
    ok &= expect(near(generated.hallway_radius, 1.0),
                 "explicit hallway radius mismatch");
    ok &= expect(generated.cylinder_obstacles.size() == 19,
                 "horizontal hallway split cylinder count mismatch");

    if (generated.cylinder_obstacles.size() >= 15) {
        for (std::size_t i = 0; i < 15; i += 3) {
            ok &= expect(near(generated.cylinder_obstacles[i].center.x(),
                              generated.cylinder_obstacles[i + 1].center.x()) &&
                             near(generated.cylinder_obstacles[i].center.x(),
                                  generated.cylinder_obstacles[i + 2].center.x()),
                         "split rack segments in a column must share x");
        }
    }
    return ok;
}

bool checkAdaptiveDefaults() {
    const auto generated = crossing::generateScenario(
        crossing::CrossingScenario::Adaptive, 16, 0.5, 5.0);
    bool ok = checkCommon(generated, 16);
    ok &= expect(crossing::parseScenarioName("adaptive") ==
                     crossing::CrossingScenario::Adaptive,
                 "adaptive scenario name must parse");
    ok &= expect(generated.scenario_name == "adaptive",
                 "adaptive scenario name mismatch");
    ok &= expect(near(generated.hallway_width, 2.0),
                 "adaptive default hallway width mismatch");
    ok &= expect(near(generated.intersection_width, 4.0),
                 "adaptive default intersection width mismatch");
    ok &= expect(generated.cylinder_obstacles.size() == 12,
                 "adaptive cylinder count mismatch");
    ok &= expect(near(generated.env_min[0], -9.0) &&
                     near(generated.env_min[1], -9.0) &&
                     near(generated.env_max[0], 9.0) &&
                     near(generated.env_max[1], 9.0),
                 "adaptive bounds mismatch");

    ok &= expectPoint(generated.starts[0], 0.0, 3.0,
                      "adaptive inner north start mismatch");
    ok &= expectPoint(generated.goals[0], 0.0, -6.0,
                      "adaptive inner north goal mismatch");
    ok &= expectPoint(generated.starts[1], 3.0, 0.0,
                      "adaptive inner east start mismatch");
    ok &= expectPoint(generated.goals[1], -6.0, 0.0,
                      "adaptive inner east goal mismatch");
    ok &= expectPoint(generated.starts[2], 0.0, -3.0,
                      "adaptive inner south start mismatch");
    ok &= expectPoint(generated.goals[2], 0.0, 6.0,
                      "adaptive inner south goal mismatch");
    ok &= expectPoint(generated.starts[3], -3.0, 0.0,
                      "adaptive inner west start mismatch");
    ok &= expectPoint(generated.goals[3], 6.0, 0.0,
                      "adaptive inner west goal mismatch");

    ok &= expectPoint(generated.starts[4], 0.0, 6.0,
                      "adaptive exterior north start mismatch");
    ok &= expectPoint(generated.goals[4], 0.0, -3.0,
                      "adaptive exterior north goal mismatch");
    ok &= expectPoint(generated.starts[5], 6.0, 0.0,
                      "adaptive exterior east start mismatch");
    ok &= expectPoint(generated.goals[5], -3.0, 0.0,
                      "adaptive exterior east goal mismatch");
    ok &= expectPoint(generated.starts[6], 0.0, -6.0,
                      "adaptive exterior south start mismatch");
    ok &= expectPoint(generated.goals[6], 0.0, 3.0,
                      "adaptive exterior south goal mismatch");
    ok &= expectPoint(generated.starts[7], -6.0, 0.0,
                      "adaptive exterior west start mismatch");
    ok &= expectPoint(generated.goals[7], 3.0, 0.0,
                      "adaptive exterior west goal mismatch");

    const double outer =
        0.5 * generated.intersection_width + generated.spacing;
    const double outer_hallway_mid =
        outer + 0.5 * generated.hallway_width;
    const double endpoint_distance =
        crossing::endpointFaceDistance(generated.robot_radius);
    const double obstacle_outer_aligned_axis = outer - endpoint_distance;

    ok &= expectPoint(generated.starts[8], -obstacle_outer_aligned_axis,
                      outer_hallway_mid,
                      "adaptive top-left border start mismatch");
    ok &= expectPoint(generated.goals[8], obstacle_outer_aligned_axis,
                      outer_hallway_mid,
                      "adaptive top-left border goal mismatch");
    ok &= expectPoint(generated.starts[9], obstacle_outer_aligned_axis,
                      outer_hallway_mid,
                      "adaptive top-right border start mismatch");
    ok &= expectPoint(generated.goals[9], -obstacle_outer_aligned_axis,
                      outer_hallway_mid,
                      "adaptive top-right border goal mismatch");
    ok &= expectPoint(generated.starts[10], outer_hallway_mid,
                      obstacle_outer_aligned_axis,
                      "adaptive right-top border start mismatch");
    ok &= expectPoint(generated.goals[10], outer_hallway_mid,
                      -obstacle_outer_aligned_axis,
                      "adaptive right-top border goal mismatch");
    ok &= expectPoint(generated.starts[11], outer_hallway_mid,
                      -obstacle_outer_aligned_axis,
                      "adaptive right-bottom border start mismatch");
    ok &= expectPoint(generated.goals[11], outer_hallway_mid,
                      obstacle_outer_aligned_axis,
                      "adaptive right-bottom border goal mismatch");
    ok &= expectPoint(generated.starts[12], obstacle_outer_aligned_axis,
                      -outer_hallway_mid,
                      "adaptive bottom-right border start mismatch");
    ok &= expectPoint(generated.goals[12], -obstacle_outer_aligned_axis,
                      -outer_hallway_mid,
                      "adaptive bottom-right border goal mismatch");
    ok &= expectPoint(generated.starts[13], -obstacle_outer_aligned_axis,
                      -outer_hallway_mid,
                      "adaptive bottom-left border start mismatch");
    ok &= expectPoint(generated.goals[13], obstacle_outer_aligned_axis,
                      -outer_hallway_mid,
                      "adaptive bottom-left border goal mismatch");
    ok &= expectPoint(generated.starts[14], -outer_hallway_mid,
                      -obstacle_outer_aligned_axis,
                      "adaptive left-bottom border start mismatch");
    ok &= expectPoint(generated.goals[14], -outer_hallway_mid,
                      obstacle_outer_aligned_axis,
                      "adaptive left-bottom border goal mismatch");
    ok &= expectPoint(generated.starts[15], -outer_hallway_mid,
                      obstacle_outer_aligned_axis,
                      "adaptive left-top border start mismatch");
    ok &= expectPoint(generated.goals[15], -outer_hallway_mid,
                      -obstacle_outer_aligned_axis,
                      "adaptive left-top border goal mismatch");

    const auto &first_y_leg = generated.cylinder_obstacles[0];
    ok &= expect(near(first_y_leg.center.x(), -4.5) &&
                     near(first_y_leg.center.y(), -4.0),
                 "adaptive first y-leg center mismatch");
    ok &= expect(near(first_y_leg.axis.y(), 1.0) &&
                     near(first_y_leg.radius, 2.5) &&
                     near(first_y_leg.half_height, 3.0),
                 "adaptive first y-leg dimensions mismatch");
    const auto &first_x_leg = generated.cylinder_obstacles[1];
    ok &= expect(near(first_x_leg.center.x(), -4.0) &&
                     near(first_x_leg.center.y(), -4.5),
                 "adaptive first x-leg center mismatch");
    ok &= expect(near(first_x_leg.axis.x(), 1.0) &&
                     near(first_x_leg.radius, 2.5) &&
                     near(first_x_leg.half_height, 3.0),
                 "adaptive first x-leg dimensions mismatch");
    return ok;
}

bool checkAdaptiveRobotCounts() {
    bool ok = true;
    const auto four = crossing::generateScenario(
        crossing::CrossingScenario::Adaptive, 4, 0.5, 5.0);
    ok &= checkCommon(four, 4);
    ok &= expect(four.cylinder_obstacles.size() == 12,
                 "adaptive 4 robot cylinder count mismatch");
    ok &= expectPoint(four.starts[0], 0.0, 3.0,
                      "adaptive 4 robot first start mismatch");
    ok &= expectPoint(four.goals[3], 6.0, 0.0,
                      "adaptive 4 robot last goal mismatch");

    const auto eight = crossing::generateScenario(
        crossing::CrossingScenario::Adaptive, 8, 0.5, 5.0);
    ok &= checkCommon(eight, 8);
    ok &= expect(eight.cylinder_obstacles.size() == 12,
                 "adaptive 8 robot cylinder count mismatch");
    ok &= expectPoint(eight.starts[7], -6.0, 0.0,
                      "adaptive 8 robot last start mismatch");
    ok &= expectPoint(eight.goals[7], 3.0, 0.0,
                      "adaptive 8 robot last goal mismatch");
    return ok;
}

bool checkObstacleCorridorContactClearance() {
    bool ok = true;
    constexpr double robot_radius = 0.25;
    constexpr double width = 1.0;
    constexpr double eps = 1e-3;

    crossing::HallwayOptions hallway_options;
    hallway_options.vertical_hallways = 1;
    hallway_options.hallway_radius = 0.5 * width;
    const auto hallways = crossing::generateScenario(
        crossing::CrossingScenario::Hallways, 4, robot_radius, 2.0,
        hallway_options);
    ok &= checkSingleTouchValidity(
        hallways, {-0.25, 0.0, 0.0},
        "hallways left contact center must be valid for touch-permissive backends");
    ok &= checkSingleTouchValidity(
        hallways, {0.25, 0.0, 0.0},
        "hallways right contact center must be valid for touch-permissive backends");
    ok &= checkSingleValidity(hallways, {-0.25 + eps, 0.0, 0.0}, true,
                              "hallways just inside left contact must be valid");
    ok &= checkSingleValidity(hallways, {0.25 - eps, 0.0, 0.0}, true,
                              "hallways just inside right contact must be valid");
    ok &= checkSingleValidity(hallways, {-0.25 - eps, 0.0, 0.0}, false,
                              "hallways beyond left contact must collide");
    ok &= checkSingleValidity(hallways, {0.25 + eps, 0.0, 0.0}, false,
                              "hallways beyond right contact must collide");
    ok &= checkPairTouchValidity(
        hallways, {-0.25, 0.0, 0.0}, {0.25, 0.0, 0.0},
        "two hallway robots touching must be valid for touch-permissive backends");
    ok &= checkPairValidity(hallways, {-0.25 + eps, 0.0, 0.0},
                            {0.25, 0.0, 0.0}, false,
                            "overlapping hallway robots must collide");

    crossing::AdaptiveOptions adaptive_options;
    adaptive_options.hallway_width = width;
    adaptive_options.intersection_width = 2.0 * width;
    const auto adaptive = crossing::generateScenario(
        crossing::CrossingScenario::Adaptive, 4, robot_radius, 2.0, {},
        adaptive_options);
    ok &= checkSingleTouchValidity(
        adaptive, {1.0, 3.25, 0.0},
        "adaptive obstacle contact center must be valid for touch-permissive backends");
    ok &= checkSingleTouchValidity(
        adaptive, {1.0, 3.75, 0.0},
        "adaptive boundary contact center must be valid for touch-permissive backends");
    ok &= checkSingleValidity(adaptive, {1.0, 3.25 + eps, 0.0}, true,
                              "adaptive just inside obstacle contact must be valid");
    ok &= checkSingleValidity(adaptive, {1.0, 3.75 - eps, 0.0}, true,
                              "adaptive just inside boundary contact must be valid");
    ok &= checkSingleValidity(adaptive, {1.0, 3.25 - eps, 0.0}, false,
                              "adaptive beyond obstacle contact must collide");
    ok &= checkSingleValidity(adaptive, {1.0, 3.75 + eps, 0.0}, false,
                              "adaptive beyond boundary contact must collide");

    crossing::InletOptions inlet_options;
    inlet_options.hallway_width = width;
    inlet_options.hallway_length = 4.0;
    const auto inlet = crossing::generateScenario(
        crossing::CrossingScenario::Inlet, 2, robot_radius, 1.0, {}, {},
        inlet_options);
    ok &= checkSingleTouchValidity(
        inlet, {-1.0, 0.25, 0.0},
        "inlet boundary contact center must be valid for touch-permissive backends");
    ok &= checkSingleTouchValidity(
        inlet, {-1.0, 0.75, 0.0},
        "inlet obstacle contact center must be valid for touch-permissive backends");
    ok &= checkSingleValidity(inlet, {-1.0, 0.25 + eps, 0.0}, true,
                              "inlet just inside boundary contact must be valid");
    ok &= checkSingleValidity(inlet, {-1.0, 0.75 - eps, 0.0}, true,
                              "inlet just inside obstacle contact must be valid");
    ok &= checkSingleValidity(inlet, {-1.0, 0.25 - eps, 0.0}, false,
                              "inlet beyond boundary contact must collide");
    ok &= checkSingleValidity(inlet, {-1.0, 0.75 + eps, 0.0}, false,
                              "inlet beyond obstacle contact must collide");
    ok &= checkPairTouchValidity(
        inlet, {-1.0, 0.25, 0.0}, {-1.0, 0.75, 0.0},
        "two inlet robots touching must be valid for touch-permissive backends");
    ok &= checkPairValidity(inlet, {-1.0, 0.25 + eps, 0.0},
                            {-1.0, 0.75, 0.0}, false,
                            "overlapping inlet robots must collide");

    return ok;
}

bool checkInletDefault() {
    const auto generated = crossing::generateScenario(
        crossing::CrossingScenario::Inlet, 2, 0.5, 5.0);
    bool ok = checkCommon(generated, 2, false);
    ok &= expect(crossing::parseScenarioName("inlet") ==
                     crossing::CrossingScenario::Inlet,
                 "inlet scenario name must parse");
    ok &= expect(generated.scenario_name == "inlet",
                 "inlet scenario name mismatch");
    ok &= expect(near(generated.hallway_width, 2.0),
                 "inlet default hallway width mismatch");
    ok &= expect(near(generated.hallway_length, 20.0),
                 "inlet default hallway length mismatch");
    ok &= expect(generated.cylinder_obstacles.size() == 6,
                 "inlet cylinder count mismatch");
    ok &= expect(near(generated.env_min[0], -10.0) &&
                     near(generated.env_min[1], 0.0) &&
                     near(generated.env_max[0], 10.0) &&
                     near(generated.env_max[1], 4.0),
                 "inlet bounds mismatch");

    ok &= expectPoint(generated.starts[0], -9.0, 1.0,
                      "inlet left start mismatch");
    ok &= expectPoint(generated.goals[0], 9.0, 1.0,
                      "inlet left goal mismatch");
    ok &= expectPoint(generated.starts[1], 9.0, 1.0,
                      "inlet right start mismatch");
    ok &= expectPoint(generated.goals[1], -9.0, 1.0,
                      "inlet right goal mismatch");

    if (generated.cylinder_obstacles.size() >= 2) {
        const auto &left = generated.cylinder_obstacles[0];
        const auto &right = generated.cylinder_obstacles[1];
        ok &= expect(near(left.center.x(), -5.5) &&
                         near(right.center.x(), 5.5) &&
                         near(left.center.y(), 3.0) &&
                         near(right.center.y(), 3.0),
                     "inlet obstacle centers mismatch");
        ok &= expect(near(left.axis.x(), 1.0) &&
                         near(right.axis.x(), 1.0) &&
                         near(left.radius, 1.0) &&
                         near(right.radius, 1.0) &&
                         near(left.half_height, 4.5) &&
                         near(right.half_height, 4.5),
                     "inlet obstacle dimensions mismatch");
        const double left_inner_edge =
            left.center.x() + left.half_height;
        const double right_inner_edge =
            right.center.x() - right.half_height;
        ok &= expect(near(right_inner_edge - left_inner_edge,
                          generated.hallway_width),
                     "inlet center gap must equal hallway width");
        ok &= expect(near(left.center.y() - left.radius -
                              generated.env_min[1],
                          generated.hallway_width),
                     "inlet hallway must match inlet gap width");
        const double boundary_radius =
            crossing::boundaryCylinderRadius(generated.robot_radius);
        const auto &left_boundary =
            generated.cylinder_obstacles[generated.cylinder_obstacles.size() -
                                         4];
        const auto &right_boundary =
            generated.cylinder_obstacles[generated.cylinder_obstacles.size() -
                                         3];
        const auto &top_boundary =
            generated.cylinder_obstacles[generated.cylinder_obstacles.size() -
                                         1];
        ok &= expect(near(left.center.x() - left.half_height,
                          left_boundary.center.x() + boundary_radius) &&
                         near(right.center.x() + right.half_height,
                              right_boundary.center.x() - boundary_radius) &&
                         near(left.center.y() + left.radius,
                              top_boundary.center.y() - boundary_radius) &&
                         near(right.center.y() + right.radius,
                              top_boundary.center.y() - boundary_radius),
                     "inlet obstacles must touch visual boundary cylinders");
    }
    return ok;
}

bool checkInletCustomSize() {
    crossing::InletOptions options;
    options.hallway_width = 3.0;
    options.hallway_length = 15.0;
    const auto generated = crossing::generateScenario(
        crossing::CrossingScenario::Inlet, 2, 0.5, 5.0, {}, {}, options);
    bool ok = checkCommon(generated, 2, false);
    ok &= expect(near(generated.hallway_width, 3.0),
                 "inlet explicit hallway width mismatch");
    ok &= expect(near(generated.hallway_length, 15.0),
                 "inlet explicit hallway length mismatch");
    ok &= expectPoint(generated.starts[0], -6.5, 1.5,
                      "inlet custom left start mismatch");
    ok &= expectPoint(generated.goals[0], 6.5, 1.5,
                      "inlet custom left goal mismatch");
    if (generated.cylinder_obstacles.size() >= 2) {
        const auto &left = generated.cylinder_obstacles[0];
        const auto &right = generated.cylinder_obstacles[1];
        ok &= expect(near(left.center.x(), -4.5) &&
                         near(right.center.x(), 4.5) &&
                         near(left.center.y(), 4.5) &&
                         near(right.center.y(), 4.5),
                     "inlet custom obstacle centers mismatch");
        ok &= expect(near(left.radius, 1.5) &&
                         near(right.radius, 1.5) &&
                         near(left.half_height, 3.0) &&
                         near(right.half_height, 3.0),
                     "inlet custom obstacle dimensions mismatch");
    }
    return ok;
}

bool checkInvalidInputs() {
    bool ok = true;
    try {
        (void)crossing::generateScenario(crossing::CrossingScenario::Circle,
                                         5, 1.0, 5.0);
        ok &= expect(false, "odd robot count should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        (void)crossing::generateScenario(crossing::CrossingScenario::Parallel,
                                         2, 1.0, 5.0);
        ok &= expect(false, "robot count below four should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        crossing::HallwayOptions options;
        options.vertical_hallways = 0;
        (void)crossing::generateScenario(
            crossing::CrossingScenario::Hallways, 8, 0.5, 5.0, options);
        ok &= expect(false, "zero vertical hallways should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        crossing::HallwayOptions options;
        options.horizontal_hallways = -1;
        (void)crossing::generateScenario(
            crossing::CrossingScenario::Hallways, 8, 0.5, 5.0, options);
        ok &= expect(false, "negative horizontal hallways should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        crossing::HallwayOptions options;
        options.hallway_radius = 0.5;
        (void)crossing::generateScenario(
            crossing::CrossingScenario::Hallways, 8, 0.5, 5.0, options);
        ok &= expect(false, "hallway radius at robot radius should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        crossing::HallwayOptions options;
        options.hallway_radius = 1.0;
        (void)crossing::generateScenario(
            crossing::CrossingScenario::Hallways, 8, 0.5, 2.0, options);
        ok &= expect(false, "spacing at twice hallway radius should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        (void)crossing::generateScenario(crossing::CrossingScenario::Adaptive,
                                         12, 0.5, 5.0);
        ok &= expect(false, "adaptive non-4/8/16 robot count should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        crossing::AdaptiveOptions options;
        options.hallway_width = 1.0;
        (void)crossing::generateScenario(
            crossing::CrossingScenario::Adaptive, 4, 0.5, 5.0, {}, options);
        ok &= expect(false, "adaptive hallway width at diameter should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        crossing::AdaptiveOptions options;
        options.hallway_width = 2.0;
        options.intersection_width = 2.0;
        (void)crossing::generateScenario(
            crossing::CrossingScenario::Adaptive, 4, 0.5, 5.0, {}, options);
        ok &= expect(false, "adaptive non-expanded intersection should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        (void)crossing::generateScenario(crossing::CrossingScenario::Adaptive,
                                         8, 0.5, 2.0);
        ok &= expect(false, "adaptive 8 robot tight spacing should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        (void)crossing::generateScenario(crossing::CrossingScenario::Adaptive,
                                         4, 0.5, 2.0);
        ok &= expect(false,
                     "adaptive 4 robot endpoint-contact spacing should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        (void)crossing::generateScenario(crossing::CrossingScenario::Inlet,
                                         4, 0.5, 5.0);
        ok &= expect(false, "inlet non-2 robot count should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        crossing::InletOptions options;
        options.hallway_width = 1.0;
        (void)crossing::generateScenario(
            crossing::CrossingScenario::Inlet, 2, 0.5, 5.0, {}, {}, options);
        ok &= expect(false, "inlet hallway width at diameter should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        crossing::InletOptions options;
        options.hallway_width = 3.0;
        options.hallway_length = 3.0;
        (void)crossing::generateScenario(
            crossing::CrossingScenario::Inlet, 2, 0.5, 5.0, {}, {}, options);
        ok &= expect(false, "inlet non-expanded length should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        crossing::InletOptions options;
        options.hallway_width = 1.5;
        options.hallway_length = 2.0;
        (void)crossing::generateScenario(
            crossing::CrossingScenario::Inlet, 2, 0.5, 5.0, {}, {}, options);
        ok &= expect(false, "inlet endpoint-contact length should throw");
    } catch (const std::runtime_error &) {
    }
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok &= checkCircle();
    ok &= checkParallel();
    ok &= checkParallelDirectCompositeMotionRejected();
    ok &= checkPerpendicular();
    ok &= checkHallwaysDefault();
    ok &= checkHallwaysHorizontalSplit();
    ok &= checkAdaptiveDefaults();
    ok &= checkAdaptiveRobotCounts();
    ok &= checkObstacleCorridorContactClearance();
    ok &= checkInletDefault();
    ok &= checkInletCustomSize();
    ok &= checkInvalidInputs();
    if (!ok)
        return 1;
    std::cout << "mobile_robot_2d_crossing_geometry: OK\n";
    return 0;
}
