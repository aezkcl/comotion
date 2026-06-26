#include "planar_manipulator_cross_scenarios.hpp"
#include "comotion/collision/CollisionChecker.h"
#include "comotion/robot/RobotModel.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace planar = comotion::benchmark_apps::planar_manipulator_cross;

namespace {

bool near(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

bool expect(bool condition, const std::string &message) {
    if (condition)
        return true;
    std::cerr << "planar_manipulator_cross_geometry: " << message << "\n";
    return false;
}

bool sameConfig(const std::vector<double> &a, const std::vector<double> &b) {
    return a.size() == b.size() && a.size() == 3 && near(a[0], b[0]) &&
           near(a[1], b[1]) && near(a[2], b[2]);
}

bool checkBasePose(const planar::BasePose &pose, double x, double y,
                   double yaw, const std::string &label) {
    bool ok = true;
    ok &= expect(near(pose.x, x), label + " base x mismatch");
    ok &= expect(near(pose.y, y), label + " base y mismatch");
    ok &= expect(near(pose.z, 0.0), label + " base z mismatch");
    ok &= expect(near(pose.yaw, yaw), label + " base yaw mismatch");
    return ok;
}

bool checkSphere(const comotion::ObstacleSphere &sphere, double x, double y,
                 const std::string &label) {
    bool ok = true;
    ok &= expect(near(sphere.center.x(), x), label + " sphere x mismatch");
    ok &= expect(near(sphere.center.y(), y), label + " sphere y mismatch");
    ok &= expect(near(sphere.center.z(), 0.0), label + " sphere z mismatch");
    ok &= expect(near(sphere.radius, planar::kAdaptiveObstacleRadius),
                 label + " sphere radius mismatch");
    return ok;
}

std::string getResourcePath(const std::string &relative) {
    const char *prefixes[] = {"../resources/", "../../resources/",
                              "../../../resources/", "resources/"};
    for (const char *prefix : prefixes) {
        std::string path = std::string(prefix) + relative;
        std::ifstream file(path);
        if (file.good())
            return path;
    }
    return std::string("resources/") + relative;
}

std::shared_ptr<comotion::RobotModel> loadPlanar3(const planar::BasePose &pose) {
    auto robot = std::make_shared<comotion::RobotModel>();
    robot->loadURDF(getResourcePath("planar3/planar3_spherized.urdf"));
    robot->loadSRDF(getResourcePath("planar3/planar3.srdf"));
    robot->setBaseTransform(planar::baseTransform(pose));
    return robot;
}

bool checkCrossGeometry() {
    const auto generated = planar::generateScenario(planar::Scenario::Cross, 8,
                                                    1.35, 1.2);
    bool ok = true;
    ok &= expect(generated.scenario_name == "cross", "scenario name mismatch");
    ok &= expect(generated.num_robots == 8, "robot count mismatch");
    ok &= expect(generated.robots_per_line == 4,
                 "robots per line mismatch");
    ok &= expect(generated.base_poses.size() == 8,
                 "base pose count mismatch");
    ok &= expect(generated.starts.size() == 8, "start count mismatch");
    ok &= expect(generated.goals.size() == 8, "goal count mismatch");
    ok &= expect(generated.cylinder_obstacles.size() == 4,
                 "boundary cylinder count mismatch");
    ok &= expect(generated.sphere_obstacles.empty(),
                 "cross must not generate sphere obstacles");

    const std::vector<double> expected_y{-1.8, -0.6, 0.6, 1.8};
    for (int i = 0; i < 4; ++i) {
        const auto index = static_cast<std::size_t>(i);
        ok &= expect(near(generated.base_poses[index].x, -0.675),
                     "left base x mismatch");
        ok &= expect(near(generated.base_poses[index].y, expected_y[index]),
                     "left base y mismatch");
        ok &= expect(near(generated.base_poses[index].yaw, 0.0),
                     "left base yaw mismatch");
    }
    for (int i = 0; i < 4; ++i) {
        const auto index = static_cast<std::size_t>(i + 4);
        ok &= expect(near(generated.base_poses[index].x, 0.675),
                     "right base x mismatch");
        ok &= expect(near(generated.base_poses[index].y, expected_y[static_cast<std::size_t>(i)]),
                     "right base y mismatch");
        ok &= expect(near(generated.base_poses[index].yaw, planar::kPi),
                     "right base yaw mismatch");
    }

    const std::vector<double> expected_start{planar::kPi / 6.0, 0.0, 0.0};
    const std::vector<double> expected_goal{-planar::kPi / 6.0, 0.0, 0.0};
    for (int i = 0; i < generated.num_robots; ++i) {
        const auto index = static_cast<std::size_t>(i);
        ok &= expect(sameConfig(generated.starts[index], expected_start),
                     "start config mismatch");
        ok &= expect(sameConfig(generated.goals[index], expected_goal),
                     "goal config mismatch");
    }

    ok &= expect(near(generated.side_min_y, -3.0),
                 "side lower boundary mismatch");
    ok &= expect(near(generated.side_max_y, 3.0),
                 "side upper boundary mismatch");
    ok &= expect(near(generated.side_max_y - expected_y.back(), 1.2),
                 "upper side boundary margin mismatch");
    ok &= expect(near(expected_y.front() - generated.side_min_y, 1.2),
                 "lower side boundary margin mismatch");
    ok &= expect(near(generated.left_backboard_x, -0.785),
                 "left backboard x mismatch");
    ok &= expect(near(generated.right_backboard_x, 0.785),
                 "right backboard x mismatch");

    const auto &left_wall = generated.cylinder_obstacles[0];
    const auto &right_wall = generated.cylinder_obstacles[1];
    const auto &bottom_wall = generated.cylinder_obstacles[2];
    const auto &top_wall = generated.cylinder_obstacles[3];
    ok &= expect(near(left_wall.radius, 0.05) &&
                     near(right_wall.radius, 0.05) &&
                     near(bottom_wall.radius, 0.05) &&
                     near(top_wall.radius, 0.05),
                 "boundary wall radius mismatch");
    ok &= expect(near(left_wall.axis.y(), 1.0) &&
                     near(right_wall.axis.y(), 1.0) &&
                     near(bottom_wall.axis.x(), 1.0) &&
                     near(top_wall.axis.x(), 1.0),
                 "boundary wall axis mismatch");
    ok &= expect(near(left_wall.center.x(), -0.785) &&
                     near(right_wall.center.x(), 0.785),
                 "backboard center x mismatch");
    ok &= expect(near(bottom_wall.center.y(), -3.0) &&
                     near(top_wall.center.y(), 3.0),
                 "side wall center y mismatch");
    ok &= expect(near(left_wall.half_height, 3.0) &&
                     near(right_wall.half_height, 3.0),
                 "backboard half height mismatch");
    ok &= expect(near(bottom_wall.half_height, 0.785) &&
                     near(top_wall.half_height, 0.785),
                 "side wall half height mismatch");

    return ok;
}

bool checkAdaptiveEndpointValidity() {
    const auto generated = planar::generateScenario(
        planar::Scenario::Adaptive, 8, 1.35, 1.2);

    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    robots.reserve(generated.base_poses.size());
    for (const auto &pose : generated.base_poses)
        robots.push_back(loadPlanar3(pose));

    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Vamp);
    checker.setObstacles(generated.sphere_obstacles);
    checker.setCylinderObstacles(generated.cylinder_obstacles);

    bool ok = true;
    for (std::size_t i = 0; i < robots.size(); ++i) {
        ok &= expect(checker.isValidSingleFull(*robots[i], generated.starts[i]),
                     generated.robot_names[i] + " start must be valid");
        ok &= expect(checker.isValidSingleFull(*robots[i], generated.goals[i]),
                     generated.robot_names[i] + " goal must be valid");
    }

    return ok;
}

bool checkAdaptiveGeometry() {
    const auto generated = planar::generateScenario(
        planar::Scenario::Adaptive, 8, 1.35, 1.2);
    bool ok = true;
    ok &= expect(planar::parseScenarioName("adaptive") ==
                     planar::Scenario::Adaptive,
                 "adaptive scenario name must parse");
    ok &= expect(generated.scenario_name == "adaptive",
                 "adaptive scenario name mismatch");
    ok &= expect(generated.num_robots == 8, "adaptive robot count mismatch");
    ok &= expect(generated.base_poses.size() == 8,
                 "adaptive base pose count mismatch");
    ok &= expect(generated.starts.size() == 8,
                 "adaptive start count mismatch");
    ok &= expect(generated.goals.size() == 8,
                 "adaptive goal count mismatch");
    ok &= expect(generated.cylinder_obstacles.empty(),
                 "adaptive must not generate cylinder obstacles");
    ok &= expect(generated.sphere_obstacles.size() == 8,
                 "adaptive sphere obstacle count mismatch");
    ok &= expect(!generated.reverse_adaptive_endpoints,
                 "adaptive endpoints must be swapped by default");

    const std::vector<std::string> expected_names{
        "top_inner", "top_outer", "bottom_inner", "bottom_outer",
        "left_inner", "left_outer", "right_inner", "right_outer"};
    ok &= expect(generated.robot_names == expected_names,
                 "adaptive robot names mismatch");

    struct Expected {
        double base_x;
        double base_y;
        double base_yaw;
        std::vector<double> world_start;
        std::vector<double> world_goal;
        double sphere_x;
        double sphere_y;
    };
    const double up_right = planar::kPi / 4.0;
    const double up_left = 3.0 * planar::kPi / 4.0;
    const double down_right = -planar::kPi / 4.0;
    const double down_left = -3.0 * planar::kPi / 4.0;
    const double wrist_fold = 0.0;
    const double inner = 1.2 / std::sqrt(2.0);
    const double outer = inner + 1.2;
    const std::vector<Expected> expected{
        {0.0, inner, 0.0, {up_left, 0.0, 0.0}, {down_left, 0.0, wrist_fold}, -0.5, inner},
        {0.0, outer, planar::kPi, {down_right, 0.0, 0.0}, {up_right, 0.0, 0.0}, 0.5, outer},
        {0.0, -inner, planar::kPi, {down_right, 0.0, 0.0}, {up_right, 0.0, wrist_fold}, 0.5, -inner},
        {0.0, -outer, 0.0, {up_left, 0.0, 0.0}, {down_left, 0.0, 0.0}, -0.5, -outer},
        {-inner, 0.0, planar::kPi / 2.0, {down_left, 0.0, wrist_fold}, {down_right, 0.0, wrist_fold}, -inner, -0.5},
        {-outer, 0.0, -planar::kPi / 2.0, {up_right, 0.0, 0.0}, {up_left, 0.0, 0.0}, -outer, 0.5},
        {inner, 0.0, -planar::kPi / 2.0, {up_right, 0.0, wrist_fold}, {up_left, 0.0, wrist_fold}, inner, 0.5},
        {outer, 0.0, planar::kPi / 2.0, {down_left, 0.0, 0.0}, {down_right, 0.0, 0.0}, outer, -0.5},
    };

    ok &= expect(near(std::sqrt(2.0) * inner, 1.2),
                 "adaptive inner diamond side spacing mismatch");
    ok &= expect(near(outer - inner, 1.2),
                 "adaptive inner-to-outer spacing mismatch");

    for (std::size_t i = 0; i < expected.size(); ++i) {
        const auto &e = expected[i];
        ok &= checkBasePose(generated.base_poses[i], e.base_x, e.base_y,
                            e.base_yaw, expected_names[i]);
        ok &= expect(sameConfig(generated.starts[i],
                                planar::configRelativeToBase(e.world_start,
                                                             e.base_yaw)),
                     expected_names[i] + " start mismatch");
        ok &= expect(sameConfig(generated.goals[i],
                                planar::configRelativeToBase(e.world_goal,
                                                             e.base_yaw)),
                     expected_names[i] + " goal mismatch");
        ok &= expect(sameConfig(
                         planar::configRelativeToBase(generated.starts[i],
                                                      -e.base_yaw),
                         e.world_start),
                     expected_names[i] + " world start mismatch");
        ok &= expect(sameConfig(
                         planar::configRelativeToBase(generated.goals[i],
                                                      -e.base_yaw),
                         e.world_goal),
                     expected_names[i] + " world goal mismatch");
        ok &= checkSphere(generated.sphere_obstacles[i], e.sphere_x,
                          e.sphere_y, expected_names[i]);
    }

    const auto reversed = planar::generateScenario(
        planar::Scenario::Adaptive, 8, 1.35, 1.2, true);
    ok &= expect(reversed.reverse_adaptive_endpoints,
                 "adaptive reverse endpoint flag mismatch");
    ok &= expect(reversed.robot_names == generated.robot_names,
                 "reversed adaptive robot names mismatch");
    for (std::size_t i = 0; i < expected.size(); ++i) {
        ok &= checkBasePose(reversed.base_poses[i], expected[i].base_x,
                            expected[i].base_y, expected[i].base_yaw,
                            expected_names[i] + " reversed");
        ok &= expect(sameConfig(reversed.starts[i], generated.goals[i]),
                     expected_names[i] + " reversed start mismatch");
        ok &= expect(sameConfig(reversed.goals[i], generated.starts[i]),
                     expected_names[i] + " reversed goal mismatch");
        ok &= checkSphere(reversed.sphere_obstacles[i], expected[i].sphere_x,
                          expected[i].sphere_y,
                          expected_names[i] + " reversed");
    }

    return ok;
}

bool checkInvalidInputs() {
    bool ok = true;
    try {
        (void)planar::generateScenario(planar::Scenario::Cross, 5, 1.35, 1.2);
        ok &= expect(false, "odd robot count should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        (void)planar::generateScenario(planar::Scenario::Cross, 2, 1.35, 1.2);
        ok &= expect(false, "robot count below four should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        (void)planar::generateScenario(planar::Scenario::Cross, 4, 0.0, 1.2);
        ok &= expect(false, "zero line distance should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        (void)planar::generateScenario(planar::Scenario::Cross, 4, 1.35, 0.0);
        ok &= expect(false, "zero spacing should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        (void)planar::parseScenarioName("unknown");
        ok &= expect(false, "unknown scenario should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        (void)planar::generateScenario(planar::Scenario::Adaptive, 4, 1.35,
                                       1.2);
        ok &= expect(false, "adaptive non-8 robot count should throw");
    } catch (const std::runtime_error &) {
    }
    try {
        (void)planar::generateScenario(planar::Scenario::Adaptive, 8, 1.35,
                                       0.0);
        ok &= expect(false, "adaptive zero spacing should throw");
    } catch (const std::runtime_error &) {
    }
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok &= checkCrossGeometry();
    ok &= checkAdaptiveGeometry();
    ok &= checkAdaptiveEndpointValidity();
    ok &= checkInvalidInputs();
    if (!ok)
        return 1;
    std::cout << "planar_manipulator_cross_geometry: OK\n";
    return 0;
}
