#pragma once

#include "comotion/collision/ObstacleShapes.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace comotion::benchmark_apps::planar_manipulator_cross {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kStartAngle = kPi / 6.0;
constexpr double kGoalAngle = -kPi / 6.0;
constexpr double kWallRadius = 0.05;
constexpr double kBoundaryMargin = 1.2;
constexpr double kBackboardOffset = 0.11;
constexpr double kAdaptiveObstacleRadius = 0.05;
constexpr double kAdaptiveObstacleOffset = 0.5;

enum class Scenario {
    Cross,
    Adaptive,
};

struct BasePose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
};

struct GeneratedScenario {
    Scenario scenario = Scenario::Cross;
    std::string scenario_name = "cross";
    int num_robots = 0;
    int robots_per_line = 0;
    double line_distance = 1.35;
    double spacing = 1.2;
    double start_angle = kStartAngle;
    double goal_angle = kGoalAngle;
    double wall_radius = kWallRadius;
    double boundary_margin = kBoundaryMargin;
    double backboard_offset = kBackboardOffset;
    double adaptive_obstacle_radius = kAdaptiveObstacleRadius;
    double adaptive_obstacle_offset = kAdaptiveObstacleOffset;
    bool reverse_adaptive_endpoints = false;
    double side_min_y = 0.0;
    double side_max_y = 0.0;
    double left_backboard_x = 0.0;
    double right_backboard_x = 0.0;
    std::vector<std::string> robot_names;
    std::vector<BasePose> base_poses;
    std::vector<std::vector<double>> starts;
    std::vector<std::vector<double>> goals;
    std::vector<comotion::ObstacleSphere> sphere_obstacles;
    std::vector<comotion::ObstacleCylinder> cylinder_obstacles;
};

inline std::string scenarioName(Scenario scenario) {
    switch (scenario) {
    case Scenario::Cross:
        return "cross";
    case Scenario::Adaptive:
        return "adaptive";
    }
    return "unknown";
}

inline Scenario parseScenarioName(const std::string &name) {
    if (name == "cross")
        return Scenario::Cross;
    if (name == "adaptive")
        return Scenario::Adaptive;
    throw std::runtime_error("Unknown planar manipulator scenario: " + name);
}

inline Eigen::Affine3d baseTransform(const BasePose &pose) {
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = Eigen::Vector3d(pose.x, pose.y, pose.z);
    transform.linear() =
        Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return transform;
}

inline std::vector<double> jointConfig(double joint1) {
    return {joint1, 0.0, 0.0};
}

inline std::vector<double> jointConfig(double joint1, double joint2,
                                       double joint3) {
    return {joint1, joint2, joint3};
}

inline double normalizeAngle(double angle) {
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle <= -kPi)
        angle += 2.0 * kPi;
    return angle;
}

inline double yawToward(double base_x, double base_y, double target_x,
                        double target_y) {
    const double dx = target_x - base_x;
    const double dy = target_y - base_y;
    if (std::abs(dx) < 1e-12 && std::abs(dy) < 1e-12)
        throw std::runtime_error(
            "adaptive obstacle cannot be colocated with the robot base");
    return std::atan2(dy, dx);
}

inline double yawAwayFrom(double base_x, double base_y, double target_x,
                          double target_y) {
    return normalizeAngle(yawToward(base_x, base_y, target_x, target_y) + kPi);
}

inline std::vector<double> configRelativeToBase(
    const std::vector<double> &world_config, double base_yaw) {
    std::vector<double> local = world_config;
    if (!local.empty())
        local[0] = normalizeAngle(local[0] - base_yaw);
    return local;
}

inline comotion::ObstacleSphere sphereObstacle(double x, double y, double radius) {
    return {Eigen::Vector3d(x, y, 0.0), radius};
}

inline std::vector<double> evenlySpacedCenters(int count, double spacing) {
    std::vector<double> centers;
    centers.reserve(static_cast<std::size_t>(count));
    const double center = 0.5 * static_cast<double>(count - 1);
    for (int i = 0; i < count; ++i)
        centers.push_back((static_cast<double>(i) - center) * spacing);
    return centers;
}

inline comotion::ObstacleCylinder axisCylinder(double x, double y,
                                           const Eigen::Vector3d &axis,
                                           double radius,
                                           double half_height) {
    comotion::ObstacleCylinder cylinder;
    cylinder.center = Eigen::Vector3d(x, y, 0.0);
    cylinder.axis = axis.normalized();
    cylinder.radius = radius;
    cylinder.half_height = half_height;
    return cylinder;
}

inline comotion::ObstacleCylinder xAxisCylinder(double x, double y,
                                            double radius,
                                            double half_height) {
    return axisCylinder(x, y, Eigen::Vector3d::UnitX(), radius, half_height);
}

inline comotion::ObstacleCylinder yAxisCylinder(double x, double y,
                                            double radius,
                                            double half_height) {
    return axisCylinder(x, y, Eigen::Vector3d::UnitY(), radius, half_height);
}

inline void validateInputs(int num_robots, double line_distance,
                           double spacing) {
    if (num_robots < 4)
        throw std::runtime_error("--num-robots must be at least 4");
    if (num_robots % 2 != 0)
        throw std::runtime_error("--num-robots must be even");
    if (line_distance <= 0.0)
        throw std::runtime_error("--line-distance must be positive");
    if (spacing <= 0.0)
        throw std::runtime_error("--spacing must be positive");
}

inline void validateAdaptiveInputs(int num_robots, double spacing) {
    if (num_robots != 8)
        throw std::runtime_error(
            "--num-robots must be exactly 8 for --scenario adaptive");
    if (spacing <= 0.0)
        throw std::runtime_error("--spacing must be positive");
}

inline void appendAdaptiveRobot(GeneratedScenario &generated,
                                const std::string &name, double base_x,
                                double base_y,
                                const std::vector<double> &original_start_config,
                                const std::vector<double> &original_goal_config,
                                double obstacle_x,
                                double obstacle_y) {
    const double base_yaw =
        yawAwayFrom(base_x, base_y, obstacle_x, obstacle_y);
    const auto &world_start_config = generated.reverse_adaptive_endpoints
                                         ? original_start_config
                                         : original_goal_config;
    const auto &world_goal_config = generated.reverse_adaptive_endpoints
                                        ? original_goal_config
                                        : original_start_config;
    generated.robot_names.push_back(name);
    generated.base_poses.push_back({base_x, base_y, 0.0, base_yaw});
    generated.starts.push_back(
        configRelativeToBase(world_start_config, base_yaw));
    generated.goals.push_back(
        configRelativeToBase(world_goal_config, base_yaw));
    generated.sphere_obstacles.push_back(
        sphereObstacle(obstacle_x, obstacle_y, kAdaptiveObstacleRadius));
}

inline void generateCross(GeneratedScenario &generated) {
    const double left_x = -0.5 * generated.line_distance;
    const double right_x = 0.5 * generated.line_distance;
    const auto ys = evenlySpacedCenters(generated.robots_per_line,
                                        generated.spacing);

    const double min_y = *std::min_element(ys.begin(), ys.end());
    const double max_y = *std::max_element(ys.begin(), ys.end());
    generated.side_min_y = min_y - kBoundaryMargin;
    generated.side_max_y = max_y + kBoundaryMargin;
    generated.left_backboard_x = left_x - kBackboardOffset;
    generated.right_backboard_x = right_x + kBackboardOffset;

    for (int i = 0; i < generated.robots_per_line; ++i) {
        const double y = ys[static_cast<std::size_t>(i)];
        generated.robot_names.push_back("left_" + std::to_string(i));
        generated.base_poses.push_back({left_x, y, 0.0, 0.0});
        generated.starts.push_back(jointConfig(kStartAngle));
        generated.goals.push_back(jointConfig(kGoalAngle));
    }
    for (int i = 0; i < generated.robots_per_line; ++i) {
        const double y = ys[static_cast<std::size_t>(i)];
        generated.robot_names.push_back("right_" + std::to_string(i));
        generated.base_poses.push_back({right_x, y, 0.0, kPi});
        generated.starts.push_back(jointConfig(kStartAngle));
        generated.goals.push_back(jointConfig(kGoalAngle));
    }

    const double center_y = 0.5 * (generated.side_min_y + generated.side_max_y);
    const double backboard_half_height =
        0.5 * (generated.side_max_y - generated.side_min_y);
    const double center_x =
        0.5 * (generated.left_backboard_x + generated.right_backboard_x);
    const double side_half_height =
        0.5 * (generated.right_backboard_x - generated.left_backboard_x);

    generated.cylinder_obstacles.push_back(yAxisCylinder(
        generated.left_backboard_x, center_y, kWallRadius,
        backboard_half_height));
    generated.cylinder_obstacles.push_back(yAxisCylinder(
        generated.right_backboard_x, center_y, kWallRadius,
        backboard_half_height));
    generated.cylinder_obstacles.push_back(xAxisCylinder(
        center_x, generated.side_min_y, kWallRadius, side_half_height));
    generated.cylinder_obstacles.push_back(xAxisCylinder(
        center_x, generated.side_max_y, kWallRadius, side_half_height));
}

inline void generateAdaptive(GeneratedScenario &generated) {
    constexpr double up_right = kPi / 4.0;
    constexpr double up_left = 3.0 * kPi / 4.0;
    constexpr double down_right = -kPi / 4.0;
    constexpr double down_left = -3.0 * kPi / 4.0;
    constexpr double wrist_fold = 0.0;

    const double inner = generated.spacing / std::sqrt(2.0);
    const double outer = inner + generated.spacing;
    const double offset = kAdaptiveObstacleOffset;

    appendAdaptiveRobot(generated, "top_inner", 0.0, inner,
                        jointConfig(down_left, 0.0, wrist_fold),
                        jointConfig(up_left), -offset, inner);
    appendAdaptiveRobot(generated, "top_outer", 0.0, outer,
                        jointConfig(up_right),
                        jointConfig(down_right), offset,
                        outer);
    appendAdaptiveRobot(generated, "bottom_inner", 0.0, -inner,
                        jointConfig(up_right, 0.0, wrist_fold),
                        jointConfig(down_right), offset, -inner);
    appendAdaptiveRobot(generated, "bottom_outer", 0.0, -outer,
                        jointConfig(down_left),
                        jointConfig(up_left), -offset,
                        -outer);
    appendAdaptiveRobot(generated, "left_inner", -inner, 0.0,
                        jointConfig(down_right, 0.0, wrist_fold),
                        jointConfig(down_left, 0.0, wrist_fold),
                        -inner, -offset);
    appendAdaptiveRobot(generated, "left_outer", -outer, 0.0,
                        jointConfig(up_left),
                        jointConfig(up_right),
                        -outer, offset);
    appendAdaptiveRobot(generated, "right_inner", inner, 0.0,
                        jointConfig(up_left, 0.0, wrist_fold),
                        jointConfig(up_right, 0.0, wrist_fold),
                        inner, offset);
    appendAdaptiveRobot(generated, "right_outer", outer, 0.0,
                        jointConfig(down_right),
                        jointConfig(down_left),
                        outer, -offset);
}

inline GeneratedScenario generateScenario(Scenario scenario, int num_robots,
                                          double line_distance,
                                          double spacing,
                                          bool reverse_adaptive_endpoints =
                                              false) {
    if (scenario == Scenario::Adaptive)
        validateAdaptiveInputs(num_robots, spacing);
    else
        validateInputs(num_robots, line_distance, spacing);

    GeneratedScenario generated;
    generated.scenario = scenario;
    generated.scenario_name = scenarioName(scenario);
    generated.num_robots = num_robots;
    generated.robots_per_line = num_robots / 2;
    generated.line_distance = line_distance;
    generated.spacing = spacing;
    generated.start_angle = kStartAngle;
    generated.goal_angle = kGoalAngle;
    generated.wall_radius = kWallRadius;
    generated.boundary_margin = kBoundaryMargin;
    generated.backboard_offset = kBackboardOffset;
    generated.adaptive_obstacle_radius = kAdaptiveObstacleRadius;
    generated.adaptive_obstacle_offset = kAdaptiveObstacleOffset;
    generated.reverse_adaptive_endpoints =
        scenario == Scenario::Adaptive && reverse_adaptive_endpoints;
    generated.robot_names.reserve(static_cast<std::size_t>(num_robots));
    generated.base_poses.reserve(static_cast<std::size_t>(num_robots));
    generated.starts.reserve(static_cast<std::size_t>(num_robots));
    generated.goals.reserve(static_cast<std::size_t>(num_robots));

    if (scenario == Scenario::Adaptive)
        generateAdaptive(generated);
    else
        generateCross(generated);

    return generated;
}

} // namespace comotion::benchmark_apps::planar_manipulator_cross
