#pragma once

#include "comotion/collision/ObstacleShapes.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace comotion::benchmark_apps::mobile_robot_2d {

constexpr double kBoundaryCylinderRadiusFactor = 0.1;

inline double boundaryCylinderRadius(double robot_radius) {
    return kBoundaryCylinderRadiusFactor * robot_radius;
}

inline double boundaryCylinderOffset(double robot_radius) {
    return boundaryCylinderRadius(robot_radius);
}

inline double endpointFaceDistance(double robot_radius) {
    return 2.0 * robot_radius;
}

enum class CrossingScenario {
    Circle,
    Parallel,
    Perpendicular,
    Hallways,
    Adaptive,
    Inlet,
};

struct HallwayOptions {
    int vertical_hallways = -1;
    int horizontal_hallways = 0;
    double hallway_radius = -1.0;
};

struct AdaptiveOptions {
    double hallway_width = -1.0;
    double intersection_width = -1.0;
};

struct InletOptions {
    double hallway_width = -1.0;
    double hallway_length = -1.0;
};

struct GeneratedScenario {
    CrossingScenario scenario;
    std::string scenario_name;
    int num_robots = 0;
    double robot_radius = 1.0;
    double spacing = 5.0;
    int vertical_hallways = 0;
    int horizontal_hallways = 0;
    double hallway_radius = 0.0;
    double hallway_width = 0.0;
    double hallway_length = 0.0;
    double intersection_width = 0.0;
    std::vector<std::vector<double>> starts;
    std::vector<std::vector<double>> goals;
    std::vector<double> env_min;
    std::vector<double> env_max;
    std::vector<comotion::ObstacleCylinder> cylinder_obstacles;
};

inline std::string scenarioName(CrossingScenario scenario) {
    switch (scenario) {
    case CrossingScenario::Circle:
        return "circle";
    case CrossingScenario::Parallel:
        return "parallel";
    case CrossingScenario::Perpendicular:
        return "perpendicular";
    case CrossingScenario::Hallways:
        return "hallways";
    case CrossingScenario::Adaptive:
        return "adaptive";
    case CrossingScenario::Inlet:
        return "inlet";
    }
    return "unknown";
}

inline CrossingScenario parseScenarioName(const std::string &name) {
    if (name == "circle")
        return CrossingScenario::Circle;
    if (name == "parallel")
        return CrossingScenario::Parallel;
    if (name == "perpendicular")
        return CrossingScenario::Perpendicular;
    if (name == "hallways")
        return CrossingScenario::Hallways;
    if (name == "adaptive")
        return CrossingScenario::Adaptive;
    if (name == "inlet")
        return CrossingScenario::Inlet;
    throw std::runtime_error("Unknown crossing scenario: " + name);
}

inline std::vector<double> point(double x, double y) {
    return {x, y, 0.0};
}

inline void updateBounds(const std::vector<double> &p, double &min_x,
                         double &max_x, double &min_y, double &max_y) {
    min_x = std::min(min_x, p[0]);
    max_x = std::max(max_x, p[0]);
    min_y = std::min(min_y, p[1]);
    max_y = std::max(max_y, p[1]);
}

inline void setPlanarBounds(GeneratedScenario &generated) {
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (const auto &p : generated.starts)
        updateBounds(p, min_x, max_x, min_y, max_y);
    for (const auto &p : generated.goals)
        updateBounds(p, min_x, max_x, min_y, max_y);
    for (const auto &cylinder : generated.cylinder_obstacles) {
        Eigen::Vector3d axis = cylinder.axis;
        if (axis.squaredNorm() <= 1e-12)
            axis = Eigen::Vector3d::UnitY();
        axis.normalize();
        const Eigen::Vector3d a =
            cylinder.center - axis * cylinder.half_height;
        const Eigen::Vector3d b =
            cylinder.center + axis * cylinder.half_height;
        updateBounds(point(a.x() - cylinder.radius, a.y() - cylinder.radius),
                     min_x, max_x, min_y, max_y);
        updateBounds(point(a.x() + cylinder.radius, a.y() + cylinder.radius),
                     min_x, max_x, min_y, max_y);
        updateBounds(point(b.x() - cylinder.radius, b.y() - cylinder.radius),
                     min_x, max_x, min_y, max_y);
        updateBounds(point(b.x() + cylinder.radius, b.y() + cylinder.radius),
                     min_x, max_x, min_y, max_y);
    }

    if (!std::isfinite(min_x) || !std::isfinite(max_x))
        throw std::runtime_error("Generated scenario has no start/goal points");

    const double buffer = 2.0 * generated.robot_radius;
    const double center_x = 0.5 * (min_x + max_x);
    const double center_y = 0.5 * (min_y + max_y);
    const double half_extent =
        0.5 * std::max(max_x - min_x, max_y - min_y) + buffer;
    generated.env_min = {center_x - half_extent, center_y - half_extent, 0.0};
    generated.env_max = {center_x + half_extent, center_y + half_extent, 0.0};
}

inline void validateScenarioInputs(CrossingScenario scenario, int num_robots,
                                   double robot_radius,
                                   double spacing) {
    if (scenario == CrossingScenario::Inlet) {
        if (num_robots != 2)
            throw std::runtime_error(
                "--num-robots must be exactly 2 for --scenario inlet");
    } else if (num_robots < 4) {
        throw std::runtime_error("--num-robots must be at least 4");
    }
    if (num_robots % 2 != 0)
        throw std::runtime_error("--num-robots must be even");
    if (robot_radius <= 0.0)
        throw std::runtime_error("--robot-radius must be positive");
    if (spacing <= 2.0 * robot_radius)
        throw std::runtime_error(
            "--spacing must be greater than twice --robot-radius");
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

inline void appendBoundaryCylinders(GeneratedScenario &generated) {
    if (generated.env_min.size() != 3 || generated.env_max.size() != 3)
        throw std::runtime_error(
            "Cannot append boundary cylinders before environment bounds exist");

    const double radius = boundaryCylinderRadius(generated.robot_radius);
    const double offset = boundaryCylinderOffset(generated.robot_radius);
    const double min_x = generated.env_min[0];
    const double max_x = generated.env_max[0];
    const double min_y = generated.env_min[1];
    const double max_y = generated.env_max[1];
    const double center_x = 0.5 * (min_x + max_x);
    const double center_y = 0.5 * (min_y + max_y);
    const double half_width = 0.5 * (max_x - min_x) + offset;
    const double half_height = 0.5 * (max_y - min_y) + offset;

    generated.cylinder_obstacles.push_back(
        yAxisCylinder(min_x - offset, center_y, radius, half_height));
    generated.cylinder_obstacles.push_back(
        yAxisCylinder(max_x + offset, center_y, radius, half_height));
    generated.cylinder_obstacles.push_back(
        xAxisCylinder(center_x, min_y - offset, radius, half_width));
    generated.cylinder_obstacles.push_back(
        xAxisCylinder(center_x, max_y + offset, radius, half_width));
}

inline void validateHallwayInputs(int vertical_hallways,
                                  int horizontal_hallways,
                                  double hallway_radius,
                                  double robot_radius,
                                  double spacing) {
    if (vertical_hallways < 1)
        throw std::runtime_error("--vertical-hallways must be at least 1");
    if (horizontal_hallways < 0)
        throw std::runtime_error("--horizontal-hallways must be non-negative");
    if (hallway_radius <= robot_radius)
        throw std::runtime_error(
            "--hallway-radius must be greater than --robot-radius");
    if (spacing <= 2.0 * hallway_radius)
        throw std::runtime_error(
            "--spacing must be greater than twice --hallway-radius");
}

inline void generateHallways(GeneratedScenario &generated,
                             const HallwayOptions &options) {
    const int pairs = generated.num_robots / 2;
    generated.vertical_hallways =
        options.vertical_hallways >= 0 ? options.vertical_hallways : pairs - 1;
    generated.horizontal_hallways = options.horizontal_hallways;
    generated.hallway_radius =
        options.hallway_radius > 0.0 ? options.hallway_radius
                                     : 2.0 * generated.robot_radius;

    validateHallwayInputs(generated.vertical_hallways,
                          generated.horizontal_hallways,
                          generated.hallway_radius,
                          generated.robot_radius, generated.spacing);

    const double top_y = generated.spacing * static_cast<double>(pairs);
    const auto robot_xs = evenlySpacedCenters(pairs, generated.spacing);
    for (double x : robot_xs) {
        generated.starts.push_back(point(x, top_y));
        generated.goals.push_back(point(x, -top_y));
    }
    for (double x : robot_xs) {
        generated.starts.push_back(point(x, -top_y));
        generated.goals.push_back(point(x, top_y));
    }

    const int obstacle_columns = generated.vertical_hallways + 1;
    const auto obstacle_centers =
        evenlySpacedCenters(obstacle_columns, generated.spacing);
    const double obstacle_radius =
        0.5 * (generated.spacing - 2.0 * generated.hallway_radius);
    const double rack_y_extent = top_y - generated.hallway_radius;
    if (rack_y_extent <= generated.hallway_radius)
        throw std::runtime_error(
            "hallway layout does not leave positive rack height");

    std::vector<std::pair<double, double>> rack_segments;
    const auto horizontal_centers =
        evenlySpacedCenters(generated.horizontal_hallways, generated.spacing);
    double segment_begin = -rack_y_extent;
    for (double center : horizontal_centers) {
        const double gap_begin = center - generated.hallway_radius;
        const double gap_end = center + generated.hallway_radius;
        if (gap_begin <= segment_begin || gap_end >= rack_y_extent)
            throw std::runtime_error(
                "horizontal hallways do not fit in the generated layout");
        rack_segments.push_back({segment_begin, gap_begin});
        segment_begin = gap_end;
    }
    rack_segments.push_back({segment_begin, rack_y_extent});

    for (const auto &segment : rack_segments) {
        if (segment.second <= segment.first)
            throw std::runtime_error(
                "hallway layout produced a non-positive rack segment");
    }

    for (int i = 0; i < obstacle_columns; ++i) {
        const double x = obstacle_centers[static_cast<std::size_t>(i)];
        for (const auto &segment : rack_segments) {
            const double y = 0.5 * (segment.first + segment.second);
            const double half_height = 0.5 * (segment.second - segment.first);
            generated.cylinder_obstacles.push_back(
                yAxisCylinder(x, y, obstacle_radius, half_height));
        }
    }

    const double left_rack_face = obstacle_centers.front() - obstacle_radius;
    const double right_rack_face = obstacle_centers.back() + obstacle_radius;
    const double bottom_rack_face = -rack_y_extent;
    const double top_rack_face = rack_y_extent;
    const double hallway_width = 2.0 * generated.hallway_radius;
    generated.env_min = {left_rack_face - hallway_width,
                         bottom_rack_face - hallway_width, 0.0};
    generated.env_max = {right_rack_face + hallway_width,
                         top_rack_face + hallway_width, 0.0};
}

inline void validateAdaptiveInputs(int num_robots, double hallway_width,
                                   double intersection_width,
                                   double robot_radius, double spacing) {
    if (num_robots != 4 && num_robots != 8 && num_robots != 16)
        throw std::runtime_error(
            "--num-robots must be 4, 8, or 16 for --scenario adaptive");
    if (hallway_width <= 2.0 * robot_radius)
        throw std::runtime_error(
            "--hallway-width must be greater than twice --robot-radius");
    if (intersection_width <= hallway_width)
        throw std::runtime_error(
            "--intersection-width must be greater than --hallway-width");
    if (spacing <= 2.0 * robot_radius)
        throw std::runtime_error(
            "--spacing must be greater than twice --robot-radius");
    if (spacing <= 4.0 * robot_radius)
        throw std::runtime_error(
            "--spacing must be greater than four times --robot-radius for "
            "adaptive endpoint clearance");
}

inline void appendPair(GeneratedScenario &generated,
                       const std::vector<double> &start,
                       const std::vector<double> &goal) {
    generated.starts.push_back(start);
    generated.goals.push_back(goal);
}

inline void generateAdaptive(GeneratedScenario &generated,
                             const AdaptiveOptions &options) {
    generated.hallway_width =
        options.hallway_width > 0.0 ? options.hallway_width
                                    : 4.0 * generated.robot_radius;
    generated.intersection_width =
        options.intersection_width > 0.0
            ? options.intersection_width
            : 2.0 * generated.hallway_width;

    validateAdaptiveInputs(generated.num_robots, generated.hallway_width,
                           generated.intersection_width,
                           generated.robot_radius, generated.spacing);

    const double r = generated.robot_radius;
    const double h = generated.hallway_width;
    const double i = generated.intersection_width;
    const double outer = 0.5 * i + generated.spacing;
    const double world = outer + h;
    const double obstacle_radius = 0.5 * generated.spacing;
    const double y_leg_center = 0.25 * (h + 2.0 * outer);
    const double y_leg_half_height = 0.5 * (outer - 0.5 * h);
    const double x_leg_center = 0.25 * (h + 2.0 * outer);
    const double x_leg_half_height = 0.5 * (outer - 0.5 * h);
    const double obstacle_center = 0.25 * i + 0.5 * outer;

    for (double sx : {-1.0, 1.0}) {
        for (double sy : {-1.0, 1.0}) {
            generated.cylinder_obstacles.push_back(yAxisCylinder(
                sx * obstacle_center, sy * y_leg_center, obstacle_radius,
                y_leg_half_height));
            generated.cylinder_obstacles.push_back(xAxisCylinder(
                sx * x_leg_center, sy * obstacle_center, obstacle_radius,
                x_leg_half_height));
        }
    }

    const double endpoint_distance = endpointFaceDistance(r);
    const double inner = 0.5 * i + endpoint_distance;
    const double exterior = outer - endpoint_distance;
    const std::vector<std::vector<double>> inner_starts = {
        point(0.0, inner), point(inner, 0.0), point(0.0, -inner),
        point(-inner, 0.0)};
    const std::vector<std::vector<double>> exterior_starts = {
        point(0.0, exterior), point(exterior, 0.0), point(0.0, -exterior),
        point(-exterior, 0.0)};

    for (int index = 0; index < 4; ++index) {
        const int opposite = (index + 2) % 4;
        appendPair(generated, inner_starts[static_cast<std::size_t>(index)],
                   exterior_starts[static_cast<std::size_t>(opposite)]);
    }

    if (generated.num_robots >= 8) {
        for (int index = 0; index < 4; ++index) {
            const int opposite = (index + 2) % 4;
            appendPair(
                generated, exterior_starts[static_cast<std::size_t>(index)],
                inner_starts[static_cast<std::size_t>(opposite)]);
        }
    }

    if (generated.num_robots == 16) {
        const double outer_hallway_mid = outer + 0.5 * h;
        const double obstacle_outer_aligned_axis = outer - endpoint_distance;
        const std::vector<std::vector<double>> outer_starts = {
            point(-obstacle_outer_aligned_axis, outer_hallway_mid),
            point(obstacle_outer_aligned_axis, outer_hallway_mid),
            point(outer_hallway_mid, obstacle_outer_aligned_axis),
            point(outer_hallway_mid, -obstacle_outer_aligned_axis),
            point(obstacle_outer_aligned_axis, -outer_hallway_mid),
            point(-obstacle_outer_aligned_axis, -outer_hallway_mid),
            point(-outer_hallway_mid, -obstacle_outer_aligned_axis),
            point(-outer_hallway_mid, obstacle_outer_aligned_axis),
        };
        for (std::size_t index = 0; index < outer_starts.size();
             index += 2) {
            appendPair(generated, outer_starts[index],
                       outer_starts[index + 1]);
            appendPair(generated, outer_starts[index + 1],
                       outer_starts[index]);
        }
    }

    generated.env_min = {-world, -world, 0.0};
    generated.env_max = {world, world, 0.0};
}

inline void validateInletInputs(int num_robots, double hallway_width,
                                double hallway_length, double robot_radius) {
    if (num_robots != 2)
        throw std::runtime_error(
            "--num-robots must be exactly 2 for --scenario inlet");
    if (hallway_width <= 2.0 * robot_radius)
        throw std::runtime_error(
            "--hallway-width must be greater than twice --robot-radius");
    if (hallway_length <= hallway_width)
        throw std::runtime_error(
            "--hallway-length must be greater than --hallway-width");
    if (hallway_length <= 4.0 * robot_radius)
        throw std::runtime_error(
            "--hallway-length must be greater than four times --robot-radius");
}

inline void generateInlet(GeneratedScenario &generated,
                          const InletOptions &options) {
    generated.hallway_width =
        options.hallway_width > 0.0 ? options.hallway_width
                                    : 4.0 * generated.robot_radius;
    generated.hallway_length =
        options.hallway_length > 0.0 ? options.hallway_length
                                     : 4.0 * generated.spacing;

    validateInletInputs(generated.num_robots, generated.hallway_width,
                        generated.hallway_length, generated.robot_radius);

    const double r = generated.robot_radius;
    const double width = generated.hallway_width;
    const double length = generated.hallway_length;
    const double left = -0.5 * length;
    const double right = 0.5 * length;
    const double bottom = 0.0;
    const double top = 2.0 * width;
    const double center_y = 0.5 * width;

    generated.env_min = {left, bottom, 0.0};
    generated.env_max = {right, top, 0.0};

    const double endpoint_distance = endpointFaceDistance(r);
    appendPair(generated, point(left + endpoint_distance, center_y),
               point(right - endpoint_distance, center_y));
    appendPair(generated, point(right - endpoint_distance, center_y),
               point(left + endpoint_distance, center_y));

    const double boundary_radius = boundaryCylinderRadius(r);
    const double boundary_offset = boundaryCylinderOffset(r);
    const double left_boundary_inner_x =
        generated.env_min[0] - boundary_offset + boundary_radius;
    const double right_boundary_inner_x =
        generated.env_max[0] + boundary_offset - boundary_radius;
    const double top_boundary_inner_y =
        generated.env_max[1] + boundary_offset - boundary_radius;

    const double gap_left_x = -0.5 * width;
    const double gap_right_x = 0.5 * width;
    const double obstacle_bottom_y = width;
    const double obstacle_radius =
        0.5 * (top_boundary_inner_y - obstacle_bottom_y);
    const double obstacle_center_y = obstacle_bottom_y + obstacle_radius;
    const double left_obstacle_half_height =
        0.5 * (gap_left_x - left_boundary_inner_x);
    const double right_obstacle_half_height =
        0.5 * (right_boundary_inner_x - gap_right_x);
    const double left_obstacle_center_x =
        0.5 * (left_boundary_inner_x + gap_left_x);
    const double right_obstacle_center_x =
        0.5 * (gap_right_x + right_boundary_inner_x);

    generated.cylinder_obstacles.push_back(xAxisCylinder(
        left_obstacle_center_x, obstacle_center_y, obstacle_radius,
        left_obstacle_half_height));
    generated.cylinder_obstacles.push_back(xAxisCylinder(
        right_obstacle_center_x, obstacle_center_y, obstacle_radius,
        right_obstacle_half_height));
}

inline GeneratedScenario generateScenario(CrossingScenario scenario,
                                          int num_robots,
                                          double robot_radius,
                                          double spacing,
                                          HallwayOptions hallway_options = {},
                                          AdaptiveOptions adaptive_options = {},
                                          InletOptions inlet_options = {}) {
    validateScenarioInputs(scenario, num_robots, robot_radius, spacing);

    GeneratedScenario generated;
    generated.scenario = scenario;
    generated.scenario_name = scenarioName(scenario);
    generated.num_robots = num_robots;
    generated.robot_radius = robot_radius;
    generated.spacing = spacing;
    generated.starts.reserve(static_cast<std::size_t>(num_robots));
    generated.goals.reserve(static_cast<std::size_t>(num_robots));

    const int pairs = num_robots / 2;
    constexpr double kPi = 3.141592653589793238462643383279502884;

    if (scenario == CrossingScenario::Circle) {
        const double circle_radius =
            std::max(spacing, spacing * static_cast<double>(num_robots) /
                                  (2.0 * kPi));
        for (int i = 0; i < num_robots; ++i) {
            const double theta =
                2.0 * kPi * static_cast<double>(i) /
                static_cast<double>(num_robots);
            generated.starts.push_back(
                point(circle_radius * std::cos(theta),
                      circle_radius * std::sin(theta)));
        }
        for (int i = 0; i < num_robots; ++i)
            generated.goals.push_back(
                generated.starts[static_cast<std::size_t>((i + pairs) %
                                                          num_robots)]);
    } else if (scenario == CrossingScenario::Parallel) {
        const double x_extent = spacing * static_cast<double>(pairs);
        const double center = 0.5 * static_cast<double>(pairs - 1);
        for (int i = 0; i < pairs; ++i) {
            const double y = (static_cast<double>(i) - center) * spacing;
            generated.starts.push_back(point(x_extent, y));
            generated.goals.push_back(point(-x_extent, y));
            generated.starts.push_back(point(-x_extent, y));
            generated.goals.push_back(point(x_extent, y));
        }
    } else if (scenario == CrossingScenario::Perpendicular) {
        const double line_extent = spacing * static_cast<double>(pairs);
        const double center = 0.5 * static_cast<double>(pairs - 1);
        for (int i = 0; i < pairs; ++i) {
            const double offset =
                (static_cast<double>(i) - center) * spacing;
            generated.starts.push_back(point(-line_extent, offset));
            generated.goals.push_back(point(line_extent, offset));
            generated.starts.push_back(point(offset, -line_extent));
            generated.goals.push_back(point(offset, line_extent));
        }
    } else if (scenario == CrossingScenario::Hallways) {
        generateHallways(generated, hallway_options);
    } else if (scenario == CrossingScenario::Adaptive) {
        generateAdaptive(generated, adaptive_options);
    } else if (scenario == CrossingScenario::Inlet) {
        generateInlet(generated, inlet_options);
    }

    if (scenario != CrossingScenario::Hallways &&
        scenario != CrossingScenario::Adaptive &&
        scenario != CrossingScenario::Inlet)
        setPlanarBounds(generated);
    appendBoundaryCylinders(generated);
    return generated;
}

} // namespace comotion::benchmark_apps::mobile_robot_2d
