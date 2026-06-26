#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/robot/FlyingSphere.h"

#include <ompl/base/ScopedState.h>
#include <vamp/collision/factory.hh>
#include <vamp/collision/sphere_cylinder.hh>
#include <vamp/vector.hh>

#include <Eigen/Core>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace ob = ompl::base;

namespace
{
    auto make_z_cylinder(const Eigen::Vector3d &center, double radius, double half_height)
        -> comotion::ObstacleCylinder
    {
        comotion::ObstacleCylinder cylinder;
        cylinder.center = center;
        cylinder.axis = Eigen::Vector3d::UnitZ();
        cylinder.radius = radius;
        cylinder.half_height = half_height;
        return cylinder;
    }

    auto expect_equal(const std::string &label, bool actual, bool expected) -> bool
    {
        if (actual != expected)
        {
            std::cerr << "vamp_cylinder_regression: " << label << " expected " << expected << " got "
                      << actual << "\n";
            return false;
        }
        return true;
    }
}  // namespace

int main()
{
    auto robot = std::make_shared<comotion::FlyingSphere>(0.1, -5.0, 5.0);

    const auto cylinder = make_z_cylinder(Eigen::Vector3d(0.0, 0.0, 2.0), 1.0, 1.0);

    comotion::CollisionChecker sphere_backend(comotion::CollisionChecker::Backend::Spheres);
    comotion::CollisionChecker vamp_backend(comotion::CollisionChecker::Backend::Vamp);
    sphere_backend.setCylinderObstacles({cylinder});
    vamp_backend.setCylinderObstacles({cylinder});

    const auto check_case = [&](const std::string &label,
                                const std::vector<double> &config,
                                bool expected_valid) -> bool
    {
        const bool sphere_valid = sphere_backend.isValidSingle(*robot, config);
        const bool vamp_valid = vamp_backend.isValidSingle(*robot, config);
        return expect_equal(label + " sphere_backend", sphere_valid, expected_valid) &&
               expect_equal(label + " vamp_backend", vamp_valid, expected_valid) &&
               expect_equal(label + " parity", sphere_valid, vamp_valid);
    };

    if (!check_case("side_wall_collision", {1.05, 0.0, 2.0}, false))
        return 1;
    if (!check_case("flat_end_collision", {0.0, 0.0, 3.05}, false))
        return 1;
    if (!check_case("capsule_false_positive_corner", {1.09, 0.0, 3.09}, true))
        return 1;
    if (!check_case("clear_separation", {1.25, 0.0, 2.0}, true))
        return 1;

    const auto mobile_boundary_robot = std::make_shared<comotion::FlyingSphere>(
        0.5, std::vector<double>{-321.0, -321.0, 0.0},
        std::vector<double>{321.0, 321.0, 0.0});
    comotion::ObstacleCylinder right_wall;
    right_wall.center = Eigen::Vector3d(321.05, 0.0, 0.0);
    right_wall.axis = Eigen::Vector3d::UnitY();
    right_wall.radius = 0.05;
    right_wall.half_height = 321.05;

    const auto direct_wall = vamp::collision::factory::cylinder::endpoints::eigen(
        Eigen::Vector3f(321.05f, 321.05f, 0.0f),
        Eigen::Vector3f(321.05f, -321.05f, 0.0f), 0.05f);
    const float direct_clearance = vamp::collision::sphere_cylinder(
        direct_wall, 320.0f, -157.5f, 0.0f, 0.5f);
    if (!(direct_clearance > 0.0f)) {
        std::cerr << "vamp_cylinder_regression: direct right-wall clearance "
                  << direct_clearance << " expected positive\n";
        return 1;
    }
    const vamp::collision::Cylinder<vamp::FloatVector<>> vector_wall(
        direct_wall);
    const auto vector_clearance = vamp::collision::sphere_cylinder(
        vector_wall, vamp::FloatVector<>(320.0f),
        vamp::FloatVector<>(-157.5f), vamp::FloatVector<>(0.0f),
        vamp::FloatVector<>(0.5f));
    std::array<float, vamp::FloatVector<>::num_scalars_rounded> vector_buffer{};
    vector_clearance.to_array_unaligned(vector_buffer.data());
    if (!(vector_buffer[0] > 0.0f)) {
        std::cerr << "vamp_cylinder_regression: vector right-wall clearance "
                  << vector_buffer[0] << " expected positive\n";
        return 1;
    }

    comotion::CollisionChecker mobile_sphere_backend(comotion::CollisionChecker::Backend::Spheres);
    comotion::CollisionChecker mobile_vamp_backend(comotion::CollisionChecker::Backend::Vamp);
    mobile_sphere_backend.setCylinderObstacles({right_wall});
    mobile_vamp_backend.setCylinderObstacles({right_wall});

    const auto check_mobile_boundary_case = [&](const std::string &label,
                                                const std::vector<double> &config,
                                                bool expected_valid) -> bool
    {
        const bool sphere_valid = mobile_sphere_backend.isValidSingle(*mobile_boundary_robot, config);
        const bool vamp_valid = mobile_vamp_backend.isValidSingle(*mobile_boundary_robot, config);
        return expect_equal(label + " sphere_backend", sphere_valid, expected_valid) &&
               expect_equal(label + " vamp_backend", vamp_valid, expected_valid) &&
               expect_equal(label + " parity", sphere_valid, vamp_valid);
    };

    if (!check_mobile_boundary_case("mobile_parallel_right_wall_clear",
                                    {320.0, -157.5, 0.0}, true))
        return 1;
    if (!check_mobile_boundary_case("mobile_parallel_right_wall_contact",
                                    {320.5, -157.5, 0.0}, true))
        return 1;
    if (!check_mobile_boundary_case("mobile_parallel_right_wall_overlap",
                                    {320.51, -157.5, 0.0}, false))
        return 1;

    comotion::ObstacleSphere sphere_obstacle;
    sphere_obstacle.center = Eigen::Vector3d(0.0, 0.0, 2.0);
    sphere_obstacle.radius = 0.35;

    comotion::CollisionChecker sphere_only_ref(comotion::CollisionChecker::Backend::Spheres);
    comotion::CollisionChecker sphere_only_vamp(comotion::CollisionChecker::Backend::Vamp);
    sphere_only_ref.setObstacles({sphere_obstacle});
    sphere_only_vamp.setObstacles({sphere_obstacle});

    if (!expect_equal("sphere_only_collision sphere_backend",
                      sphere_only_ref.isValidSingle(*robot, {0.0, 0.0, 2.2}), false) ||
        !expect_equal("sphere_only_collision vamp_backend",
                      sphere_only_vamp.isValidSingle(*robot, {0.0, 0.0, 2.2}), false) ||
        !expect_equal("sphere_only_clear sphere_backend",
                      sphere_only_ref.isValidSingle(*robot, {1.0, 0.0, 2.0}), true) ||
        !expect_equal("sphere_only_clear vamp_backend",
                      sphere_only_vamp.isValidSingle(*robot, {1.0, 0.0, 2.0}), true))
    {
        return 1;
    }

    comotion::MultiRobotProblem problem(comotion::CollisionChecker::Backend::Vamp);
    problem.addRobot(robot, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0});
    problem.setCylinderObstacles({cylinder});

    auto si = problem.createSpaceInfo(0);
    ob::ScopedState<> colliding(si->getStateSpace());
    colliding[0] = 1.05;
    colliding[1] = 0.0;
    colliding[2] = 2.0;

    ob::ScopedState<> clear(si->getStateSpace());
    clear[0] = 1.09;
    clear[1] = 0.0;
    clear[2] = 3.09;

    if (!expect_equal("ompl_space_info_colliding", si->isValid(colliding.get()), false) ||
        !expect_equal("ompl_space_info_clear", si->isValid(clear.get()), true))
    {
        return 1;
    }

    std::cout << "vamp_cylinder_regression: OK\n";
    return 0;
}
