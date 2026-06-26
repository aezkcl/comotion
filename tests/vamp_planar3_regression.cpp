#include "comotion/collision/CollisionChecker.h"
#include "comotion/robot/RobotModel.h"

#include <vamp/collision/environment.hh>
#include <vamp/robots/planar3.hh>
#include <vamp/vector.hh>

#include <Eigen/Geometry>

#include <array>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace
{
    constexpr std::size_t kRake = vamp::FloatVectorWidth;

    std::string getResourcePath(const std::string &relative)
    {
        const char *prefixes[] = {"../resources/", "../../resources/",
                                  "../../../resources/", "resources/"};
        for (const char *pfx : prefixes)
        {
            std::string path = std::string(pfx) + relative;
            std::ifstream f(path);
            if (f.good())
                return path;
        }
        return std::string("resources/") + relative;
    }

    std::shared_ptr<comotion::RobotModel> loadPlanar3(const Eigen::Vector3d &base_xyz)
    {
        auto robot = std::make_shared<comotion::RobotModel>();
        robot->loadURDF(getResourcePath("planar3/planar3_spherized.urdf"));
        robot->loadSRDF(getResourcePath("planar3/planar3.srdf"));
        Eigen::Affine3d base = Eigen::Affine3d::Identity();
        base.translation() = base_xyz;
        robot->setBaseTransform(base);
        return robot;
    }

    bool expectEqual(const std::string &label, bool actual, bool expected)
    {
        if (actual != expected)
        {
            std::cerr << "vamp_planar3_regression: " << label << " expected "
                      << expected << " got " << actual << "\n";
            return false;
        }
        return true;
    }

    bool expectFamily(const comotion::RobotModel &robot,
                      comotion::RobotModel::RobotFamily expected)
    {
        const auto actual = robot.robotFamily();
        if (actual != expected)
        {
            std::cerr << "vamp_planar3_regression: expected family "
                      << static_cast<int>(expected) << " got "
                      << static_cast<int>(actual) << "\n";
            return false;
        }
        return true;
    }

    vamp::robots::Planar3::ConfigurationBlock<kRake>
    makePlanar3Block(const std::vector<double> &config)
    {
        vamp::robots::Planar3::ConfigurationBlock<kRake> block;
        for (std::size_t dim = 0; dim < vamp::robots::Planar3::dimension; ++dim)
        {
            std::array<float, kRake> lane_values{};
            lane_values.fill(static_cast<float>(config[dim]));
            using Row = std::decay_t<decltype(block[0])>;
            block[dim] = Row(lane_values);
        }
        return block;
    }

    bool hasLink1Link2CylinderCollision(const std::vector<double> &config)
    {
        vamp::collision::Environment<vamp::FloatVector<kRake>> empty_environment;
        const auto debug = vamp::robots::Planar3::fkcc_debug(
            empty_environment, makePlanar3Block(config));
        for (const auto &[a, b] : debug.second)
        {
            const bool a_link1 = a >= 1 && a <= 3;
            const bool b_link1 = b >= 1 && b <= 3;
            const bool a_link2 = a >= 5 && a <= 7;
            const bool b_link2 = b >= 5 && b <= 7;
            if ((a_link1 && b_link2) || (a_link2 && b_link1))
                return true;
        }
        return false;
    }
}  // namespace

int main()
{
    const std::vector<double> neutral{0.0, 0.0, 0.0};
    const std::vector<double> folded{0.0, 2.7, 0.0};

    auto robot = loadPlanar3(Eigen::Vector3d::Zero());
    if (!expectFamily(*robot, comotion::RobotModel::RobotFamily::Planar3))
        return 1;

    if (!expectEqual("adjacent link1/joint1 disabled",
                     robot->isSelfCollisionDisabled("link1_cylinder",
                                                    "joint1_sphere"),
                     true))
        return 1;
    if (!expectEqual("link1/link2 cylinder collision enabled",
                     robot->isSelfCollisionDisabled("link1_cylinder",
                                                    "link2_cylinder"),
                     false))
        return 1;

    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Vamp);
    if (!expectEqual("neutral self valid",
                     checker.isSelfCollisionFree(*robot, neutral), true))
        return 1;
    if (!expectEqual("folded self invalid",
                     checker.isSelfCollisionFree(*robot, folded), false))
        return 1;
    if (!expectEqual("folded link1/link2 cylinder collision",
                     hasLink1Link2CylinderCollision(folded), true))
        return 1;

    auto robot_a = loadPlanar3(Eigen::Vector3d(-1.25, 0.0, 0.0));
    auto robot_b = loadPlanar3(Eigen::Vector3d(1.25, 0.0, 0.0));
    if (!expectEqual("far pair valid",
                     checker.isValidPair(*robot_a, neutral, *robot_b, neutral),
                     true))
        return 1;

    robot_b->setBaseTransform(Eigen::Affine3d::Identity());
    if (!expectEqual("same-base pair invalid",
                     checker.isValidPair(*robot, neutral, *robot_b, neutral),
                     false))
        return 1;

    comotion::ObstacleSphere far_obstacle{{10.0, 0.0, 0.0}, 0.1};
    checker.setObstacles({far_obstacle});
    if (!expectEqual("far obstacle valid",
                     checker.isValidSingle(*robot, neutral), true))
        return 1;

    comotion::ObstacleSphere shaft_obstacle{{0.15, 0.0, 0.0}, 0.04};
    checker.setObstacles({shaft_obstacle});
    if (!expectEqual("shaft obstacle invalid",
                     checker.isValidSingle(*robot, neutral), false))
        return 1;

    std::cout << "vamp_planar3_regression: OK\n";
    return 0;
}
