#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/Path.h"
#include "comotion/robot/RobotModel.h"

#include <Eigen/Geometry>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

std::string getResourcePath(const std::string &relative) {
    const char *prefixes[] = {"../resources/", "../../resources/",
                              "../../../resources/", "resources/"};
    for (const char *pfx : prefixes) {
        std::string path = std::string(pfx) + relative;
        std::ifstream f(path);
        if (f.good())
            return path;
    }
    return std::string("resources/") + relative;
}

std::shared_ptr<comotion::RobotModel> loadPanda(const std::string &urdf,
                                            const std::string &srdf,
                                            const Eigen::Vector3d &base_xyz) {
    auto robot = std::make_shared<comotion::RobotModel>();
    robot->loadURDF(urdf);
    robot->loadSRDF(srdf);
    Eigen::Affine3d base = Eigen::Affine3d::Identity();
    base.translation() = base_xyz;
    robot->setBaseTransform(base);
    return robot;
}

bool expectEqual(const std::string &label, bool actual, bool expected) {
    if (actual != expected) {
        std::cerr << "vamp_panda_cage_regression: " << label << " expected "
                  << expected << " got " << actual << "\n";
        return false;
    }
    return true;
}

} // namespace

int main() {
    const std::string urdf_sph = getResourcePath("panda/panda_spherized.urdf");
    const std::string srdf = getResourcePath("panda/panda.srdf");
    const std::vector<double> zero_cfg(7, 0.0);
    const std::vector<double> cage_valid_cfg = {
        0.23035124432833062,
        1.5818255578708462,
        -2.012433195110716,
        -0.9717805044903518,
        -1.3048756193872513,
        0.13839312320230085,
        0.6559295503425098,
    };
    const Eigen::Vector3d cage_base(-0.75, -0.5, -1.0);

    auto panda_vamp = loadPanda(urdf_sph, srdf, cage_base);
    auto panda_fcl = loadPanda(urdf_sph, srdf, cage_base);

    comotion::ObstacleCylinder far_away{{10.0, 0.0, 0.4},
                                    Eigen::Vector3d::UnitZ(), 0.2, 0.3};

    comotion::CollisionChecker vamp_checker(comotion::CollisionChecker::Backend::Vamp);
    comotion::CollisionChecker fcl_checker(comotion::CollisionChecker::Backend::Fcl);

    if (!expectEqual("baseline valid vamp",
                     vamp_checker.isValidSingle(*panda_vamp, cage_valid_cfg),
                     true) ||
        !expectEqual("baseline valid fcl",
                     fcl_checker.isValidSingle(*panda_fcl, cage_valid_cfg),
                     true)) {
        return 1;
    }

    vamp_checker.setCylinderObstacles({far_away});
    fcl_checker.setCylinderObstacles({far_away});
    if (!expectEqual("far cylinder vamp",
                     vamp_checker.isValidSingle(*panda_vamp, cage_valid_cfg),
                     true) ||
        !expectEqual("far cylinder fcl",
                     fcl_checker.isValidSingle(*panda_fcl, cage_valid_cfg),
                     true)) {
        return 1;
    }

    std::optional<comotion::ObstacleCylinder> colliding_cylinder;
    for (const auto &sphere : panda_vamp->getCollisionSpheres(cage_valid_cfg)) {
        comotion::ObstacleCylinder probe{sphere.center,
                                     Eigen::Vector3d::UnitZ(),
                                     std::max(0.02, sphere.radius * 0.75),
                                     std::max(0.04, sphere.radius)};
        fcl_checker.setCylinderObstacles({probe});
        if (!fcl_checker.isValidSingle(*panda_fcl, cage_valid_cfg)) {
            colliding_cylinder = probe;
            break;
        }
    }
    if (!colliding_cylinder) {
        std::cerr << "vamp_panda_cage_regression: could not find a cylinder placement"
                     " that collides with the Panda sphere model\n";
        return 1;
    }

    vamp_checker.setCylinderObstacles({*colliding_cylinder});
    fcl_checker.setCylinderObstacles({*colliding_cylinder});
    if (!expectEqual("collision cylinder vamp",
                     vamp_checker.isValidSingle(*panda_vamp, cage_valid_cfg),
                     false) ||
        !expectEqual("collision cylinder fcl",
                     fcl_checker.isValidSingle(*panda_fcl, cage_valid_cfg),
                     false)) {
        return 1;
    }

    auto panda_a = loadPanda(urdf_sph, srdf, Eigen::Vector3d::Zero());
    auto panda_b = loadPanda(urdf_sph, srdf, Eigen::Vector3d::Zero());
    comotion::CollisionChecker pair_checker(comotion::CollisionChecker::Backend::Vamp);

    if (!expectEqual("overlap pair invalid",
                     pair_checker.isValidPair(*panda_a, zero_cfg, *panda_b, zero_cfg),
                     false)) {
        return 1;
    }

    Eigen::Affine3d shifted = Eigen::Affine3d::Identity();
    shifted.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
    panda_b->setBaseTransform(shifted);
    if (!expectEqual("shifted pair valid",
                     pair_checker.isValidPair(*panda_a, zero_cfg, *panda_b, zero_cfg),
                     true)) {
        return 1;
    }

    comotion::Path path_a;
    comotion::Path path_b;
    path_a.push_back(zero_cfg);
    path_a.push_back(zero_cfg);
    path_b.push_back(zero_cfg);
    path_b.push_back(zero_cfg);

    panda_b->setBaseTransform(Eigen::Affine3d::Identity());
    auto overlap_conflict = pair_checker.findFirstPairPathConflict(
        *panda_a, path_a, *panda_b, path_b);
    if (!overlap_conflict || overlap_conflict->timestep != 0) {
        std::cerr << "vamp_panda_cage_regression: expected overlap conflict at timestep 0\n";
        return 1;
    }

    panda_b->setBaseTransform(shifted);
    auto shifted_conflict = pair_checker.findFirstPairPathConflict(
        *panda_a, path_a, *panda_b, path_b);
    if (shifted_conflict) {
        std::cerr << "vamp_panda_cage_regression: unexpected conflict after base shift\n";
        return 1;
    }

    std::cout << "vamp_panda_cage_regression: OK\n";
    return 0;
}
