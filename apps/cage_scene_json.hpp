#pragma once

#include "comotion/collision/ObstacleShapes.h"

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace cage_scene {

using Affine3dVector =
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>;

// Parse robot_bases[]: required "position" [x,y,z]; optional "quaternion_xyzw"
// [qx,qy,qz,qw] matching the benchmark app JSON output.
inline Affine3dVector
parseRobotBasesArray(const nlohmann::json &bases_json) {
    Affine3dVector out;
    out.reserve(bases_json.size());
    for (const auto &entry : bases_json) {
        const auto &pj = entry.at("position");
        Eigen::Vector3d pos(pj.at(0).get<double>(), pj.at(1).get<double>(),
                            pj.at(2).get<double>());
        Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
        if (entry.contains("quaternion_xyzw")) {
            const auto &qq = entry.at("quaternion_xyzw");
            q = Eigen::Quaterniond(qq.at(3).get<double>(), qq.at(0).get<double>(),
                                   qq.at(1).get<double>(), qq.at(2).get<double>());
            if (q.squaredNorm() < 1e-12)
                throw std::runtime_error("parseRobotBasesArray: invalid quaternion (near zero norm)");
            q.normalize();
        }
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.translation() = pos;
        T.linear() = q.toRotationMatrix();
        out.push_back(T);
    }
    return out;
}

// Parse doc["obstacles"] for type "cylinder" only; ignores spheres and unknown types.
inline std::vector<comotion::ObstacleCylinder>
parseCylinderObstacles(const nlohmann::json &doc) {
    std::vector<comotion::ObstacleCylinder> cyls;
    if (!doc.contains("obstacles"))
        return cyls;
    for (const auto &obs : doc.at("obstacles")) {
        std::string type = obs.at("type").get<std::string>();
        if (type != "cylinder")
            continue;
        comotion::ObstacleCylinder c;
        const auto &ctr = obs.at("center");
        const auto &axis = obs.at("axis");
        c.center = Eigen::Vector3d(ctr.at(0).get<double>(), ctr.at(1).get<double>(),
                                   ctr.at(2).get<double>());
        c.axis = Eigen::Vector3d(axis.at(0).get<double>(), axis.at(1).get<double>(),
                                 axis.at(2).get<double>())
                     .normalized();
        c.radius = obs.at("radius").get<double>();
        c.half_height = obs.at("half_height").get<double>();
        cyls.push_back(c);
    }
    return cyls;
}

} // namespace cage_scene
