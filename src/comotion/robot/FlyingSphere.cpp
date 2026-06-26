#include "comotion/robot/FlyingSphere.h"
#include <algorithm>
#include <stdexcept>

namespace comotion {

FlyingSphere::FlyingSphere(double radius, double workspace_low,
                           double workspace_high)
    : FlyingSphere(radius,
                   std::vector<double>{workspace_low, workspace_low, workspace_low},
                   std::vector<double>{workspace_high, workspace_high, workspace_high}) {
}

FlyingSphere::FlyingSphere(double radius,
                           const std::vector<double> &workspace_min_xyz,
                           const std::vector<double> &workspace_max_xyz) {
    setRobotFamily(RobotFamily::Sphere);

    if (workspace_min_xyz.size() < 3 || workspace_max_xyz.size() < 3)
        throw std::invalid_argument(
            "FlyingSphere: workspace_min_xyz and workspace_max_xyz must have size >= 3");

    LinkInfo base_link;
    base_link.name = "sphere_link";
    base_link.index = 0;

    CollisionSphere s;
    s.center = Eigen::Vector3d::Zero();
    s.radius = radius;
    s.link_index = 0;
    base_link.collision_spheres.push_back(s);

    links_.push_back(base_link);
    link_name_to_index_["sphere_link"] = 0;

    auto addPrismatic = [&](const std::string &name, const std::string &parent,
                            const std::string &child, const Eigen::Vector3d &axis,
                            int axis_idx) {
        LinkInfo cl;
        cl.name = child;
        cl.index = static_cast<int>(links_.size());
        link_name_to_index_[child] = cl.index;
        links_.push_back(cl);

        double lo = workspace_min_xyz[static_cast<size_t>(axis_idx)];
        double hi = workspace_max_xyz[static_cast<size_t>(axis_idx)];
        if (lo > hi)
            std::swap(lo, hi);

        JointInfo j;
        j.name = name;
        j.type = "prismatic";
        j.parent_link = parent;
        j.child_link = child;
        j.origin = Eigen::Affine3d::Identity();
        j.axis = axis;
        j.lower_limit = lo;
        j.upper_limit = hi;
        j.velocity_limit = 10.0;
        joints_.push_back(j);
    };

    addPrismatic("joint_x", "sphere_link", "link_x", Eigen::Vector3d::UnitX(), 0);
    addPrismatic("joint_y", "link_x", "link_y", Eigen::Vector3d::UnitY(), 1);
    addPrismatic("joint_z", "link_y", "link_z", Eigen::Vector3d::UnitZ(), 2);

    // Move collision sphere to the end link
    links_[0].collision_spheres.clear();
    auto &end_link = links_.back();
    end_link.collision_spheres.push_back(s);
    end_link.collision_spheres.back().link_index = end_link.index;

    buildKinematicChain();
}

} // namespace comotion
