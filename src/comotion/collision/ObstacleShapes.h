#pragma once

#include <Eigen/Core>

namespace comotion {

struct ObstacleSphere {
    Eigen::Vector3d center;
    double radius;
};

// Finite cylinder obstacle. `center` is the midpoint of the cylinder axis,
// `axis` is a unit vector along the cylinder axis, `radius` is the circular
// cross-section radius, and `half_height` is half the total cylinder height.
struct ObstacleCylinder {
    Eigen::Vector3d center;
    Eigen::Vector3d axis;
    double radius;
    double half_height;
};

} // namespace comotion
