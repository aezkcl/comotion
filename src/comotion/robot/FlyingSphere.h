#pragma once

#include "comotion/robot/RobotModel.h"
#include <vector>

namespace comotion {

// A 3-DOF flying unit sphere robot (x, y, z position).
// Config is {x, y, z}. A single collision sphere of the given radius.
// Per-axis workspace bounds (joint 0 = x, 1 = y, 2 = z); vectors must have size 3.
class FlyingSphere : public RobotModel {
public:
    FlyingSphere(double radius,
                 const std::vector<double> &workspace_min_xyz,
                 const std::vector<double> &workspace_max_xyz);

    /// Same lower/upper bound on x, y, and z (legacy / quick tests).
    FlyingSphere(double radius = 1.0,
                 double workspace_low = -10.0,
                 double workspace_high = 10.0);
};

} // namespace comotion
