#pragma once

#include "comotion/planning/MultiRobotProblem.h"

#include <memory>
#include <vector>

namespace comotion {

struct RobotCspaceRegion {
    int global_robot_index = -1;
    // Lower and upper bounds for each robot configuration coordinate.
    std::vector<double> lower;
    std::vector<double> upper;
};

struct MRMPSubproblem {
    std::shared_ptr<MultiRobotProblem> problem;
    std::vector<int> global_robot_indices;
    int window_start_t = 0;
    int window_end_t = 0;
    int global_end_t = 0;
    std::vector<RobotCspaceRegion> cspace_regions;
    bool uses_global_cspace = false;

    bool spansGlobalTime() const noexcept {
        return window_start_t == 0 && window_end_t >= global_end_t;
    }
};

} // namespace comotion