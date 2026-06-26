#pragma once

#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/PathSimplification.h"

#include <algorithm>
#include <vector>

namespace comotion {

/// Baseline planner API.
///
/// Composite PRM* treats all robots as one composite system and searches the
/// resulting roadmap in the selected composite metric.
class CompositePRMStar : public MultiRobotPlanner {
public:
    enum class MetricMode {
        Makespan,
        PlainL2,
    };

    void setRobotIndices(const std::vector<int> &indices) {
        robot_indices_ = indices;
    }

    void setMetricMode(MetricMode mode) { metric_mode_ = mode; }
    MetricMode metricMode() const { return metric_mode_; }

    void setSimplifySolution(bool simplify) { simplify_solution_ = simplify; }
    bool getSimplifySolution() const { return simplify_solution_; }

    void setPathSimplificationOptions(PathSimplificationOptions options) {
        simplification_options_ =
            detail::normalizePathSimplificationOptions(options);
    }
    PathSimplificationOptions getPathSimplificationOptions() const {
        return simplification_options_;
    }
    void setSimplificationMaxSteps(unsigned int max_steps) {
        simplification_options_.max_shortcut_steps = std::max(1u, max_steps);
    }
    unsigned int getSimplificationMaxSteps() const {
        return simplification_options_.max_shortcut_steps;
    }

    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::vector<Path> getSolutionPaths() const override;
    std::string name() const override { return "CompositePRMStar"; }

private:
    std::vector<int> robot_indices_;
    std::vector<Path> solution_paths_;
    MetricMode metric_mode_{MetricMode::Makespan};
    bool simplify_solution_{true};
    PathSimplificationOptions simplification_options_{};
};

const char *toString(CompositePRMStar::MetricMode mode);

} // namespace comotion
