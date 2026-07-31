#pragma once

#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/PathSimplification.h"
#include <optional>
#include <vector>

namespace comotion {

/// Baseline planner API.
///
/// Composite RRT (RRT-C): treats all robots as one composite system,
/// concatenating their joint spaces and using RRTConnect.
class CompositeRRT : public MultiRobotPlanner {
public:
    // If robot_indices is empty, plans for all robots.
    void setRobotIndices(const std::vector<int> &indices) {
        robot_indices_ = indices;
    }

    void setSimplifySolution(bool simplify) { simplify_solution_ = simplify; }
    bool getSimplifySolution() const { return simplify_solution_; }
    void setPathSimplificationOptions(PathSimplificationOptions options) {
        simplification_options_ =
            detail::normalizePathSimplificationOptions(options);
    }
    PathSimplificationOptions getPathSimplificationOptions() const {
        return simplification_options_;
    }

    /// Upper bound on RRTConnect outer-loop iterations (each iteration draws a
    /// random sample state). Zero disables the cap so planning is time-limited only.
    void setMaxRrtConnectIterations(unsigned n) { max_rrt_connect_iterations_ = n; }
    unsigned getMaxRrtConnectIterations() const { return max_rrt_connect_iterations_; }

    /// If enabled with a nonzero iteration cap, continue expanding RRTConnect
    /// after the first exact solution until the cap is exhausted. This is useful
    /// for validation-corpus benchmarks that need a fixed amount of planner work.
    void setContinueAfterSolutionUntilIterationCap(bool value) {
        continue_after_solution_until_iteration_cap_ = value;
    }
    bool getContinueAfterSolutionUntilIterationCap() const {
        return continue_after_solution_until_iteration_cap_;
    }

    /// Maximum RRTConnect extension length. If unset or non-positive, OMPL
    /// auto-configures the range from the state-space extent.
    void setRange(double distance) { range_ = distance; }
    std::optional<double> getRange() const { return range_; }

    /// Use the synchronized multi-robot makespan metric for composite
    /// nearest-neighbor distance, extension length, and auto range.
    void setUseMakespanMetric(bool value) { use_makespan_metric_ = value; }
    bool getUseMakespanMetric() const { return use_makespan_metric_; }

    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::vector<Path> getSolutionPaths() const override;
    std::string name() const override { return "CompositeRRT"; }

private:
    std::vector<int> robot_indices_;
    std::vector<Path> solution_paths_;
    bool simplify_solution_{true};
    PathSimplificationOptions simplification_options_{};
    bool use_makespan_metric_{false};
    bool continue_after_solution_until_iteration_cap_{false};
    unsigned max_rrt_connect_iterations_{0};
    std::optional<double> range_;
};

} // namespace comotion
