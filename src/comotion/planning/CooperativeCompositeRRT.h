#pragma once

#include "comotion/planning/MultiRobotPlanner.h"

#include <vector>

namespace comotion {

// Shared-tree bidirectional composite RRTConnect. Multiple worker threads
// cooperatively grow the same start and goal trees.
class CooperativeCompositeRRT : public MultiRobotPlanner {
public:
    void setRobotIndices(const std::vector<int> &indices) {
        robot_indices_ = indices;
    }

    void setWorkerThreads(unsigned n) { worker_threads_ = n == 0 ? 1 : n; }
    unsigned getWorkerThreads() const { return worker_threads_; }

    void setSimplifySolution(bool simplify) { simplify_solution_ = simplify; }
    bool getSimplifySolution() const { return simplify_solution_; }

    void setMaxRrtConnectIterations(unsigned n) {
        max_rrt_connect_iterations_ = n;
    }
    unsigned getMaxRrtConnectIterations() const {
        return max_rrt_connect_iterations_;
    }

    void setRange(double distance) { range_ = distance; }
    double getRange() const { return range_; }

    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::vector<Path> getSolutionPaths() const override {
        return solution_paths_;
    }
    std::string name() const override { return "CooperativeCompositeRRT"; }

private:
    std::vector<int> robot_indices_;
    std::vector<Path> solution_paths_;
    unsigned worker_threads_{2};
    bool simplify_solution_{true};
    unsigned max_rrt_connect_iterations_{0};
    double range_{0.0};
};

} // namespace comotion
