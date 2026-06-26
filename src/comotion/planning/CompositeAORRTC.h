#pragma once

#include "comotion/planning/AORRTCUtils.h"
#include "comotion/planning/MultiRobotPlanner.h"

#include <cstddef>
#include <vector>

namespace comotion {

/// Experimental/advanced planner API.
///
/// Composite bounded/anytime RRTConnect helper exposed for AO-ARC experiments.
class CompositeAORRTC : public MultiRobotPlanner {
public:
    void setRobotIndices(const std::vector<int> &indices) {
        robot_indices_ = indices;
    }

    void setSimplifySolution(bool simplify) { simplify_solution_ = simplify; }
    bool getSimplifySolution() const { return simplify_solution_; }
    void setMaxInternalSamples(std::size_t value) {
        max_internal_samples_ = value;
    }
    void setMaxInternalVertices(std::size_t value) {
        max_internal_vertices_ = value;
    }

    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::vector<Path> getSolutionPaths() const override;
    std::string name() const override { return "CompositeAORRTC"; }

private:
    static nlohmann::json
    solutionEventsJson(const std::vector<aorrtc::SolutionEvent> &events);

    std::vector<int> robot_indices_;
    std::vector<Path> solution_paths_;
    std::vector<aorrtc::SolutionEvent> solution_events_;
    bool simplify_solution_{true};
    std::size_t max_internal_samples_{10000};
    std::size_t max_internal_vertices_{10000};
};

} // namespace comotion
