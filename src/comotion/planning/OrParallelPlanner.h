#pragma once

#include "comotion/planning/MultiRobotPlanner.h"

#include <functional>
#include <memory>
#include <string>

namespace comotion {

class OrParallelPlanner : public MultiRobotPlanner {
public:
    using PlannerFactory = std::function<std::shared_ptr<MultiRobotPlanner>()>;

    void setPlannerFactory(PlannerFactory factory) {
        planner_factory_ = std::move(factory);
    }

    void setBasePlannerName(std::string name) {
        base_planner_name_ = std::move(name);
    }

    void setWorkerProcesses(unsigned n) { worker_processes_ = n; }
    void setReturnApproximateIfNoExact(bool v) {
        return_approximate_if_no_exact_ = v;
    }

    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::vector<Path> getSolutionPaths() const override { return solution_paths_; }
    std::string name() const override;

private:
    PlannerFactory planner_factory_;
    std::string base_planner_name_ = "unknown";
    std::vector<Path> solution_paths_;
    unsigned worker_processes_ = 1;
    bool return_approximate_if_no_exact_ = true;
};

} // namespace comotion
