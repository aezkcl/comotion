#pragma once

#include "comotion/planning/MultiRobotPlanner.h"
#include "comotion/planning/USTRRTstar.h"

#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace comotion {

struct STCBSConflict {
    int robot_i = -1;
    int robot_j = -1;
    int timestep = 0;
    int motion_index_i = -1;
    int motion_index_j = -1;
    std::vector<double> config_i;
    std::vector<double> config_j;
};

struct STCBSNode {
    std::vector<std::shared_ptr<USTRRTstar::TreeSnapshot>> trees;
    std::vector<USTRRTstar::Result> results;
    double st_cost = std::numeric_limits<double>::infinity();
};

/// Experimental/advanced planner API.
///
/// Space-time CBS planner using USTRRTstar as its branch-level search machinery.
class STCBS : public MultiRobotPlanner {
public:
    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::vector<Path> getSolutionPaths() const override;
    std::string name() const override { return "STCBS"; }

    void setRange(double v) { range_ = v; }
    void setMaxIterations(int v) { max_iterations_ = v; }
    void setMaxSamples(int v) { max_samples_ = v; }
    void setGoalThreshold(double v) { goal_threshold_ = v; }
    void setLayerDtSeconds(double v) { layer_dt_seconds_ = v; }
    void setRewireMode(USTRRTstar::RewireMode mode) { rewire_mode_ = mode; }
    void setRewireRadius(double v) { rewire_radius_ = v; }
    void setRewireK(int v) { rewire_k_ = v; }
    void setLambda(double v) { lambda_ = v; }
    void setMaxCTNodes(int v) { max_ct_nodes_ = v; }
    void setOccupiedRadius(double v) { occupied_radius_ = v; }
    void setGoalHoldEnabled(bool v) { goal_hold_enabled_ = v; }

private:
    USTRRTstar::Params makeParams() const;
    std::optional<STCBSConflict> getFirstConflict(
        const STCBSNode &node) const;
    double computeTotalSTCost(const STCBSNode &node) const;
    bool applyBranchGoalHold(USTRRTstar::TreeSnapshot &tree,
                             int constrained_robot, int other_robot,
                             const Path &other_path) const;

    std::vector<Path> solution_paths_;
    std::uint64_t conflict_count_ = 0;
    std::uint64_t ct_nodes_expanded_ = 0;
    std::uint64_t ust_rrt_calls_total_ = 0;
    std::uint64_t ust_rrt_calls_successes_ = 0;
    std::vector<double> ust_rrt_solve_times_seconds_;

    double range_ = 10.0;
    int max_iterations_ = 1000;
    int max_samples_ = 100000;
    double goal_threshold_ = 0.1;
    double layer_dt_seconds_ = 1.0;
    USTRRTstar::RewireMode rewire_mode_ =
        USTRRTstar::RewireMode::KNearest;
    double rewire_radius_ = 1.0;
    int rewire_k_ = 10;

    double lambda_ = 0.5;
    int max_ct_nodes_ = 10000;
    double occupied_radius_ = 0.1;
    bool goal_hold_enabled_ = true;
};

} // namespace comotion
