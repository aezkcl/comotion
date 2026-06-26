#pragma once

#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/Path.h"
#include <nlohmann/json.hpp>
#include <ompl/base/PlannerStatus.h>
#include <ompl/geometric/PathGeometric.h>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace comotion {

class MultiRobotPlanner {
public:
    struct SolutionMetrics {
        std::optional<std::uint64_t> sum_of_cost_timesteps;
        std::optional<std::uint64_t> makespan_timesteps;
    };

    virtual ~MultiRobotPlanner() = default;

    void setProblem(std::shared_ptr<MultiRobotProblem> problem) {
        problem_ = std::move(problem);
    }

    virtual ompl::base::PlannerStatus solve(double timeLimit) = 0;

    virtual std::vector<Path> getSolutionPaths() const = 0;

    virtual std::string name() const = 0;

    /// User-facing planning seed (same value as CLI `--seed` in the apps). Used to derive
    /// explicit local RNG streams (e.g. per-agent `ompl::RNG` seeds). Do not call
    /// `ompl::RNG::setSeed` from `solve()`; the trial driver must seed OMPL once before
    /// planning via `comotion::seedOmplGlobalFromUserPlanningSeed` (see `PlanningRng.h`).
    virtual void setPlanningSeed(std::uint32_t seed) { planning_seed_ = seed; }
    virtual void
    setCancellationCallback(std::function<bool()> cancel_requested) {
        cancel_requested_ = std::move(cancel_requested);
    }

    std::uint32_t planningSeed() const { return planning_seed_; }
    const SolutionMetrics &solutionMetrics() const { return solution_metrics_; }
    const std::optional<std::uint64_t> &sumOfCostTimesteps() const {
        return solution_metrics_.sum_of_cost_timesteps;
    }
    const std::optional<std::uint64_t> &makespanTimesteps() const {
        return solution_metrics_.makespan_timesteps;
    }
    const nlohmann::json &plannerStatsJson() const { return planner_stats_json_; }

protected:
    std::shared_ptr<MultiRobotProblem> problem_;
    std::uint32_t planning_seed_ = 42;
    std::function<bool()> cancel_requested_;
    SolutionMetrics solution_metrics_{};
    nlohmann::json planner_stats_json_ = nlohmann::json::object();

    void resetPlannerRunMetrics();
    bool cancellationRequested() const {
        return cancel_requested_ && cancel_requested_();
    }
    void clearSolutionMetrics();
    void setSolutionMetrics(std::uint64_t sum_of_cost_timesteps,
                            std::uint64_t makespan_timesteps);
    void setSolutionMetricsFromPaths(const std::vector<Path> &paths);
    void setPlannerStatsJson(nlohmann::json stats);
    static std::pair<std::uint64_t, std::uint64_t>
    computeSolutionMetrics(const std::vector<Path> &paths);

    // Split a composite OMPL path into synchronized per-robot paths. The
    // returned paths are interpolated so index k corresponds to timestep k.
    static std::vector<Path> splitCompositePathToRobotPaths(
        const ompl::geometric::PathGeometric &path,
        const MultiRobotProblem &problem, const std::vector<int> &robot_indices,
        const std::string &planner_label);

    // Helper: extract a config vector from an OMPL state
    static std::vector<double> stateToConfig(
        const ompl::base::State *state, int ndof) {
        auto *rv = state->as<ompl::base::RealVectorStateSpace::StateType>();
        std::vector<double> config(ndof);
        for (int i = 0; i < ndof; ++i)
            config[i] = rv->values[i];
        return config;
    }

    // Helper: set an OMPL state from a config vector
    static void configToState(ompl::base::State *state,
                              const std::vector<double> &config) {
        auto *rv = state->as<ompl::base::RealVectorStateSpace::StateType>();
        for (size_t i = 0; i < config.size(); ++i)
            rv->values[i] = config[i];
    }

    // Helper: convert an OMPL geometric path to a Path with timesteps (max velocity per edge)
    static Path omplPathToPath(const ompl::geometric::PathGeometric &gpath,
                               int ndof, size_t resolution, double vmax);
};

} // namespace comotion
