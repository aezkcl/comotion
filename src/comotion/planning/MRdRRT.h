#pragma once

#include "comotion/planning/MultiRobotPlanner.h"
#include <ompl/util/RandomNumbers.h>
#include <algorithm>
#include <vector>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <chrono>

namespace comotion {

/// Baseline planner API.
///
/// Section 4 dRRT (Solovey et al.): discrete RRT on the implicit tensor product
/// of per-robot PRM roadmaps (Algorithms 1-2 in the dRRT* manuscript, Section 4).
/// Phase 1: build PRM roadmaps per robot until each reaches roadmap_size_
/// vertices (see setRoadmapSize) or, for randomized dRRT when roadmaps are
/// charged to the budget, the overall solve() wall-time deadline.
/// Phase 2: repeat nit x Expand, then ConnectToTarget, until the same deadline
/// or success. In A* tensor-search mode, Phase 2 is intentionally unbounded and
/// runs until the tensor graph is solved/exhausted or external cancellation fires.
class MRdRRT : public MultiRobotPlanner {
public:
    enum class CostMetric {
        SumOfCosts,
        CompositeL2,
        Makespan,
    };

    enum class TensorSearchMode {
        Drrt,
        AStar,
        LazyAStar,
    };

    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::vector<Path> getSolutionPaths() const override;
    std::string name() const override { return "MRdRRT"; }

    /// Target PRM milestone count per robot (including start/goal); phase 1 stops when reached.
    void setRoadmapSize(int n) { roadmap_size_ = std::max(2, n); }
    /// Unused: PRM* manages its own connection strategy (reserved for API stability).
    void setConnectionRadius(double r) { connection_radius_ = r; }
    /// nit in Algorithm 1: Expand calls per ConnectToTarget check (default 1 = paper).
    void setIterationsPerBatch(int n) { iterations_per_batch_ = std::max(1, n); }
    void setCostMetric(CostMetric metric) { cost_metric_ = metric; }
    CostMetric costMetric() const { return cost_metric_; }
    void setTensorSearchMode(TensorSearchMode mode) { tensor_search_mode_ = mode; }
    TensorSearchMode tensorSearchMode() const { return tensor_search_mode_; }
    void setStopAtFirstSolution(bool value) { stop_at_first_solution_ = value; }
    bool stopAtFirstSolution() const { return stop_at_first_solution_; }
    void setUseStarRewiring(bool value) { use_star_rewiring_ = value; }
    bool useStarRewiring() const { return use_star_rewiring_; }
    /// If true, PRM* roadmap construction is not charged to solve(timeLimit).
    /// Roadmaps are built to roadmap_size_ first, then tensor dRRT gets the full budget.
    void setExcludeRoadmapBuildTimeFromBudget(bool value) {
        exclude_roadmap_build_time_from_budget_ = value;
    }
    bool excludeRoadmapBuildTimeFromBudget() const {
        return exclude_roadmap_build_time_from_budget_;
    }

    static const char *costMetricName(CostMetric metric);
    static const char *tensorSearchModeName(TensorSearchMode mode);

private:
    /// Single-robot roadmap from OMPL PRM* getPlannerData: vertex 0 = query start,
    /// vertex 1 = query goal (see PRM::getPlannerData addStartVertex / addGoalVertex order).
    struct Roadmap {
        std::vector<std::vector<double>> vertices;
        std::vector<std::vector<int>> adjacency;
        int start_vertex = -1;
        int goal_vertex = -1;
    };

    struct CompositeVertex {
        std::vector<int> vertex_ids;
        bool operator==(const CompositeVertex &o) const {
            return vertex_ids == o.vertex_ids;
        }
    };

    struct CompositeVertexHash {
        size_t operator()(const CompositeVertex &v) const {
            size_t seed = 0;
            for (int id : v.vertex_ids)
                seed ^= std::hash<int>()(id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    struct TreeNode {
        CompositeVertex vertex;
        int parent = -1;
        std::vector<int> children;
        double cost = 0.0;
    };

    void buildRoadmaps(
        std::optional<std::chrono::steady_clock::time_point> solve_deadline);

    CompositeVertex directionOracle(
        const CompositeVertex &v_near,
        const std::vector<std::vector<double>> &q_rand) const;

    double compositeDistance(const CompositeVertex &a,
                             const std::vector<std::vector<double>> &target) const;

    double compositeDistance(const CompositeVertex &a,
                             const CompositeVertex &b) const;

    double localDistance(
        const CompositeVertex &a,
        const std::vector<std::vector<double>> &target) const;

    double edgeCost(const CompositeVertex &from,
                    const CompositeVertex &to) const;

    double localEdgeCost(
        const CompositeVertex &from,
        const std::vector<std::vector<double>> &target) const;

    bool areTensorAdjacent(const CompositeVertex &a,
                           const CompositeVertex &b) const;

    bool wouldCreateCycle(int node_index, int candidate_parent) const;

    void removeChild(int parent_index, int child_index);
    void addChild(int parent_index, int child_index);
    void propagateCostDelta(int node_index, double delta);

    int chooseBestParent(const CompositeVertex &v_new,
                         int fallback_parent,
                         double fallback_edge_cost) const;

    int rewireFrom(int added_index);

    std::vector<Path> pathsFromCompositePath(
        const std::vector<CompositeVertex> &cv_path) const;

    bool isCompositeEdgeValid(const CompositeVertex &from,
                              const CompositeVertex &to) const;

    /// Straight-line motion in joint space per robot from `from` to `goal_configs`;
    /// discretization matches `isCompositeEdgeValid`; uses full composite collision.
    bool localConnector(
        const CompositeVertex &from,
        const std::vector<std::vector<double>> &goal_configs,
        std::chrono::steady_clock::time_point deadline) const;

    /// k = max(1, floor(log2(max(1, total_iteration)))), capped by tree size;
    /// try local composite bridge from k nearest tree nodes to goal. Returns
    /// start → … → anchor → goal_v (goal appended) or nullopt.
    std::optional<std::vector<CompositeVertex>> connectToTarget(
        const CompositeVertex &goal_v,
        const std::unordered_map<CompositeVertex, int, CompositeVertexHash>
            &vertex_to_node,
        uint64_t total_iteration,
        std::chrono::steady_clock::time_point deadline,
        double *path_cost = nullptr) const;

    std::vector<CompositeVertex> traceTreePathToRoot(
        int node_index) const;

    int findNearestNode(const std::vector<std::vector<double>> &q_rand) const;

    std::vector<std::vector<double>> computeIndividualGoalDistances() const;

    double astarHeuristic(
        const CompositeVertex &v,
        const std::vector<std::vector<double>> &individual_goal_distances) const;

    ompl::base::PlannerStatus solveTensorAStar(
        const CompositeVertex &start_v, const CompositeVertex &goal_v,
        std::chrono::steady_clock::time_point accounting_start_time,
        double &best_solution_cost, nlohmann::json &solution_events,
        nlohmann::json &astar_stats);

    ompl::base::PlannerStatus solveTensorLazyAStar(
        const CompositeVertex &start_v, const CompositeVertex &goal_v,
        std::chrono::steady_clock::time_point accounting_start_time,
        double &best_solution_cost, nlohmann::json &solution_events,
        nlohmann::json &lazy_astar_stats);

    int roadmap_size_ = 150;
    double connection_radius_ = 2.0;
    int iterations_per_batch_ = 1;
    CostMetric cost_metric_ = CostMetric::SumOfCosts;
    TensorSearchMode tensor_search_mode_ = TensorSearchMode::Drrt;
    bool stop_at_first_solution_ = true;
    bool use_star_rewiring_ = false;
    bool exclude_roadmap_build_time_from_budget_ = false;

    std::vector<Roadmap> roadmaps_;
    std::vector<TreeNode> tree_;
    std::vector<Path> solution_paths_;

    /// 1% of single-robot extent; set once per solve() for edge / localConnector checks.
    mutable double longest_valid_segment_cache_{0.0};

    /// Composite anchors that already failed `localConnector` to the current goal this solve;
    /// skipped in later connectToTarget calls to avoid repeating the same collision checks.
    mutable std::unordered_set<CompositeVertex, CompositeVertexHash> local_connect_failed_;

    /// Phase-2 tensor RRT composite configuration sampling (isolated from PRM* draw count).
    mutable ompl::RNG tensor_phase_rng_;
};

/// Baseline planner API.
///
/// Anytime dRRT* variant: keeps searching after the first exact connection,
/// uses best-parent/rewire updates on tensor-adjacent tree vertices, and reports
/// the best path under the selected CostMetric.
class MRdRRTStar : public MRdRRT {
public:
    MRdRRTStar();
    std::string name() const override { return "MRdRRTStar"; }
};

} // namespace comotion
