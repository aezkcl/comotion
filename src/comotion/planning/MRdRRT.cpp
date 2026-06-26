#include "comotion/planning/MRdRRT.h"
#include "comotion/planning/PlanningSeed.h"
#include "comotion/planning/Path.h"
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/PlannerData.h>
#include <algorithm>
#include <unordered_map>
#include <chrono>
#include <cmath>
#include <optional>
#include <functional>
#include <queue>

namespace comotion {

namespace {

double configDistance(const std::vector<double> &a,
                     const std::vector<double> &b) {
    double d = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        double dd = a[i] - b[i];
        d += dd * dd;
    }
    return std::sqrt(d);
}

} // anonymous namespace

MRdRRTStar::MRdRRTStar() {
    setStopAtFirstSolution(false);
    setUseStarRewiring(true);
}

const char *MRdRRT::costMetricName(const CostMetric metric) {
    switch (metric) {
    case CostMetric::SumOfCosts:
        return "sum_of_costs";
    case CostMetric::CompositeL2:
        return "composite_l2";
    case CostMetric::Makespan:
        return "makespan";
    }
    return "unknown";
}

const char *MRdRRT::tensorSearchModeName(const TensorSearchMode mode) {
    switch (mode) {
    case TensorSearchMode::Drrt:
        return "drrt";
    case TensorSearchMode::AStar:
        return "astar";
    case TensorSearchMode::LazyAStar:
        return "lazy_astar";
    }
    return "unknown";
}

void MRdRRT::buildRoadmaps(
    std::optional<std::chrono::steady_clock::time_point> solve_deadline) {
    using clock = std::chrono::steady_clock;
    int n = problem_->numRobots();
    roadmaps_.resize(n);

    const int target_vertices = std::max(2, roadmap_size_);

    for (int r = 0; r < n; ++r) {
        const auto before_robot = clock::now();
        if (solve_deadline && before_robot >= *solve_deadline) {
            break;
        }

        auto &rm = roadmaps_[r];
        auto &robot = problem_->robot(r);
        int ndof = robot.model->numJoints();

        auto si = problem_->createSpaceInfo(r);
        auto space = si->getStateSpace();

        ompl::geometric::PRMstar prm(si);
        ompl::base::ProblemDefinitionPtr pdef =
            std::make_shared<ompl::base::ProblemDefinition>(si);

        ompl::base::ScopedState<> start(space);
        ompl::base::ScopedState<> goal(space);
        for (int d = 0; d < ndof; ++d) {
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] =
                robot.start[d];
            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[d] =
                robot.goal[d];
        }
        pdef->setStartAndGoalStates(start, goal);
        prm.setProblemDefinition(pdef);
        prm.setup();

        const auto after_setup = clock::now();
        ompl::base::PlannerTerminationCondition stop_at_vertex_count([&prm, si,
                                                                      target_vertices]() {
            ompl::base::PlannerData pdata(si);
            prm.getPlannerData(pdata);
            return static_cast<int>(pdata.numVertices()) >= target_vertices;
        });
        double remaining_sec = std::numeric_limits<double>::infinity();
        if (solve_deadline) {
            remaining_sec =
                std::chrono::duration<double>(*solve_deadline - after_setup).count();
            // OMPL timed condition requires positive duration; skip PRM if none left.
            if (remaining_sec <= 0.0)
                break;
            ompl::base::PlannerTerminationCondition stop_time =
                ompl::base::timedPlannerTerminationCondition(remaining_sec);
            ompl::base::PlannerTerminationCondition stop =
                ompl::base::plannerOrTerminationCondition(stop_time,
                                                          stop_at_vertex_count);
            prm.solve(stop);
        } else {
            prm.solve(stop_at_vertex_count);
        }

        // Extract roadmap from PlannerData
        ompl::base::PlannerData pdata(si);
        prm.getPlannerData(pdata);

        rm.vertices.clear();
        rm.adjacency.clear();

        int nv = static_cast<int>(pdata.numVertices());
        rm.vertices.resize(nv);
        rm.adjacency.resize(nv);

        for (int i = 0; i < nv; ++i) {
            rm.vertices[i] = stateToConfig(pdata.getVertex(i).getState(), ndof);
        }

        for (int i = 0; i < nv; ++i) {
            std::vector<unsigned int> edges;
            pdata.getEdges(i, edges);
            for (unsigned int j : edges) {
                rm.adjacency[i].push_back(static_cast<int>(j));
            }
            // No self-edge: tensor-product expansion with self in every factor yields
            // v_new == v_near (zero motion), burning RRT iterations without growing the tree.
        }

        if (nv < 2) {
            rm.vertices.clear();
            rm.adjacency.clear();
            rm.start_vertex = -1;
            rm.goal_vertex = -1;
            continue;
        }

        // OMPL PRM::getPlannerData adds start milestones then goal milestones before
        // bulk edges; single start/goal ⇒ indices 0 and 1 match robot.start / robot.goal.
        rm.start_vertex = 0;
        rm.goal_vertex = 1;
    }
}

double MRdRRT::compositeDistance(
    const CompositeVertex &a,
    const std::vector<std::vector<double>> &target) const {
    return localDistance(a, target);
}

double MRdRRT::compositeDistance(const CompositeVertex &a,
                                 const CompositeVertex &b) const {
    std::vector<std::vector<double>> target;
    target.reserve(b.vertex_ids.size());
    for (std::size_t r = 0; r < b.vertex_ids.size(); ++r)
        target.push_back(roadmaps_[r].vertices[b.vertex_ids[r]]);
    return localDistance(a, target);
}

double MRdRRT::localDistance(
    const CompositeVertex &a,
    const std::vector<std::vector<double>> &target) const {
    double sum = 0.0;
    double max_dist = 0.0;
    double sq_sum = 0.0;
    for (size_t r = 0; r < a.vertex_ids.size(); ++r) {
        const double d = configDistance(roadmaps_[r].vertices[a.vertex_ids[r]],
                                        target[r]);
        sum += d;
        max_dist = std::max(max_dist, d);
        sq_sum += d * d;
    }

    switch (cost_metric_) {
    case CostMetric::SumOfCosts:
        return sum;
    case CostMetric::CompositeL2:
        return std::sqrt(sq_sum);
    case CostMetric::Makespan:
        return max_dist;
    }
    return sum;
}

double MRdRRT::edgeCost(const CompositeVertex &from,
                        const CompositeVertex &to) const {
    return compositeDistance(from, to);
}

double MRdRRT::localEdgeCost(
    const CompositeVertex &from,
    const std::vector<std::vector<double>> &target) const {
    return localDistance(from, target);
}

bool MRdRRT::areTensorAdjacent(const CompositeVertex &a,
                               const CompositeVertex &b) const {
    if (a.vertex_ids.size() != b.vertex_ids.size())
        return false;

    bool any_changed = false;
    for (std::size_t r = 0; r < a.vertex_ids.size(); ++r) {
        const int av = a.vertex_ids[r];
        const int bv = b.vertex_ids[r];
        if (av == bv)
            continue;
        any_changed = true;
        const auto &adj = roadmaps_[r].adjacency[av];
        if (std::find(adj.begin(), adj.end(), bv) == adj.end())
            return false;
    }
    return any_changed;
}

bool MRdRRT::wouldCreateCycle(const int node_index,
                              const int candidate_parent) const {
    for (int idx = candidate_parent; idx >= 0; idx = tree_[idx].parent) {
        if (idx == node_index)
            return true;
    }
    return false;
}

void MRdRRT::removeChild(const int parent_index, const int child_index) {
    if (parent_index < 0)
        return;
    auto &children = tree_[parent_index].children;
    children.erase(std::remove(children.begin(), children.end(), child_index),
                   children.end());
}

void MRdRRT::addChild(const int parent_index, const int child_index) {
    if (parent_index < 0)
        return;
    auto &children = tree_[parent_index].children;
    if (std::find(children.begin(), children.end(), child_index) == children.end())
        children.push_back(child_index);
}

void MRdRRT::propagateCostDelta(const int node_index, const double delta) {
    tree_[node_index].cost += delta;
    for (const int child : tree_[node_index].children)
        propagateCostDelta(child, delta);
}

int MRdRRT::chooseBestParent(const CompositeVertex &v_new,
                             const int fallback_parent,
                             const double fallback_edge_cost) const {
    if (!use_star_rewiring_)
        return fallback_parent;

    int best_parent = fallback_parent;
    double best_cost = tree_[fallback_parent].cost + fallback_edge_cost;

    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        if (i == fallback_parent)
            continue;
        if (!areTensorAdjacent(tree_[i].vertex, v_new))
            continue;
        const double candidate_edge_cost = edgeCost(tree_[i].vertex, v_new);
        const double candidate_cost = tree_[i].cost + candidate_edge_cost;
        if (candidate_cost + 1e-12 >= best_cost)
            continue;
        if (!isCompositeEdgeValid(tree_[i].vertex, v_new))
            continue;
        best_cost = candidate_cost;
        best_parent = i;
    }

    return best_parent;
}

int MRdRRT::rewireFrom(const int added_index) {
    if (!use_star_rewiring_)
        return 0;

    int rewired = 0;
    const CompositeVertex &added = tree_[added_index].vertex;
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        if (i == added_index || i == 0)
            continue;
        if (tree_[i].parent == added_index)
            continue;
        if (!areTensorAdjacent(added, tree_[i].vertex))
            continue;
        if (wouldCreateCycle(i, added_index))
            continue;

        const double candidate_edge_cost = edgeCost(added, tree_[i].vertex);
        const double candidate_cost = tree_[added_index].cost + candidate_edge_cost;
        if (candidate_cost + 1e-12 >= tree_[i].cost)
            continue;
        if (!isCompositeEdgeValid(added, tree_[i].vertex))
            continue;

        const int old_parent = tree_[i].parent;
        removeChild(old_parent, i);
        tree_[i].parent = added_index;
        addChild(added_index, i);
        propagateCostDelta(i, candidate_cost - tree_[i].cost);
        ++rewired;
    }
    return rewired;
}

std::vector<Path> MRdRRT::pathsFromCompositePath(
    const std::vector<CompositeVertex> &cv_path) const {
    const int n = problem_->numRobots();
    std::vector<Path> paths(static_cast<std::size_t>(n));
    for (const auto &cv : cv_path) {
        for (int r = 0; r < n; ++r)
            paths[static_cast<std::size_t>(r)].push_back(
                roadmaps_[r].vertices[cv.vertex_ids[r]]);
    }

    const double vmax = problem_->vmax();
    const size_t resolution = problem_->resolution();
    std::vector<double> segment_times_sec;
    if (!paths.empty() && paths[0].size() >= 2) {
        const size_t n_segments = paths[0].size() - 1;
        segment_times_sec.reserve(n_segments);
        for (size_t seg = 0; seg < n_segments; ++seg) {
            double max_seg_dist = 0.0;
            for (const auto &rp : paths)
                max_seg_dist =
                    std::max(max_seg_dist, configDistance(rp[seg], rp[seg + 1]));
            segment_times_sec.push_back(max_seg_dist / vmax);
        }
    }

    for (auto &p : paths) {
        if (!segment_times_sec.empty())
            p.setTimestepsFromSegmentTimes(segment_times_sec, resolution);
        p.interpolate_to_timesteps(resolution, vmax);
    }
    return paths;
}

MRdRRT::CompositeVertex MRdRRT::directionOracle(
    const CompositeVertex &v_near,
    const std::vector<std::vector<double>> &q_rand) const {

    constexpr double kEpsDir2 = 1e-18; // min squared length for a direction

    CompositeVertex v_new;
    v_new.vertex_ids.resize(v_near.vertex_ids.size());

    // For each robot, find the best neighbor in the roadmap
    for (size_t r = 0; r < v_near.vertex_ids.size(); ++r) {
        int best_neighbor = v_near.vertex_ids[r];
        double best_angle = std::numeric_limits<double>::infinity();
        double best_alignment = -std::numeric_limits<double>::infinity();

        const auto &v_near_config = roadmaps_[r].vertices[v_near.vertex_ids[r]];
        const auto &q_rand_r = q_rand[r];

        // Calculate the magnitude of the direction vector to q_rand
        double mag_a = 0;
        for (size_t d = 0; d < v_near_config.size(); ++d) {
            double da = q_rand_r[d] - v_near_config[d];
            mag_a += da * da;
        }

        // No preferred direction in C-space: choose uniformly among roadmap neighbors
        // (avoids "first adjacency entry wins" when all angles would be degenerate).
        if (mag_a < kEpsDir2) {
            // const auto &adj = roadmaps_[r].adjacency[v_near.vertex_ids[r]];
            // std::vector<int> candidates;
            // candidates.reserve(adj.size());
            // for (int nb : adj) {
            //     if (nb == v_near.vertex_ids[r])
            //         continue;
            //     const auto &vc = roadmaps_[r].vertices[nb];
            //     double mb = 0;
            //     for (size_t d = 0; d < v_near_config.size(); ++d) {
            //         double db = vc[d] - v_near_config[d];
            //         mb += db * db;
            //     }
            //     if (mb >= kEpsDir2)
            //         candidates.push_back(nb);
            // }
            // if (!candidates.empty()) {
            //     std::uniform_int_distribution<size_t> dist(0, candidates.size() - 1);
            //     best_neighbor = candidates[dist(rng_)];
            // }
            // v_new.vertex_ids[r] = best_neighbor;

            // Instead of forcing a neighbor, we allow the direction to be zero and
            // let this robot stay in the same place.
            v_new.vertex_ids[r] = v_near.vertex_ids[r];
            continue;
        }

        const double inv_sqrt_mag_a = 1.0 / std::sqrt(mag_a);

        for (int neighbor : roadmaps_[r].adjacency[v_near.vertex_ids[r]]) {
            if (neighbor == v_near.vertex_ids[r])
                continue;

            const auto &v_config = roadmaps_[r].vertices[neighbor];

            double dot = 0, mag_b = 0;
            for (size_t d = 0; d < v_near_config.size(); ++d) {
                double da = q_rand_r[d] - v_near_config[d];
                double db = v_config[d] - v_near_config[d];
                dot += da * db;
                mag_b += db * db;
            }

            if (mag_b < kEpsDir2)
                continue;

            const double cosang =
                std::clamp(dot / (std::sqrt(mag_b) * std::sqrt(mag_a)), -1.0, 1.0);
            const double angle = std::acos(cosang);
            const double alignment = dot * inv_sqrt_mag_a; // = ||b|| cos(angle)

            // Update the best neighbor if the new neighbor is better
            // - the new neighbor has a smaller angle
            // - the new neighbor has the same angle but is more aligned with the direction vector
            if (angle < best_angle - 1e-15 ||
                (std::abs(angle - best_angle) <= 1e-15 &&
                 alignment > best_alignment)) {
                best_angle = angle;
                best_alignment = alignment;
                best_neighbor = neighbor;
            }
        }
        // Set the best neighbor for this robot
        v_new.vertex_ids[r] = best_neighbor;
    }
    return v_new;
}

bool MRdRRT::isCompositeEdgeValid(const CompositeVertex &from,
                                    const CompositeVertex &to) const {
    int n = static_cast<int>(from.vertex_ids.size());
    auto ptrs = problem_->robotModelPtrs();

    double edge_distance = 0.0;
    for (int r = 0; r < n; ++r) {
        double d = configDistance(
            roadmaps_[r].vertices[from.vertex_ids[r]],
            roadmaps_[r].vertices[to.vertex_ids[r]]);
        edge_distance = std::max(edge_distance, d);
    }

    const double longest_valid_segment = longest_valid_segment_cache_;
    int num_checks = std::max(3, static_cast<int>(std::ceil(edge_distance / longest_valid_segment)) - 1);

    std::vector<std::vector<double>> from_configs(static_cast<std::size_t>(n));
    std::vector<std::vector<double>> to_configs(static_cast<std::size_t>(n));
    for (int r = 0; r < n; ++r) {
        from_configs[static_cast<std::size_t>(r)] =
            roadmaps_[r].vertices[from.vertex_ids[r]];
        to_configs[static_cast<std::size_t>(r)] =
            roadmaps_[r].vertices[to.vertex_ids[r]];
    }

    CompositePathValidationOptions options;
    options.check_environment = false;
    options.discrete_num_checks_hint = num_checks;
    bool ok = problem_->collisionChecker().isCompositeMotionValid(
        ptrs, from_configs, to_configs, options);
    return ok;
}

bool MRdRRT::localConnector(
    const CompositeVertex &from,
    const std::vector<std::vector<double>> &goal_configs,
    const std::chrono::steady_clock::time_point deadline) const {
    const int n = static_cast<int>(from.vertex_ids.size());
    auto ptrs = problem_->robotModelPtrs();

    double edge_distance = 0.0;
    for (int r = 0; r < n; ++r) {
        double d = configDistance(roadmaps_[r].vertices[from.vertex_ids[r]],
                                  goal_configs[r]);
        edge_distance = std::max(edge_distance, d);
    }

    const double longest_valid_segment = longest_valid_segment_cache_;
    const int num_checks =
        std::max(3, static_cast<int>(std::ceil(edge_distance / longest_valid_segment)) - 1);

    if (std::chrono::steady_clock::now() >= deadline)
        return false;

    std::vector<std::vector<double>> from_configs(static_cast<std::size_t>(n));
    for (int r = 0; r < n; ++r) {
        from_configs[static_cast<std::size_t>(r)] =
            roadmaps_[r].vertices[from.vertex_ids[r]];
    }

    CompositePathValidationOptions options;
    options.check_environment = true;
    options.discrete_num_checks_hint = num_checks;
    return problem_->collisionChecker().isCompositeMotionValid(
        ptrs, from_configs, goal_configs, options);
}

std::vector<MRdRRT::CompositeVertex> MRdRRT::traceTreePathToRoot(
    int node_index) const {
    std::vector<CompositeVertex> rev;
    for (int idx = node_index; idx >= 0; idx = tree_[idx].parent)
        rev.push_back(tree_[idx].vertex);
    std::reverse(rev.begin(), rev.end());
    return rev;
}

std::optional<std::vector<MRdRRT::CompositeVertex>> MRdRRT::connectToTarget(
    const CompositeVertex &goal_v,
    const std::unordered_map<CompositeVertex, int, CompositeVertexHash>
        &vertex_to_node,
    const uint64_t total_iteration,
    const std::chrono::steady_clock::time_point deadline,
    double *path_cost) const {

    auto git = vertex_to_node.find(goal_v);
    if (git != vertex_to_node.end()) {
        if (path_cost)
            *path_cost = tree_[git->second].cost;
        return traceTreePathToRoot(git->second);
    }

    const int n = static_cast<int>(goal_v.vertex_ids.size());
    std::vector<std::vector<double>> goal_configs(static_cast<size_t>(n));
    for (int r = 0; r < n; ++r)
        goal_configs[static_cast<size_t>(r)] =
            roadmaps_[r].vertices[goal_v.vertex_ids[r]];

    const size_t nt = tree_.size();
    if (nt == 0)
        return std::nullopt;

    const int k_want = std::max(
        1, static_cast<int>(std::floor(std::log2(static_cast<double>(
               std::max<uint64_t>(1, total_iteration))))));
    const size_t k = std::min(static_cast<size_t>(k_want), nt);

    std::vector<std::pair<double, int>> ranked;
    ranked.reserve(nt);
    for (size_t i = 0; i < nt; ++i) {
        ranked.push_back(
            {compositeDistance(tree_[i].vertex, goal_configs), static_cast<int>(i)});
    }
    const auto dist_less = [](const std::pair<double, int> &a,
                              const std::pair<double, int> &b) {
        return a.first < b.first;
    };
    std::partial_sort(ranked.begin(), ranked.begin() + static_cast<std::ptrdiff_t>(k),
                      ranked.end(), dist_less);

    std::optional<std::vector<CompositeVertex>> best_path;
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t j = 0; j < k; ++j) {
        if (std::chrono::steady_clock::now() >= deadline)
            break;
        const int tree_idx = ranked[j].second;
        const CompositeVertex &anchor = tree_[tree_idx].vertex;
        if (local_connect_failed_.count(anchor)) {
            continue;
        }
        if (!localConnector(anchor, goal_configs, deadline)) {
            local_connect_failed_.insert(anchor);
            continue;
        }
        const double candidate_last_edge_cost =
            localEdgeCost(anchor, goal_configs);
        const double candidate_cost =
            tree_[tree_idx].cost + candidate_last_edge_cost;
        if (candidate_cost + 1e-12 >= best_cost)
            continue;
        std::vector<CompositeVertex> path = traceTreePathToRoot(tree_idx);
        path.push_back(goal_v);
        best_cost = candidate_cost;
        best_path = std::move(path);
        if (stop_at_first_solution_)
            break;
    }

    if (best_path) {
        if (path_cost)
            *path_cost = best_cost;
        return best_path;
    }

    return std::nullopt;
}

int MRdRRT::findNearestNode(
    const std::vector<std::vector<double>> &q_rand) const {
    int best = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        double d = compositeDistance(tree_[i].vertex, q_rand);
        if (d < best_dist) {
            best_dist = d;
            best = i;
        }
    }
    return best;
}

std::vector<std::vector<double>>
MRdRRT::computeIndividualGoalDistances() const {
    std::vector<std::vector<double>> distances;
    distances.reserve(roadmaps_.size());

    for (const auto &rm : roadmaps_) {
        const int nv = static_cast<int>(rm.vertices.size());
        std::vector<std::vector<int>> reverse_adjacency(static_cast<std::size_t>(nv));
        for (int u = 0; u < nv; ++u) {
            for (int v : rm.adjacency[static_cast<std::size_t>(u)]) {
                if (v >= 0 && v < nv)
                    reverse_adjacency[static_cast<std::size_t>(v)].push_back(u);
            }
        }

        std::vector<double> dist(static_cast<std::size_t>(nv),
                                 std::numeric_limits<double>::infinity());
        using QueueEntry = std::pair<double, int>;
        std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                            std::greater<QueueEntry>>
            queue;

        if (rm.goal_vertex >= 0 && rm.goal_vertex < nv) {
            dist[static_cast<std::size_t>(rm.goal_vertex)] = 0.0;
            queue.push({0.0, rm.goal_vertex});
        }

        while (!queue.empty()) {
            const auto [cost, vertex] = queue.top();
            queue.pop();
            if (cost > dist[static_cast<std::size_t>(vertex)] + 1e-12)
                continue;

            for (int prev : reverse_adjacency[static_cast<std::size_t>(vertex)]) {
                const double edge =
                    configDistance(rm.vertices[static_cast<std::size_t>(prev)],
                                   rm.vertices[static_cast<std::size_t>(vertex)]);
                const double candidate = cost + edge;
                auto &prev_dist = dist[static_cast<std::size_t>(prev)];
                if (candidate + 1e-12 < prev_dist) {
                    prev_dist = candidate;
                    queue.push({candidate, prev});
                }
            }
        }

        distances.push_back(std::move(dist));
    }

    return distances;
}

double MRdRRT::astarHeuristic(
    const CompositeVertex &v,
    const std::vector<std::vector<double>> &individual_goal_distances) const {
    double h = 0.0;
    for (std::size_t r = 0; r < v.vertex_ids.size(); ++r) {
        const int vertex = v.vertex_ids[r];
        if (vertex < 0 ||
            static_cast<std::size_t>(vertex) >= individual_goal_distances[r].size())
            return std::numeric_limits<double>::infinity();
        h = std::max(h, individual_goal_distances[r][static_cast<std::size_t>(vertex)]);
    }
    return h;
}

ompl::base::PlannerStatus MRdRRT::solveTensorAStar(
    const CompositeVertex &start_v, const CompositeVertex &goal_v,
    const std::chrono::steady_clock::time_point accounting_start_time,
    double &best_solution_cost, nlohmann::json &solution_events,
    nlohmann::json &astar_stats) {
    using clock = std::chrono::steady_clock;

    struct AStarNode {
        CompositeVertex vertex;
        int parent = -1;
        double g = std::numeric_limits<double>::infinity();
    };

    struct QueueEntry {
        double f = std::numeric_limits<double>::infinity();
        double g = std::numeric_limits<double>::infinity();
        int node_index = -1;
        std::uint64_t sequence = 0;
    };

    struct QueueEntryGreater {
        bool operator()(const QueueEntry &lhs, const QueueEntry &rhs) const {
            if (std::abs(lhs.f - rhs.f) > 1e-12)
                return lhs.f > rhs.f;
            if (std::abs(lhs.g - rhs.g) > 1e-12)
                return lhs.g < rhs.g;
            return lhs.sequence > rhs.sequence;
        }
    };

    struct TensorEdgeKey {
        CompositeVertex from;
        CompositeVertex to;
        bool operator==(const TensorEdgeKey &other) const {
            return from == other.from && to == other.to;
        }
    };

    struct TensorEdgeKeyHash {
        std::size_t operator()(const TensorEdgeKey &key) const {
            const CompositeVertexHash vertex_hash;
            std::size_t seed = vertex_hash(key.from);
            const std::size_t to_hash = vertex_hash(key.to);
            seed ^= to_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    const auto individual_goal_distances = computeIndividualGoalDistances();
    astar_stats = nlohmann::json::object();
    astar_stats["heuristic"] = "max_individual_graph_cost_to_go";
    astar_stats["expansions"] = 0;
    astar_stats["generated"] = 0;
    astar_stats["enqueued"] = 0;
    astar_stats["reopened"] = 0;
    astar_stats["stale_pops"] = 0;
    astar_stats["collision_checks"] = 0;
    astar_stats["edge_cache_hits"] = 0;
    astar_stats["deadline_reached"] = false;
    astar_stats["unbounded_search"] = true;
    astar_stats["cancellation_requested"] = false;

    std::vector<AStarNode> nodes;
    nodes.push_back({start_v, -1, 0.0});

    std::unordered_map<CompositeVertex, int, CompositeVertexHash> vertex_to_node;
    vertex_to_node[start_v] = 0;

    std::unordered_set<CompositeVertex, CompositeVertexHash> closed;
    std::unordered_map<TensorEdgeKey, bool, TensorEdgeKeyHash> edge_valid_cache;

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater>
        open;
    std::uint64_t sequence = 0;
    const double start_h = astarHeuristic(start_v, individual_goal_distances);
    if (std::isfinite(start_h))
        open.push({start_h, 0.0, 0, sequence++});

    std::uint64_t expansions = 0;
    std::uint64_t generated = 0;
    std::uint64_t enqueued = 1;
    std::uint64_t reopened = 0;
    std::uint64_t stale_pops = 0;
    std::uint64_t collision_checks = 0;
    std::uint64_t edge_cache_hits = 0;
    bool cancellation_requested = false;

    auto refreshStats = [&]() {
        astar_stats["expansions"] = expansions;
        astar_stats["generated"] = generated;
        astar_stats["enqueued"] = enqueued;
        astar_stats["reopened"] = reopened;
        astar_stats["stale_pops"] = stale_pops;
        astar_stats["collision_checks"] = collision_checks;
        astar_stats["edge_cache_hits"] = edge_cache_hits;
        astar_stats["deadline_reached"] = false;
        astar_stats["unbounded_search"] = true;
        astar_stats["cancellation_requested"] = cancellation_requested;
        astar_stats["open_remaining"] = open.size();
        astar_stats["closed_vertices"] = closed.size();
        astar_stats["discovered_vertices"] = nodes.size();
        astar_stats["edge_cache_size"] = edge_valid_cache.size();
    };

    if (!std::isfinite(start_h)) {
        refreshStats();
        return ompl::base::PlannerStatus::TIMEOUT;
    }

    while (!open.empty()) {
        if (cancellationRequested()) {
            cancellation_requested = true;
            break;
        }

        const QueueEntry entry = open.top();
        open.pop();
        if (entry.node_index < 0 ||
            entry.node_index >= static_cast<int>(nodes.size()) ||
            entry.g > nodes[static_cast<std::size_t>(entry.node_index)].g + 1e-12) {
            ++stale_pops;
            continue;
        }

        const CompositeVertex current_vertex =
            nodes[static_cast<std::size_t>(entry.node_index)].vertex;
        const double current_g =
            nodes[static_cast<std::size_t>(entry.node_index)].g;
        if (closed.count(current_vertex)) {
            ++stale_pops;
            continue;
        }
        closed.insert(current_vertex);
        ++expansions;

        if (current_vertex == goal_v) {
            std::vector<CompositeVertex> reverse_path;
            for (int idx = entry.node_index; idx >= 0;
                 idx = nodes[static_cast<std::size_t>(idx)].parent) {
                reverse_path.push_back(nodes[static_cast<std::size_t>(idx)].vertex);
            }
            std::reverse(reverse_path.begin(), reverse_path.end());

            solution_paths_ = pathsFromCompositePath(reverse_path);
            setSolutionMetricsFromPaths(solution_paths_);
            best_solution_cost = current_g;
            solution_events.push_back({
                {"elapsed_seconds",
                 std::chrono::duration<double>(clock::now() - accounting_start_time)
                     .count()},
                {"cost_metric", costMetricName(cost_metric_)},
                {"solution_cost", best_solution_cost},
                {"makespan_timesteps",
                 makespanTimesteps() ? nlohmann::json(*makespanTimesteps())
                                     : nlohmann::json(nullptr)},
                {"sum_of_cost_timesteps",
                 sumOfCostTimesteps() ? nlohmann::json(*sumOfCostTimesteps())
                                      : nlohmann::json(nullptr)},
                {"kind", "first_solution"},
            });
            refreshStats();
            return ompl::base::PlannerStatus::EXACT_SOLUTION;
        }

        CompositeVertex next;
        next.vertex_ids.resize(current_vertex.vertex_ids.size());

        std::function<void(std::size_t, bool)> enumerate =
            [&](const std::size_t robot_index, const bool any_changed) {
                if (cancellation_requested)
                    return;
                if (cancellationRequested()) {
                    cancellation_requested = true;
                    return;
                }

                if (robot_index == current_vertex.vertex_ids.size()) {
                    if (!any_changed)
                        return;
                    ++generated;

                    const double h =
                        astarHeuristic(next, individual_goal_distances);
                    if (!std::isfinite(h))
                        return;

                    const double tentative_g =
                        current_g + edgeCost(current_vertex, next);
                    auto found = vertex_to_node.find(next);
                    if (found != vertex_to_node.end()) {
                        const auto existing_idx =
                            static_cast<std::size_t>(found->second);
                        if (tentative_g + 1e-12 >= nodes[existing_idx].g)
                            return;
                    }

                    const TensorEdgeKey edge_key{current_vertex, next};
                    bool valid_edge = false;
                    auto cached = edge_valid_cache.find(edge_key);
                    if (cached != edge_valid_cache.end()) {
                        ++edge_cache_hits;
                        valid_edge = cached->second;
                    } else {
                        ++collision_checks;
                        valid_edge = isCompositeEdgeValid(current_vertex, next);
                        edge_valid_cache.emplace(edge_key, valid_edge);
                    }
                    if (!valid_edge)
                        return;

                    int next_node_index = -1;
                    if (found == vertex_to_node.end()) {
                        next_node_index = static_cast<int>(nodes.size());
                        nodes.push_back({next, entry.node_index, tentative_g});
                        vertex_to_node[next] = next_node_index;
                    } else {
                        next_node_index = found->second;
                        auto &existing =
                            nodes[static_cast<std::size_t>(next_node_index)];
                        existing.parent = entry.node_index;
                        existing.g = tentative_g;
                        if (closed.erase(next) > 0)
                            ++reopened;
                    }

                    open.push({tentative_g + h, tentative_g, next_node_index,
                               sequence++});
                    ++enqueued;
                    return;
                }

                const int current_robot_vertex =
                    current_vertex.vertex_ids[robot_index];
                next.vertex_ids[robot_index] = current_robot_vertex;
                enumerate(robot_index + 1, any_changed);

                const auto &adjacency =
                    roadmaps_[robot_index].adjacency[static_cast<std::size_t>(
                        current_robot_vertex)];
                for (const int neighbor : adjacency) {
                    if (neighbor == current_robot_vertex)
                        continue;
                    next.vertex_ids[robot_index] = neighbor;
                    enumerate(robot_index + 1, true);
                    if (cancellation_requested)
                        return;
                }
            };

        enumerate(0, false);

    }

    refreshStats();
    return ompl::base::PlannerStatus::TIMEOUT;
}

ompl::base::PlannerStatus MRdRRT::solveTensorLazyAStar(
    const CompositeVertex &start_v, const CompositeVertex &goal_v,
    const std::chrono::steady_clock::time_point accounting_start_time,
    double &best_solution_cost, nlohmann::json &solution_events,
    nlohmann::json &lazy_astar_stats) {
    using clock = std::chrono::steady_clock;
    constexpr double kInf = std::numeric_limits<double>::infinity();
    constexpr double kTol = 1e-12;

    struct LpaNode {
        CompositeVertex vertex;
        double g = kInf;
        double rhs = kInf;
        int parent = -1;
    };

    struct QueueKey {
        double k1 = kInf;
        double k2 = kInf;
    };

    struct QueueEntry {
        QueueKey key;
        int node_index = -1;
        std::uint64_t sequence = 0;
    };

    struct QueueEntryGreater {
        bool operator()(const QueueEntry &lhs, const QueueEntry &rhs) const {
            if (std::abs(lhs.key.k1 - rhs.key.k1) > 1e-12)
                return lhs.key.k1 > rhs.key.k1;
            if (std::abs(lhs.key.k2 - rhs.key.k2) > 1e-12)
                return lhs.key.k2 > rhs.key.k2;
            return lhs.sequence > rhs.sequence;
        }
    };

    struct TensorEdgeKey {
        CompositeVertex from;
        CompositeVertex to;
        bool operator==(const TensorEdgeKey &other) const {
            return from == other.from && to == other.to;
        }
    };

    struct TensorEdgeKeyHash {
        std::size_t operator()(const TensorEdgeKey &key) const {
            const CompositeVertexHash vertex_hash;
            std::size_t seed = vertex_hash(key.from);
            const std::size_t to_hash = vertex_hash(key.to);
            seed ^= to_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    enum class EdgeStatus {
        Valid,
        Invalid,
    };

    auto individual_goal_distances = computeIndividualGoalDistances();
    std::vector<std::vector<std::vector<int>>> incoming(roadmaps_.size());
    for (std::size_t r = 0; r < roadmaps_.size(); ++r) {
        const int nv = static_cast<int>(roadmaps_[r].vertices.size());
        incoming[r].resize(static_cast<std::size_t>(nv));
        for (int u = 0; u < nv; ++u) {
            for (const int v : roadmaps_[r].adjacency[static_cast<std::size_t>(u)]) {
                if (v >= 0 && v < nv)
                    incoming[r][static_cast<std::size_t>(v)].push_back(u);
            }
        }
    }

    lazy_astar_stats = nlohmann::json::object();
    lazy_astar_stats["heuristic"] = "max_individual_graph_cost_to_go";
    lazy_astar_stats["unbounded_search"] = true;
    lazy_astar_stats["candidate_paths"] = 0;
    lazy_astar_stats["repairs"] = 0;
    lazy_astar_stats["lpa_expansions"] = 0;
    lazy_astar_stats["generated_successors"] = 0;
    lazy_astar_stats["generated_predecessors"] = 0;
    lazy_astar_stats["validated_edges"] = 0;
    lazy_astar_stats["invalid_edges"] = 0;
    lazy_astar_stats["valid_edges"] = 0;
    lazy_astar_stats["stale_pops"] = 0;
    lazy_astar_stats["cancellation_requested"] = false;
    lazy_astar_stats["graph_exhausted"] = false;

    std::vector<LpaNode> nodes;
    nodes.push_back({start_v, kInf, 0.0, -1});

    std::unordered_map<CompositeVertex, int, CompositeVertexHash> vertex_to_node;
    vertex_to_node[start_v] = 0;

    std::unordered_map<TensorEdgeKey, EdgeStatus, TensorEdgeKeyHash> edge_status;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater>
        open;

    std::uint64_t sequence = 0;
    std::uint64_t candidate_paths = 0;
    std::uint64_t repairs = 0;
    std::uint64_t lpa_expansions = 0;
    std::uint64_t generated_successors = 0;
    std::uint64_t generated_predecessors = 0;
    std::uint64_t validated_edges = 0;
    std::uint64_t invalid_edges = 0;
    std::uint64_t valid_edges = 0;
    std::uint64_t stale_pops = 0;
    bool cancellation_requested = false;
    bool graph_exhausted = false;

    auto getNode = [&](const CompositeVertex &vertex) -> int {
        auto found = vertex_to_node.find(vertex);
        if (found != vertex_to_node.end())
            return found->second;
        const int index = static_cast<int>(nodes.size());
        nodes.push_back({vertex, kInf, kInf, -1});
        vertex_to_node.emplace(vertex, index);
        return index;
    };

    const int goal_index = getNode(goal_v);

    auto keyLess = [&](const QueueKey &lhs, const QueueKey &rhs) {
        if (std::abs(lhs.k1 - rhs.k1) > kTol)
            return lhs.k1 < rhs.k1;
        return lhs.k2 < rhs.k2 - kTol;
    };

    auto keyEqual = [&](const QueueKey &lhs, const QueueKey &rhs) {
        const bool k1_equal =
            (std::isinf(lhs.k1) && std::isinf(rhs.k1)) ||
            std::abs(lhs.k1 - rhs.k1) <= kTol;
        const bool k2_equal =
            (std::isinf(lhs.k2) && std::isinf(rhs.k2)) ||
            std::abs(lhs.k2 - rhs.k2) <= kTol;
        return k1_equal && k2_equal;
    };

    auto calculateKey = [&](const int index) -> QueueKey {
        const LpaNode &node = nodes[static_cast<std::size_t>(index)];
        const double min_cost = std::min(node.g, node.rhs);
        if (!std::isfinite(min_cost))
            return {kInf, kInf};
        return {min_cost + astarHeuristic(node.vertex, individual_goal_distances),
                min_cost};
    };

    auto pushIfInconsistent = [&](const int index) {
        const LpaNode &node = nodes[static_cast<std::size_t>(index)];
        if (std::abs(node.g - node.rhs) <= kTol)
            return;
        const QueueKey key = calculateKey(index);
        if (!std::isfinite(key.k1))
            return;
        open.push({key, index, sequence++});
    };

    auto edgeIsInvalid = [&](const CompositeVertex &from,
                             const CompositeVertex &to) {
        const auto found = edge_status.find({from, to});
        return found != edge_status.end() &&
               found->second == EdgeStatus::Invalid;
    };

    auto enumerateSuccessors = [&](const CompositeVertex &vertex,
                                   const auto &visit) {
        CompositeVertex next;
        next.vertex_ids.resize(vertex.vertex_ids.size());
        std::function<void(std::size_t, bool)> recurse =
            [&](const std::size_t robot_index, const bool any_changed) {
                if (robot_index == vertex.vertex_ids.size()) {
                    if (!any_changed)
                        return;
                    ++generated_successors;
                    if (!edgeIsInvalid(vertex, next))
                        visit(next);
                    return;
                }

                const int current = vertex.vertex_ids[robot_index];
                next.vertex_ids[robot_index] = current;
                recurse(robot_index + 1, any_changed);
                for (const int neighbor :
                     roadmaps_[robot_index]
                         .adjacency[static_cast<std::size_t>(current)]) {
                    if (neighbor == current)
                        continue;
                    next.vertex_ids[robot_index] = neighbor;
                    recurse(robot_index + 1, true);
                }
            };
        recurse(0, false);
    };

    auto enumeratePredecessors = [&](const CompositeVertex &vertex,
                                     const auto &visit) {
        CompositeVertex prev;
        prev.vertex_ids.resize(vertex.vertex_ids.size());
        std::function<void(std::size_t, bool)> recurse =
            [&](const std::size_t robot_index, const bool any_changed) {
                if (robot_index == vertex.vertex_ids.size()) {
                    if (!any_changed)
                        return;
                    ++generated_predecessors;
                    if (!edgeIsInvalid(prev, vertex))
                        visit(prev);
                    return;
                }

                const int current = vertex.vertex_ids[robot_index];
                prev.vertex_ids[robot_index] = current;
                recurse(robot_index + 1, any_changed);
                for (const int neighbor :
                     incoming[robot_index][static_cast<std::size_t>(current)]) {
                    if (neighbor == current)
                        continue;
                    prev.vertex_ids[robot_index] = neighbor;
                    recurse(robot_index + 1, true);
                }
            };
        recurse(0, false);
    };

    std::function<void(int)> updateVertex = [&](const int index) {
        if (index != 0) {
            double best_rhs = kInf;
            int best_parent = -1;
            enumeratePredecessors(nodes[static_cast<std::size_t>(index)].vertex,
                                  [&](const CompositeVertex &pred) {
                auto found = vertex_to_node.find(pred);
                if (found == vertex_to_node.end())
                    return;
                const int pred_index = found->second;
                const double pred_g = nodes[static_cast<std::size_t>(pred_index)].g;
                if (!std::isfinite(pred_g))
                    return;
                const double candidate =
                    pred_g + edgeCost(pred, nodes[static_cast<std::size_t>(index)].vertex);
                if (candidate + kTol < best_rhs) {
                    best_rhs = candidate;
                    best_parent = pred_index;
                }
            });
            nodes[static_cast<std::size_t>(index)].rhs = best_rhs;
            nodes[static_cast<std::size_t>(index)].parent = best_parent;
        }
        pushIfInconsistent(index);
    };

    auto refreshStats = [&]() {
        std::uint64_t cached_valid = 0;
        std::uint64_t cached_invalid = 0;
        for (const auto &item : edge_status) {
            if (item.second == EdgeStatus::Valid)
                ++cached_valid;
            else
                ++cached_invalid;
        }
        lazy_astar_stats["candidate_paths"] = candidate_paths;
        lazy_astar_stats["repairs"] = repairs;
        lazy_astar_stats["lpa_expansions"] = lpa_expansions;
        lazy_astar_stats["generated_successors"] = generated_successors;
        lazy_astar_stats["generated_predecessors"] = generated_predecessors;
        lazy_astar_stats["validated_edges"] = validated_edges;
        lazy_astar_stats["invalid_edges"] = invalid_edges;
        lazy_astar_stats["valid_edges"] = valid_edges;
        lazy_astar_stats["stale_pops"] = stale_pops;
        lazy_astar_stats["cancellation_requested"] = cancellation_requested;
        lazy_astar_stats["graph_exhausted"] = graph_exhausted;
        lazy_astar_stats["open_remaining"] = open.size();
        lazy_astar_stats["discovered_vertices"] = nodes.size();
        lazy_astar_stats["edge_cache_valid"] = cached_valid;
        lazy_astar_stats["edge_cache_invalid"] = cached_invalid;
    };

    auto computeShortestPath = [&]() {
        for (;;) {
            if (cancellationRequested()) {
                cancellation_requested = true;
                return;
            }

            while (!open.empty()) {
                const QueueEntry top = open.top();
                const bool stale_index =
                    top.node_index < 0 ||
                    top.node_index >= static_cast<int>(nodes.size());
                if (stale_index) {
                    open.pop();
                    ++stale_pops;
                    continue;
                }
                if (keyEqual(top.key, calculateKey(top.node_index)))
                    break;
                open.pop();
                ++stale_pops;
            }

            const QueueKey goal_key = calculateKey(goal_index);
            const bool goal_consistent =
                std::abs(nodes[static_cast<std::size_t>(goal_index)].g -
                         nodes[static_cast<std::size_t>(goal_index)].rhs) <= kTol;
            if (open.empty() ||
                (!keyLess(open.top().key, goal_key) && goal_consistent))
                return;

            const QueueEntry entry = open.top();
            open.pop();
            LpaNode &node = nodes[static_cast<std::size_t>(entry.node_index)];
            ++lpa_expansions;

            if (node.g > node.rhs) {
                node.g = node.rhs;
                const CompositeVertex current = node.vertex;
                const double current_g = node.g;
                enumerateSuccessors(current, [&](const CompositeVertex &succ) {
                    const int succ_index = getNode(succ);
                    const double candidate = current_g + edgeCost(current, succ);
                    LpaNode &succ_node =
                        nodes[static_cast<std::size_t>(succ_index)];
                    if (candidate + kTol < succ_node.rhs) {
                        succ_node.rhs = candidate;
                        succ_node.parent = entry.node_index;
                    }
                    pushIfInconsistent(succ_index);
                });
            } else {
                node.g = kInf;
                const CompositeVertex current = node.vertex;
                updateVertex(entry.node_index);
                enumerateSuccessors(current, [&](const CompositeVertex &succ) {
                    const int succ_index = getNode(succ);
                    updateVertex(succ_index);
                });
            }

        }
    };

    pushIfInconsistent(0);

    while (!cancellation_requested) {
        computeShortestPath();
        if (cancellation_requested)
            break;

        const LpaNode &goal = nodes[static_cast<std::size_t>(goal_index)];
        if (!std::isfinite(goal.g)) {
            graph_exhausted = true;
            break;
        }

        ++candidate_paths;
        std::vector<int> reverse_indices;
        for (int idx = goal_index; idx >= 0;
             idx = nodes[static_cast<std::size_t>(idx)].parent) {
            reverse_indices.push_back(idx);
            if (idx == 0)
                break;
        }
        if (reverse_indices.empty() || reverse_indices.back() != 0) {
            graph_exhausted = true;
            break;
        }
        std::reverse(reverse_indices.begin(), reverse_indices.end());

        bool repaired = false;
        for (std::size_t i = 0; i + 1 < reverse_indices.size(); ++i) {
            const int from_index = reverse_indices[i];
            const int to_index = reverse_indices[i + 1];
            const CompositeVertex &from =
                nodes[static_cast<std::size_t>(from_index)].vertex;
            const CompositeVertex &to =
                nodes[static_cast<std::size_t>(to_index)].vertex;
            const TensorEdgeKey edge{from, to};
            auto cached = edge_status.find(edge);
            if (cached != edge_status.end()) {
                if (cached->second == EdgeStatus::Invalid) {
                    updateVertex(to_index);
                    repaired = true;
                    ++repairs;
                    break;
                }
                continue;
            }

            ++validated_edges;
            if (isCompositeEdgeValid(from, to)) {
                edge_status.emplace(edge, EdgeStatus::Valid);
                ++valid_edges;
                continue;
            }

            edge_status.emplace(edge, EdgeStatus::Invalid);
            ++invalid_edges;
            ++repairs;
            updateVertex(to_index);
            repaired = true;
            break;
        }

        if (repaired)
            continue;

        std::vector<CompositeVertex> path;
        path.reserve(reverse_indices.size());
        for (const int idx : reverse_indices)
            path.push_back(nodes[static_cast<std::size_t>(idx)].vertex);

        solution_paths_ = pathsFromCompositePath(path);
        setSolutionMetricsFromPaths(solution_paths_);
        best_solution_cost = goal.g;
        solution_events.push_back({
            {"elapsed_seconds",
             std::chrono::duration<double>(clock::now() - accounting_start_time)
                 .count()},
            {"cost_metric", costMetricName(cost_metric_)},
            {"solution_cost", best_solution_cost},
            {"makespan_timesteps",
             makespanTimesteps() ? nlohmann::json(*makespanTimesteps())
                                 : nlohmann::json(nullptr)},
            {"sum_of_cost_timesteps",
             sumOfCostTimesteps() ? nlohmann::json(*sumOfCostTimesteps())
                                  : nlohmann::json(nullptr)},
            {"kind", "first_solution"},
        });
        refreshStats();
        return ompl::base::PlannerStatus::EXACT_SOLUTION;
    }

    refreshStats();
    return ompl::base::PlannerStatus::TIMEOUT;
}

ompl::base::PlannerStatus MRdRRT::solve(double timeLimit) {
    resetPlannerRunMetrics();
    tensor_phase_rng_.setLocalSeed(
        comotion::omplLocalSeedForMrDrrtTensorPhase(planning_seed_));

    solution_paths_.clear();
    tree_.clear();
    local_connect_failed_.clear();

    int n = problem_->numRobots();
    auto start_time = std::chrono::steady_clock::now();
    auto accounting_start_time = start_time;
    double best_solution_cost = std::numeric_limits<double>::infinity();
    nlohmann::json solution_events = nlohmann::json::array();
    nlohmann::json astar_stats = nlohmann::json::object();
    double roadmap_build_seconds = 0.0;
    double tensor_search_budget_seconds = timeLimit;
    const auto finalizePlannerStats = [&]() {
        nlohmann::json stats = nlohmann::json::object();
        stats["cost_metric"] = costMetricName(cost_metric_);
        stats["tensor_search_mode"] = tensorSearchModeName(tensor_search_mode_);
        stats["stop_at_first_solution"] = stop_at_first_solution_;
        stats["use_star_rewiring"] = use_star_rewiring_;
        stats["roadmap_size"] = roadmap_size_;
        stats["iterations_per_batch"] = iterations_per_batch_;
        stats["roadmap_build_time_excluded_from_budget"] =
            exclude_roadmap_build_time_from_budget_;
        stats["roadmap_build_time_seconds"] = roadmap_build_seconds;
        stats["tensor_search_budget_seconds"] =
            (tensor_search_mode_ == TensorSearchMode::AStar ||
             tensor_search_mode_ == TensorSearchMode::LazyAStar)
                ? nlohmann::json(nullptr)
                : nlohmann::json(tensor_search_budget_seconds);
        stats["tensor_search_unbounded"] =
            tensor_search_mode_ == TensorSearchMode::AStar ||
            tensor_search_mode_ == TensorSearchMode::LazyAStar;
        stats["accounted_solve_time_seconds"] =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - accounting_start_time)
                .count();
        stats["total_wall_time_seconds"] =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time)
                .count();
        nlohmann::json roadmap_vertices = nlohmann::json::array();
        for (const auto &roadmap : roadmaps_)
            roadmap_vertices.push_back(roadmap.vertices.size());
        stats["roadmap_vertices"] = roadmap_vertices;
        stats["solution_cost"] =
            std::isfinite(best_solution_cost)
                ? nlohmann::json(best_solution_cost)
                : nlohmann::json(nullptr);
        stats["solution_events"] = solution_events;
        stats["num_solution_events"] = solution_events.size();
        if (tensor_search_mode_ == TensorSearchMode::AStar)
            stats["astar"] = astar_stats;
        if (tensor_search_mode_ == TensorSearchMode::LazyAStar)
            stats["lazy_astar"] = astar_stats;
        setPlannerStatsJson(std::move(stats));
    };

    using clock = std::chrono::steady_clock;
    const auto initial_solve_deadline =
        start_time +
        std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(timeLimit));

    // Phase 1: PRM* per robot until roadmap_size_ vertices. Randomized dRRT
    // can still charge this to solve(timeLimit); A* mode intentionally builds
    // the requested graphs and then searches to completion.
    const auto roadmap_build_start = clock::now();
    buildRoadmaps((exclude_roadmap_build_time_from_budget_ ||
                   tensor_search_mode_ == TensorSearchMode::AStar ||
                   tensor_search_mode_ == TensorSearchMode::LazyAStar)
                      ? std::optional<clock::time_point>{}
                      : std::optional<clock::time_point>{initial_solve_deadline});
    roadmap_build_seconds =
        std::chrono::duration<double>(clock::now() - roadmap_build_start).count();

    if (exclude_roadmap_build_time_from_budget_)
        accounting_start_time = clock::now();

    const auto solve_deadline =
        accounting_start_time +
        std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(timeLimit));

    {
        auto single_space = problem_->createStateSpace(0);
        longest_valid_segment_cache_ =
            0.01 * single_space->getMaximumExtent();
        if (longest_valid_segment_cache_ < 1e-12)
            longest_valid_segment_cache_ = 1e-9;
    }

    // Validate roadmaps
    for (int r = 0; r < n; ++r) {
        if (roadmaps_[r].vertices.empty()) {
            finalizePlannerStats();
            return ompl::base::PlannerStatus::TIMEOUT;
        }
    }

    // Phase 2: Discrete RRT over tensor product graph
    CompositeVertex start_v, goal_v;
    for (int r = 0; r < n; ++r) {
        start_v.vertex_ids.push_back(roadmaps_[r].start_vertex);
        goal_v.vertex_ids.push_back(roadmaps_[r].goal_vertex);
    }

    if (tensor_search_mode_ == TensorSearchMode::AStar) {
        const auto status = solveTensorAStar(
            start_v, goal_v, accounting_start_time, best_solution_cost,
            solution_events, astar_stats);
        finalizePlannerStats();
        return status;
    }

    if (tensor_search_mode_ == TensorSearchMode::LazyAStar) {
        const auto status = solveTensorLazyAStar(
            start_v, goal_v, accounting_start_time, best_solution_cost,
            solution_events, astar_stats);
        finalizePlannerStats();
        return status;
    }

    // Initialize tree (Algorithm 1: T.init(S))
    TreeNode root;
    root.vertex = start_v;
    root.parent = -1;
    root.cost = 0.0;
    tree_.push_back(root);

    std::unordered_map<CompositeVertex, int, CompositeVertexHash> vertex_to_node;
    vertex_to_node[start_v] = 0;

    const int nit = std::max(1, iterations_per_batch_);

    uint64_t rrt_iter = 0;

    while (clock::now() < solve_deadline) {
        for (int i = 0; i < nit; ++i) {
            if (clock::now() >= solve_deadline)
                break;

            ++rrt_iter;

            // Algorithm 2: random composite sample Q_rand
            std::vector<std::vector<double>> q_rand(n);
            for (int r = 0; r < n; ++r) {
                int ndof = problem_->robot(r).model->numJoints();
                q_rand[r].resize(ndof);
                for (int d = 0; d < ndof; ++d) {
                    double lo = problem_->robot(r).model->jointLower(d);
                    double hi = problem_->robot(r).model->jointUpper(d);
                    q_rand[r][d] = tensor_phase_rng_.uniformReal(lo, hi);
                }
            }

            int near_idx = findNearestNode(q_rand);
            CompositeVertex v_near = tree_[near_idx].vertex;

            CompositeVertex v_new = directionOracle(v_near, q_rand);

            if (v_new == v_near) {
                continue;
            }

            if (!isCompositeEdgeValid(v_near, v_new)) {
                continue;
            }

            if (vertex_to_node.count(v_new)) {
                continue;
            }

            const double near_edge_cost = edgeCost(v_near, v_new);
            const int parent_idx =
                chooseBestParent(v_new, near_idx, near_edge_cost);

            TreeNode node;
            node.vertex = v_new;
            node.parent = parent_idx;
            node.cost = tree_[parent_idx].cost + edgeCost(tree_[parent_idx].vertex,
                                                          v_new);
            int new_idx = static_cast<int>(tree_.size());
            tree_.push_back(node);
            addChild(parent_idx, new_idx);
            vertex_to_node[v_new] = new_idx;
            rewireFrom(new_idx);
        }

        double candidate_cost = std::numeric_limits<double>::infinity();
        std::optional<std::vector<CompositeVertex>> cv_path = connectToTarget(
            goal_v, vertex_to_node, rrt_iter, solve_deadline, &candidate_cost);

        if (cv_path.has_value()) {
            if (candidate_cost + 1e-12 < best_solution_cost) {
                solution_paths_ = pathsFromCompositePath(*cv_path);
                setSolutionMetricsFromPaths(solution_paths_);
                best_solution_cost = candidate_cost;

                solution_events.push_back({
                    {"elapsed_seconds",
                     std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - accounting_start_time)
                         .count()},
                    {"cost_metric", costMetricName(cost_metric_)},
                    {"solution_cost", candidate_cost},
                    {"makespan_timesteps",
                     makespanTimesteps() ? nlohmann::json(*makespanTimesteps())
                                         : nlohmann::json(nullptr)},
                    {"sum_of_cost_timesteps",
                     sumOfCostTimesteps() ? nlohmann::json(*sumOfCostTimesteps())
                                          : nlohmann::json(nullptr)},
                    {"kind",
                     solution_events.empty() ? "first_solution"
                                             : "drrt_star_improvement"},
                });
            }

            if (!stop_at_first_solution_)
                continue;

            setSolutionMetricsFromPaths(solution_paths_);
            finalizePlannerStats();
            return ompl::base::PlannerStatus::EXACT_SOLUTION;
        }
    }

    if (!solution_paths_.empty()) {
        finalizePlannerStats();
        return ompl::base::PlannerStatus::EXACT_SOLUTION;
    }

    finalizePlannerStats();
    return ompl::base::PlannerStatus::TIMEOUT;
}

std::vector<Path> MRdRRT::getSolutionPaths() const {
    return solution_paths_;
}

} // namespace comotion
