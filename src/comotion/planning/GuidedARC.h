#pragma once

#include "comotion/planning/ARC.h"
#include "comotion/planning/MRMPSubproblem.h"
#include "comotion/planning/ResolutionHistory.h"

#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

namespace comotion {

class GuidedARC : public ARC {
public:
    enum class SubproblemExpansionMode {
        Temporal,
        CSpace,
    };

    enum class RobotSelectionPolicy {
        ArcRepairHistory,
        HistoricalDirectNeighbors,
        CurrentConflictComponent,
    };

    void setRobotSelectionPolicy(RobotSelectionPolicy policy) noexcept {
        robot_selection_policy_ = policy;
    }

    RobotSelectionPolicy robotSelectionPolicy() const noexcept {
        return robot_selection_policy_;
    }

    void setSubproblemExpansionMode(SubproblemExpansionMode mode) {
        expansion_mode_ = mode;
    }

    SubproblemExpansionMode subproblemExpansionMode() const noexcept {
        return expansion_mode_;
    }

    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::string name() const override { return "GuidedARC"; }

    const ResolvedConflicts &resolvedConflicts() const noexcept {
        return resolved_conflicts_;
    }

protected:
    struct HistoricalConflictRange {
        int earliest_t = 0;
        int latest_t = 0;
    };

    using EncounteredConflictNeighbors =
        std::vector<std::map<int, HistoricalConflictRange>>;

    struct CurrentConflictSnapshot {
        EncounteredConflictNeighbors neighbors;
        bool complete = false;
    };

    static std::pair<int, int>
    coveredTimesteps(const Conflict &conflict) {
        const int covered_end =
            conflict.alpha > 0.0 &&
                    conflict.timestep < std::numeric_limits<int>::max()
                ? conflict.timestep + 1
                : conflict.timestep;
        return {conflict.timestep, covered_end};
    }

    static void applyHistoricalDirectNeighbors(
        const Conflict &conflict,
        const EncounteredConflictNeighbors &neighbors,
        int window_padding,
        SubproblemConflict &expanded) {
        if (conflict.robot_i < 0 || conflict.robot_j < 0 ||
            static_cast<std::size_t>(conflict.robot_i) >= neighbors.size() ||
            static_cast<std::size_t>(conflict.robot_j) >= neighbors.size()) {
            throw std::out_of_range(
                "Conflict robot index is outside historical adjacency");
        }

        std::set<int> selected{
            conflict.robot_i,
            conflict.robot_j,
        };
        auto [earliest_t, latest_t] = coveredTimesteps(conflict);

        const auto include_direct_neighbors = [&](int robot) {
            for (const auto &[neighbor, range] :
                 neighbors[static_cast<std::size_t>(robot)]) {
                selected.insert(neighbor);
                earliest_t = std::min(earliest_t, range.earliest_t);
                latest_t = std::max(latest_t, range.latest_t);
            }
        };

        include_direct_neighbors(conflict.robot_i);
        include_direct_neighbors(conflict.robot_j);

        expanded.robots.assign(selected.begin(), selected.end());
        expanded.window_begin_t =
            std::max(0, earliest_t - window_padding);
        const auto padded_end =
            static_cast<long long>(latest_t) + window_padding;
        expanded.window_end_t = static_cast<int>(std::min(
            padded_end,
            static_cast<long long>(std::numeric_limits<int>::max())));
        expanded.expansion_trace.clear();
    }

    static void recordHistoricalConflict(
        const Conflict &conflict,
        EncounteredConflictNeighbors &neighbors) {
        const auto [earliest_t, latest_t] = coveredTimesteps(conflict);

        const auto record_direction = [&](int robot, int neighbor) {
            auto &ranges = neighbors[static_cast<std::size_t>(robot)];
            const auto [it, inserted] = ranges.try_emplace(
                neighbor,
                HistoricalConflictRange{earliest_t, latest_t});
            if (!inserted) {
                it->second.earliest_t =
                    std::min(it->second.earliest_t, earliest_t);
                it->second.latest_t =
                    std::max(it->second.latest_t, latest_t);
            }
        };

        record_direction(conflict.robot_i, conflict.robot_j);
        record_direction(conflict.robot_j, conflict.robot_i);
    }

    static void recordCurrentConflict(
        const Conflict &conflict,
        CurrentConflictSnapshot &snapshot) {
        recordHistoricalConflict(conflict, snapshot.neighbors);
    }

    static void applyCurrentConflictComponent(
        const Conflict &conflict,
        const CurrentConflictSnapshot &snapshot,
        int window_padding,
        SubproblemConflict &expanded) {
        if (!snapshot.complete) {
            throw std::logic_error(
                "Current conflict snapshot is incomplete");
        }
        if (conflict.robot_i < 0 || conflict.robot_j < 0 ||
            static_cast<std::size_t>(conflict.robot_i) >=
                snapshot.neighbors.size() ||
            static_cast<std::size_t>(conflict.robot_j) >=
                snapshot.neighbors.size()) {
            throw std::out_of_range(
                "Conflict robot index is outside current snapshot");
        }

        std::set<int> selected{
            conflict.robot_i,
            conflict.robot_j,
        };
        std::vector<int> frontier(selected.begin(), selected.end());
        for (std::size_t next = 0; next < frontier.size(); ++next) {
            const int robot = frontier[next];
            for (const auto &[neighbor, range] :
                 snapshot.neighbors[static_cast<std::size_t>(robot)]) {
                (void)range;
                if (neighbor < 0 ||
                    static_cast<std::size_t>(neighbor) >=
                        snapshot.neighbors.size()) {
                    throw std::out_of_range(
                        "Current snapshot neighbor index is invalid");
                }
                if (selected.insert(neighbor).second)
                    frontier.push_back(neighbor);
            }
        }

        auto [earliest_t, latest_t] = coveredTimesteps(conflict);
        for (const int robot : selected) {
            for (const auto &[neighbor, range] :
                 snapshot.neighbors[static_cast<std::size_t>(robot)]) {
                if (selected.count(neighbor) == 0)
                    continue;
                earliest_t = std::min(earliest_t, range.earliest_t);
                latest_t = std::max(latest_t, range.latest_t);
            }
        }

        expanded.robots.assign(selected.begin(), selected.end());
        expanded.window_begin_t =
            std::max(0, earliest_t - window_padding);
        const auto padded_end =
            static_cast<long long>(latest_t) + window_padding;
        expanded.window_end_t = static_cast<int>(std::min(
            padded_end,
            static_cast<long long>(std::numeric_limits<int>::max())));
        expanded.expansion_trace.clear();
    }

    struct SubproblemSolveResult {
        std::vector<Path> paths;
        ResolutionSolver solver = ResolutionSolver::None;
        ResolutionOutcome outcome = ResolutionOutcome::Failure;
    };

    std::optional<MRMPSubproblem> createSubProblem(
        const SubproblemConflict &conflict,
        const std::vector<Path> &paths,
        ResolutionAttempts &resolution_attempts) const;

    std::optional<MRMPSubproblem> createTemporalSubProblem(
        const SubproblemConflict &conflict,
        const std::vector<Path> &paths,
        ResolutionAttempts &resolution_attempts) const;

    std::optional<MRMPSubproblem> createCSpaceSubProblem(
        const SubproblemConflict &conflict,
        const std::vector<Path> &paths,
        const ResolutionAttempts &resolution_attempts) const;

    SubproblemSolveResult solveSubProblem(
        MRMPSubproblem subproblem,
        double time_limit,
        std::uint32_t attempt_root_seed,
        ResolutionAttempts &resolution_attempts) const;

private:
    RobotSelectionPolicy robot_selection_policy_ =
        RobotSelectionPolicy::ArcRepairHistory;
    SubproblemExpansionMode expansion_mode_ =
        SubproblemExpansionMode::Temporal;
    ResolvedConflicts resolved_conflicts_;
};

} // namespace comotion