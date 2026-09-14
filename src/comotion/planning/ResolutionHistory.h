#pragma once

#include "comotion/collision/ConflictChecker.h"
#include "comotion/planning/ExpansionScheduleState.h"
#include "comotion/planning/MRMPSubproblem.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace comotion {

enum class ResolutionSolver {
    None,
    PrioritizedSTRRT,
    CompositeRRT,
};

enum class ResolutionOutcome {
    Success,
    Failure,
};

struct ResolutionAttempt {
    Conflict conflict;

    // Historical attempt state. Later expansion must construct a fresh
    // MRMPSubproblem rather than modify this retained attempt.
    std::shared_ptr<const MRMPSubproblem> subproblem;

    std::uint64_t attempt_index = 0;
    std::uint32_t attempt_root_seed = 0;

    ResolutionSolver solver = ResolutionSolver::None;
    ResolutionOutcome outcome = ResolutionOutcome::Failure;
};

struct ResolutionAttempts {
    Conflict conflict;
    std::vector<ResolutionAttempt> attempts;

    // Cached temporal-expansion state derived from the ordered attempts.
    ExpansionScheduleState expansion_schedule_state;
    std::size_t expansion_attempts_applied = 0;
    std::optional<std::pair<int, int>> next_temporal_window;

    void append(ResolutionAttempt attempt) {
        if (!attempt.subproblem) {
            throw std::logic_error(
                "Resolution attempt has no subproblem");
        }

        if (attempt.attempt_index != attempts.size()) {
            throw std::logic_error(
                "Resolution attempt index is out of order");
        }

        attempts.push_back(std::move(attempt));
    }

    bool resolved() const noexcept {
        return !attempts.empty() &&
               attempts.back().outcome ==
                   ResolutionOutcome::Success;
    }
};

struct ResolvedConflicts {
    std::vector<ResolutionAttempts> conflicts;

    void append(ResolutionAttempts resolution_attempts) {
        if (!resolution_attempts.resolved()) {
            throw std::logic_error(
                "Resolved conflict history must end in success");
        }

        conflicts.push_back(
            std::move(resolution_attempts));
    }

    void clear() noexcept {
        conflicts.clear();
    }

    bool empty() const noexcept {
        return conflicts.empty();
    }

    std::size_t size() const noexcept {
        return conflicts.size();
    }
};

} // namespace comotion