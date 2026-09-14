#include "comotion/planning/GuidedARC.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/robot/FlyingSphere.h"

#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

class GuidedArcProbe : public comotion::GuidedARC {
public:
    using HistoricalNeighbors = EncounteredConflictNeighbors;
    using ConflictSnapshot = CurrentConflictSnapshot;

    static void applyHistoricalSelection(
        const comotion::Conflict &conflict,
        const HistoricalNeighbors &neighbors,
        int window_padding,
        comotion::SubproblemConflict &expanded) {
        applyHistoricalDirectNeighbors(
            conflict, neighbors, window_padding, expanded);
    }

    static void recordConflict(
        const comotion::Conflict &conflict,
        HistoricalNeighbors &neighbors) {
        recordHistoricalConflict(conflict, neighbors);
    }

    static void recordCurrent(
        const comotion::Conflict &conflict,
        ConflictSnapshot &snapshot) {
        recordCurrentConflict(conflict, snapshot);
    }

    static void applyCurrentSelection(
        const comotion::Conflict &conflict,
        const ConflictSnapshot &snapshot,
        int window_padding,
        comotion::SubproblemConflict &expanded) {
        applyCurrentConflictComponent(
            conflict, snapshot, window_padding, expanded);
    }

    std::optional<comotion::MRMPSubproblem> createTemporal(
        const comotion::SubproblemConflict &conflict,
        const std::vector<comotion::Path> &paths,
        comotion::ResolutionAttempts &history) const {
        return createTemporalSubProblem(conflict, paths, history);
    }

    comotion::ResolutionOutcome recordCancelledFailure(
        comotion::MRMPSubproblem subproblem,
        std::uint32_t attempt_root_seed,
        comotion::ResolutionAttempts &history) const {
        return solveSubProblem(
                   std::move(subproblem), 1.0, attempt_root_seed, history)
            .outcome;
    }
};

std::shared_ptr<comotion::FlyingSphere> makeRobot() {
    return std::make_shared<comotion::FlyingSphere>(
        0.25,
        std::vector<double>{-10.0, -10.0, 0.0},
        std::vector<double>{10.0, 10.0, 1.5});
}

comotion::Path makeStationaryPath(
    const std::vector<double> &configuration,
    std::size_t state_count) {
    comotion::Path path;
    for (std::size_t index = 0; index < state_count; ++index)
        path.push_back(configuration);
    path.markDenseTimestepsImplicit();
    return path;
}

bool expectTrue(const std::string &label, bool value) {
    if (!value) {
        std::cerr
            << "guided_arc_temporal_expansion_regression: "
            << label << " expected true\n";
        return false;
    }
    return true;
}

bool expectEqual(const std::string &label,
                 std::size_t actual,
                 std::size_t expected) {
    if (actual != expected) {
        std::cerr
            << "guided_arc_temporal_expansion_regression: "
            << label << " expected " << expected
            << " got " << actual << "\n";
        return false;
    }
    return true;
}

bool expectRobots(const std::string &label,
                  const std::vector<int> &actual,
                  const std::vector<int> &expected) {
    if (actual != expected) {
        std::cerr
            << "guided_arc_temporal_expansion_regression: "
            << label << " robot selection differs\n";
        return false;
    }
    return true;
}

template <typename Operation>
bool expectThrow(const std::string &label, Operation operation) {
    try {
        operation();
    } catch (const std::exception &) {
        return true;
    }
    std::cerr
        << "guided_arc_temporal_expansion_regression: "
        << label << " expected an exception\n";
    return false;
}

bool expectWindow(const std::string &label,
                  const comotion::MRMPSubproblem &subproblem,
                  int expected_start,
                  int expected_end) {
    if (subproblem.window_start_t != expected_start ||
        subproblem.window_end_t != expected_end) {
        std::cerr
            << "guided_arc_temporal_expansion_regression: "
            << label << " expected ["
            << expected_start << ", " << expected_end
            << "] got [" << subproblem.window_start_t
            << ", " << subproblem.window_end_t << "]\n";
        return false;
    }
    return true;
}

bool testHistoricalDirectNeighborSelection() {
    GuidedArcProbe planner;
    if (!expectTrue(
            "ARC repair history remains the default robot selection policy",
            planner.robotSelectionPolicy() ==
                comotion::GuidedARC::RobotSelectionPolicy::ArcRepairHistory)) {
        return false;
    }

    GuidedArcProbe::HistoricalNeighbors neighbors(5);
    GuidedArcProbe::recordConflict(
        comotion::Conflict{1, 2, 100}, neighbors);
    GuidedArcProbe::recordConflict(
        comotion::Conflict{2, 3, 180}, neighbors);
    GuidedArcProbe::recordConflict(
        comotion::Conflict{3, 4, 220}, neighbors);
    GuidedArcProbe::recordConflict(
        comotion::Conflict{1, 2, 250, 0.5}, neighbors);

    comotion::SubproblemConflict expanded;
    expanded.expansion_trace.push_back({});
    GuidedArcProbe::applyHistoricalSelection(
        comotion::Conflict{1, 2, 200}, neighbors, 10, expanded);

    if (!expectRobots(
            "selection includes only direct neighbors of the selected pair",
            expanded.robots,
            std::vector<int>{1, 2, 3}) ||
        !expectTrue(
            "historical selection clears ARC repair-team expansion trace",
            expanded.expansion_trace.empty()) ||
        !expectTrue(
            "historical window covers repeated pair collisions and segment endpoint",
            expanded.window_begin_t == 90 &&
                expanded.window_end_t == 261)) {
        return false;
    }

    GuidedArcProbe::HistoricalNeighbors fresh_neighbors(5);
    comotion::SubproblemConflict fresh_expanded;
    GuidedArcProbe::applyHistoricalSelection(
        comotion::Conflict{1, 2, 200}, fresh_neighbors, 10,
        fresh_expanded);

    return expectRobots(
               "fresh solve history contains only the selected pair",
               fresh_expanded.robots,
               std::vector<int>{1, 2}) &&
           expectTrue(
               "fresh solve history uses only the current padded window",
               fresh_expanded.window_begin_t == 190 &&
                   fresh_expanded.window_end_t == 210);
}

bool testCurrentConflictComponentSelection() {
    GuidedArcProbe::ConflictSnapshot snapshot;
    snapshot.neighbors.resize(6);
    GuidedArcProbe::recordCurrent(
        comotion::Conflict{0, 1, 100}, snapshot);
    GuidedArcProbe::recordCurrent(
        comotion::Conflict{1, 2, 140}, snapshot);
    GuidedArcProbe::recordCurrent(
        comotion::Conflict{2, 3, 200}, snapshot);
    GuidedArcProbe::recordCurrent(
        comotion::Conflict{2, 3, 250, 0.5}, snapshot);
    GuidedArcProbe::recordCurrent(
        comotion::Conflict{4, 5, 20}, snapshot);

    comotion::SubproblemConflict expanded;
    expanded.expansion_trace.push_back({});
    const comotion::Conflict selected_conflict{0, 1, 120};

    if (!expectThrow(
            "incomplete current snapshot",
            [&]() {
                GuidedArcProbe::applyCurrentSelection(
                    selected_conflict, snapshot, 10, expanded);
            })) {
        return false;
    }

    snapshot.complete = true;
    GuidedArcProbe::applyCurrentSelection(
        selected_conflict, snapshot, 10, expanded);

    if (!expectRobots(
            "current component includes transitive but not disconnected robots",
            expanded.robots,
            std::vector<int>{0, 1, 2, 3}) ||
        !expectTrue(
            "current component clears ARC repair-team expansion trace",
            expanded.expansion_trace.empty()) ||
        !expectTrue(
            "current component window covers every repeated component collision",
            expanded.window_begin_t == 90 &&
                expanded.window_end_t == 261)) {
        return false;
    }

    return expectThrow(
        "selected robot outside current snapshot",
        [&]() {
            comotion::SubproblemConflict invalid_expanded;
            GuidedArcProbe::applyCurrentSelection(
                comotion::Conflict{0, 6, 120}, snapshot, 10,
                invalid_expanded);
        });
}

bool testFailedAttemptsExpandThroughGlobalWindow() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(1);
    problem->setVmax(1.0);

    const std::vector<double> robot_zero{-4.0, 0.0, 0.75};
    const std::vector<double> robot_one{4.0, 0.0, 0.75};

    problem->addRobot(makeRobot(), robot_zero, robot_zero);
    problem->addRobot(makeRobot(), robot_one, robot_one);

    const std::vector<comotion::Path> paths{
        makeStationaryPath(robot_zero, 11),
        makeStationaryPath(robot_one, 11),
    };

    comotion::SubproblemConflict conflict;
    conflict.robots = {0, 1};
    conflict.conflict_timestep = 5;
    conflict.window_begin_t = 4;
    conflict.window_end_t = 6;
    conflict.seed_robot_i = 0;
    conflict.seed_robot_j = 1;

    comotion::ResolutionAttempts history;
    history.conflict.robot_i = 0;
    history.conflict.robot_j = 1;
    history.conflict.timestep = 5;

    GuidedArcProbe planner;
    planner.setProblem(problem);
    planner.setSubproblemExpansionMode(
        comotion::GuidedARC::SubproblemExpansionMode::Temporal);
    planner.setExpansionPolicy(
        comotion::ARC::ExpansionPolicy::Linear);
    planner.setExpansionStep(2.0);

    // Cancellation makes every solveSubProblem call fail before invoking a
    // randomized local planner, while preserving normal attempt recording.
    planner.setCancellationCallback([] { return true; });

    auto initial =
        planner.createTemporal(conflict, paths, history);
    if (!expectTrue("initial construction succeeds",
                    initial.has_value()) ||
        !expectWindow("initial window", *initial, 4, 6))
        return false;

    if (!expectTrue(
            "initial solve attempt fails deterministically",
            planner.recordCancelledFailure(
                std::move(*initial), 101, history) ==
                comotion::ResolutionOutcome::Failure) ||
        !expectEqual("one failed attempt is recorded",
                     history.attempts.size(), 1) ||
        !expectTrue("recorded attempt remains failed",
                    history.attempts[0].outcome ==
                        comotion::ResolutionOutcome::Failure) ||
        !expectWindow("recorded initial window",
                      *history.attempts[0].subproblem, 4, 6))
        return false;

    auto expanded =
        planner.createTemporal(conflict, paths, history);
    if (!expectTrue("expanded construction succeeds",
                    expanded.has_value()) ||
        !expectWindow("first expanded window", *expanded, 2, 8) ||
        !expectEqual("construction retains first attempt",
                     history.attempts.size(), 1) ||
        !expectWindow("retained initial window remains unchanged",
                      *history.attempts[0].subproblem, 4, 6))
        return false;

    if (!expectTrue(
            "expanded solve attempt fails deterministically",
            planner.recordCancelledFailure(
                std::move(*expanded), 102, history) ==
                comotion::ResolutionOutcome::Failure) ||
        !expectEqual("two failed attempts are recorded",
                     history.attempts.size(), 2))
        return false;

    auto global =
        planner.createTemporal(conflict, paths, history);
    if (!expectTrue("global construction still succeeds",
                    global.has_value()) ||
        !expectWindow("global window", *global, 0, 10) ||
        !expectTrue("global window spans full horizon",
                    global->spansGlobalTime()))
        return false;

    if (!expectTrue(
            "global solve attempt fails deterministically",
            planner.recordCancelledFailure(
                std::move(*global), 103, history) ==
                comotion::ResolutionOutcome::Failure) ||
        !expectEqual("global failure is recorded",
                     history.attempts.size(), 3) ||
        !expectTrue("recorded global attempt spans full horizon",
                    history.attempts.back()
                        .subproblem->spansGlobalTime()))
        return false;

    const auto exhausted =
        planner.createTemporal(conflict, paths, history);
    if (!expectTrue(
            "construction stops after failed global attempt",
            !exhausted.has_value()))
        return false;

    return true;
}

} // namespace

int main() {
    if (!testHistoricalDirectNeighborSelection())
        return 1;
    if (!testCurrentConflictComponentSelection())
        return 1;
    if (!testFailedAttemptsExpandThroughGlobalWindow())
        return 1;

    std::cout
        << "guided_arc_temporal_expansion_regression: OK\n";
    return 0;
}