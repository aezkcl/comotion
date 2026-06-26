#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/CooperativeCompositeRRT.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/OrParallelPlanner.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/planning/PlanningSeed.h"
#include "comotion/robot/FlyingSphere.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace ob = ompl::base;

namespace {

enum class FakeOutcome { Timeout, Approximate, Exact };

struct FakeBehavior {
    FakeOutcome outcome = FakeOutcome::Timeout;
    int sleep_ms = 0;
    double result_marker = 0.0;
};

bool expectTrue(const std::string &label, bool condition) {
    if (!condition) {
        std::cerr << "or_parallel_planner_regression: " << label << "\n";
        return false;
    }
    return true;
}

comotion::Path makeMarkerPath(double marker) {
    comotion::Path path;
    path.push_back({0.0});
    path.push_back({marker});
    return path;
}

double pathMarker(const std::vector<comotion::Path> &paths) {
    if (paths.empty() || paths.front().empty() ||
        paths.front().back().empty()) {
        throw std::runtime_error("missing marker path");
    }
    return paths.front().back().front();
}

class SeededFakePlanner : public comotion::MultiRobotPlanner {
public:
    SeededFakePlanner(std::uint32_t base_seed, std::vector<FakeBehavior> behaviors,
                      std::string label)
        : base_seed_(base_seed), behaviors_(std::move(behaviors)),
          label_(std::move(label)) {}

    ob::PlannerStatus solve(double /*timeLimit*/) override {
        const FakeBehavior &behavior = resolveBehavior();
        if (behavior.sleep_ms > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(behavior.sleep_ms));
        }
        solution_paths_.clear();
        if (behavior.outcome != FakeOutcome::Timeout) {
            solution_paths_.push_back(makeMarkerPath(behavior.result_marker));
        }

        switch (behavior.outcome) {
        case FakeOutcome::Timeout:
            return ob::PlannerStatus::TIMEOUT;
        case FakeOutcome::Approximate:
            return ob::PlannerStatus::APPROXIMATE_SOLUTION;
        case FakeOutcome::Exact:
            return ob::PlannerStatus::EXACT_SOLUTION;
        }
        return ob::PlannerStatus::TIMEOUT;
    }

    std::vector<comotion::Path> getSolutionPaths() const override {
        return solution_paths_;
    }

    std::string name() const override { return label_; }

private:
    const FakeBehavior &resolveBehavior() const {
        if (behaviors_.empty())
            throw std::runtime_error("no fake behaviors configured");
        if (behaviors_.size() == 1 && planningSeed() == base_seed_)
            return behaviors_.front();
        for (std::size_t i = 0; i < behaviors_.size(); ++i) {
            if (planningSeed() ==
                comotion::orParallelWorkerPlanningSeed(base_seed_,
                                                   static_cast<int>(i))) {
                return behaviors_[i];
            }
        }
        throw std::runtime_error("unexpected fake planner seed");
    }

    std::uint32_t base_seed_;
    std::vector<FakeBehavior> behaviors_;
    std::vector<comotion::Path> solution_paths_;
    std::string label_;
};

std::shared_ptr<comotion::MultiRobotProblem> makeProblem() {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(8);
    problem->setVmax(1.0);
    auto robot =
        std::make_shared<comotion::FlyingSphere>(0.5, -5.0, 5.0);
    problem->addRobot(robot, {-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0});
    return problem;
}

bool expectOrMetadata(const comotion::OrParallelPlanner &planner,
                      unsigned worker_processes,
                      const std::string &base_planner,
                      int winner_index) {
    const auto &stats = planner.plannerStatsJson();
    if (!expectTrue("OR metadata present", stats.contains("or_parallel")))
        return false;
    const auto &or_stats = stats["or_parallel"];
    if (!expectTrue("OR metadata worker count",
                    or_stats["worker_processes"].get<unsigned>() ==
                        worker_processes))
        return false;
    if (!expectTrue("OR metadata base planner",
                    or_stats["base_planner"].get<std::string>() ==
                        base_planner))
        return false;
    if (winner_index >= 0) {
        return expectTrue("OR metadata winner index",
                          or_stats["winner_index"].get<int>() ==
                              winner_index);
    }
    return expectTrue("OR metadata winner index",
                      or_stats["winner_index"].get<int>() >= 0);
}

bool testFirstExactWins() {
    constexpr std::uint32_t kSeed = 17;
    auto problem = makeProblem();

    comotion::OrParallelPlanner planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(kSeed);
    planner.setBasePlannerName("Fake");
    planner.setWorkerProcesses(2);
    planner.setPlannerFactory([=]() {
        return std::make_shared<SeededFakePlanner>(
            kSeed,
            std::vector<FakeBehavior>{
                {FakeOutcome::Exact, 20, 11},
                {FakeOutcome::Exact, 120, 22}},
            "SeededFakePlanner");
    });

    const auto status = planner.solve(1.0);
    if (!expectTrue("first exact worker returns exact",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }
    if (!expectTrue("earliest exact path wins",
                    std::fabs(pathMarker(planner.getSolutionPaths()) - 11.0) <
                        1e-9)) {
        return false;
    }
    return expectOrMetadata(planner, 2, "Fake", 0);
}

bool testApproximateDoesNotPreemptExact() {
    constexpr std::uint32_t kSeed = 23;
    auto problem = makeProblem();

    comotion::OrParallelPlanner planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(kSeed);
    planner.setBasePlannerName("Fake");
    planner.setWorkerProcesses(2);
    planner.setPlannerFactory([=]() {
        return std::make_shared<SeededFakePlanner>(
            kSeed,
            std::vector<FakeBehavior>{
                {FakeOutcome::Approximate, 10, 5},
                {FakeOutcome::Exact, 60, 9}},
            "SeededFakePlanner");
    });

    const auto status = planner.solve(1.0);
    if (!expectTrue("later exact beats earlier approximate",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }
    if (!expectTrue("exact path chosen over approximate fallback",
                    std::fabs(pathMarker(planner.getSolutionPaths()) - 9.0) <
                        1e-9)) {
        return false;
    }
    return true;
}

bool testApproximateFallbackWhenNoExact() {
    constexpr std::uint32_t kSeed = 31;
    auto problem = makeProblem();

    comotion::OrParallelPlanner planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(kSeed);
    planner.setBasePlannerName("Fake");
    planner.setWorkerProcesses(2);
    planner.setPlannerFactory([=]() {
        return std::make_shared<SeededFakePlanner>(
            kSeed,
            std::vector<FakeBehavior>{
                {FakeOutcome::Approximate, 10, 13},
                {FakeOutcome::Timeout, 40, 29}},
            "SeededFakePlanner");
    });

    const auto status = planner.solve(1.0);
    if (!expectTrue("approximate fallback returned when no exact exists",
                    status == ob::PlannerStatus::APPROXIMATE_SOLUTION)) {
        return false;
    }
    if (!expectTrue("first approximate retained as fallback",
                    std::fabs(pathMarker(planner.getSolutionPaths()) - 13.0) <
                        1e-9)) {
        return false;
    }
    return true;
}

bool testSingleWorkerPassthrough() {
    constexpr std::uint32_t kSeed = 47;
    auto problem = makeProblem();

    comotion::OrParallelPlanner planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(kSeed);
    planner.setBasePlannerName("Fake");
    planner.setWorkerProcesses(1);
    planner.setPlannerFactory([=]() {
        return std::make_shared<SeededFakePlanner>(
            kSeed,
            std::vector<FakeBehavior>{{FakeOutcome::Exact, 0, 31}},
            "SeededFakePlanner");
    });

    const auto status = planner.solve(1.0);
    if (!expectTrue("single worker passthrough returns exact",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }
    if (!expectTrue("single worker passthrough preserves result",
                    std::fabs(pathMarker(planner.getSolutionPaths()) - 31.0) <
                        1e-9)) {
        return false;
    }
    return true;
}

bool testRealCompositeRrtSmoke() {
    constexpr std::uint32_t kSeed = 53;
    comotion::seedOmplGlobalFromUserPlanningSeed(kSeed);

    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(16);
    problem->setVmax(1.0);
    auto robot =
        std::make_shared<comotion::FlyingSphere>(0.5, -5.0, 5.0);
    problem->addRobot(robot, {-4.0, 0.0, 0.0}, {4.0, 0.0, 0.0});

    comotion::OrParallelPlanner planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(kSeed);
    planner.setBasePlannerName("CompositeRRT");
    planner.setWorkerProcesses(2);
    planner.setPlannerFactory([]() {
        auto inner = std::make_shared<comotion::CompositeRRT>();
        inner->setSimplifySolution(false);
        return inner;
    });

    const auto status = planner.solve(2.0);
    if (!expectTrue("real CompositeRRT OR wrapper returns exact",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }

    const auto paths = planner.getSolutionPaths();
    if (!expectTrue("real CompositeRRT OR wrapper returns one path",
                    paths.size() == 1 && paths[0].size() >= 2)) {
        return false;
    }
    if (!expectTrue("real CompositeRRT path starts near start",
                    std::fabs(paths[0].front()[0] + 4.0) < 1e-6)) {
        return false;
    }
    return expectTrue("real CompositeRRT path ends near goal",
                      std::fabs(paths[0].back()[0] - 4.0) < 1e-6);
}

bool testRealCooperativeCompositeRrtHybridSmoke() {
    constexpr std::uint32_t kSeed = 59;
    comotion::seedOmplGlobalFromUserPlanningSeed(kSeed);

    auto problem = std::make_shared<comotion::MultiRobotProblem>(
        comotion::CollisionChecker::Backend::Spheres);
    problem->setResolution(16);
    problem->setVmax(1.0);
    auto robot =
        std::make_shared<comotion::FlyingSphere>(0.5, -5.0, 5.0);
    problem->addRobot(robot, {-4.0, 0.0, 0.0}, {4.0, 0.0, 0.0});

    comotion::OrParallelPlanner planner;
    planner.setProblem(problem);
    planner.setPlanningSeed(kSeed);
    planner.setBasePlannerName("CooperativeCompositeRRT");
    planner.setWorkerProcesses(2);
    planner.setPlannerFactory([]() {
        auto inner = std::make_shared<comotion::CooperativeCompositeRRT>();
        inner->setSimplifySolution(false);
        inner->setWorkerThreads(2);
        return inner;
    });

    const auto status = planner.solve(2.0);
    if (!expectTrue("real CooperativeCompositeRRT OR wrapper returns exact",
                    status == ob::PlannerStatus::EXACT_SOLUTION)) {
        return false;
    }
    if (!expectOrMetadata(planner, 2, "CooperativeCompositeRRT", -1)) {
        return false;
    }
    const auto &stats = planner.plannerStatsJson();
    if (!expectTrue("hybrid preserves winner cooperative stats",
                    stats.contains("cooperative_composite_rrt")))
        return false;
    return expectTrue("hybrid reports cooperative threads",
                      stats["cooperative_composite_rrt"]["worker_threads"]
                              .get<unsigned>() == 2);
}

} // namespace

int main() {
    if (!testFirstExactWins())
        return 1;
    if (!testApproximateDoesNotPreemptExact())
        return 1;
    if (!testApproximateFallbackWhenNoExact())
        return 1;
    if (!testSingleWorkerPassthrough())
        return 1;
    if (!testRealCompositeRrtSmoke())
        return 1;
    if (!testRealCooperativeCompositeRrtHybridSmoke())
        return 1;

    std::cout << "or_parallel_planner_regression: OK\n";
    return 0;
}
