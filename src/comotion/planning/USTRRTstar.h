#pragma once

#include "comotion/collision/CollisionChecker.h"
#include "comotion/planning/Path.h"
#include "comotion/robot/RobotModel.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/util/RandomNumbers.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace comotion {

/// STCBS support machinery.
///
/// Installed because STCBS exposes USTRRTstar::RewireMode and related result
/// types in its public configuration and branching data structures.
class USTRRTstar {
public:
    enum class RewireMode { Off, Radius, KNearest };

    struct Params {
        double range = 10.0;
        int max_iterations = 1000;
        int max_samples = 100000;
        double goal_threshold = 0.1;
        RewireMode rewire_mode = RewireMode::KNearest;
        double rewire_radius = 1.0;
        int rewire_k = 10;
        double layer_dt_seconds = 1.0;
    };

    struct BranchConstraint {
        int constrained_agent_id = -1;
        int other_agent_id = -1;
        int timestep = 0;
        double time_seconds = 0.0;
        std::vector<double> other_config;
        const RobotModel *other_model = nullptr;
    };

    struct Motion {
        int index = -1;
        ompl::base::State *state = nullptr;
        Motion *parent = nullptr;
        std::vector<Motion *> children;
        bool active = true;
        bool marked = false;
    };

    struct Result {
        Path raw_path;
        std::vector<double> raw_times_seconds;
        std::vector<int> motion_indices;
        Path dense_path;
        std::vector<int> dense_timestep_to_motion_index;
        double st_cost = std::numeric_limits<double>::infinity();
        double arrival_time_seconds = std::numeric_limits<double>::infinity();
        bool exact = false;
    };

    struct TreeSnapshot {
        using NN = ompl::NearestNeighbors<Motion *>;

        TreeSnapshot();
        ~TreeSnapshot();

        TreeSnapshot(const TreeSnapshot &) = delete;
        TreeSnapshot &operator=(const TreeSnapshot &) = delete;
        TreeSnapshot(TreeSnapshot &&) = delete;
        TreeSnapshot &operator=(TreeSnapshot &&) = delete;

        std::shared_ptr<TreeSnapshot> clone() const;
        void rebuildNearestNeighbors();
        std::size_t activeMotionCount() const;

        std::shared_ptr<ompl::base::SpaceTimeStateSpace> space;
        ompl::base::SpaceInformationPtr si;
        std::shared_ptr<NN> nn;

        const RobotModel *model = nullptr;
        const CollisionChecker *collision_checker = nullptr;
        int agent_index = -1;
        std::size_t resolution = 128;
        double vmax = 1.0;
        Params params;

        std::vector<double> start_config;
        std::vector<double> goal_config;
        double current_time_upper_bound = 1.0;
        double max_time_bound = 1.0;
        std::size_t min_safe_arrival_timestep = 0;
        bool goal_permanently_blocked = false;

        std::vector<BranchConstraint> constraints;
        std::vector<std::unique_ptr<Motion>> motions;
        Motion *start_motion = nullptr;

        /// Persists across `extendTree` / CBS branching so sampling stays repeatable.
        ompl::RNG iteration_rng;

        std::size_t usableMotionCount() const;
        bool isSearchFeasible() const;
    };

    static std::pair<std::shared_ptr<TreeSnapshot>, Result> buildTree(
        int agent_index, const std::vector<double> &start,
        const std::vector<double> &goal, const RobotModel &model,
        const CollisionChecker &collision_checker, const Params &params,
        std::size_t resolution, double vmax, double lambda,
        std::uint32_t planning_seed = 42);

    static Result extractBestPath(const TreeSnapshot &tree, double lambda);

    static Result extendTree(TreeSnapshot &tree, double lambda,
                             int iterations = -1);

    static void pruneDescendants(TreeSnapshot &tree, int motion_index);
    static void pruneNeighbors(TreeSnapshot &tree, int motion_index,
                               double occupied_radius);
    static void markMotion(TreeSnapshot &tree, int motion_index);
    static bool updateMinSafeArrivalTimestep(
        TreeSnapshot &tree, std::size_t min_safe_arrival_timestep);

    static bool pruneWithConstraint(TreeSnapshot &tree,
                                    const BranchConstraint &constraint);

private:
    static double configDistance(const std::vector<double> &a,
                                 const std::vector<double> &b);
};

} // namespace comotion
