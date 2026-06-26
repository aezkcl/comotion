#include "comotion/planning/USTRRTstar.h"
#include "comotion/planning/PlanningSeed.h"

#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>
#include <ompl/util/RandomNumbers.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace ob = ompl::base;

namespace comotion {

namespace {

using Motion = USTRRTstar::Motion;
using TreeSnapshot = USTRRTstar::TreeSnapshot;
using BranchConstraint = USTRRTstar::BranchConstraint;
using Params = USTRRTstar::Params;

constexpr double kTimeTolScale = 0.5;
constexpr double kGoalSampleProbability = 0.1;

double vectorDistance(const std::vector<double> &a,
                      const std::vector<double> &b) {
    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

double getTime(const ob::State *state) {
    return ob::SpaceTimeStateSpace::getStateTime(state);
}

double getTime(const Motion *motion) {
    return getTime(motion->state);
}

std::vector<double> getConfig(const ob::State *state, int ndof) {
    const auto *compound = state->as<ob::CompoundState>();
    const auto *rv =
        compound->as<ob::RealVectorStateSpace::StateType>(0);
    std::vector<double> config(static_cast<std::size_t>(ndof));
    for (int i = 0; i < ndof; ++i)
        config[static_cast<std::size_t>(i)] = rv->values[i];
    return config;
}

void setConfig(ob::State *state, const std::vector<double> &config) {
    auto *compound = state->as<ob::CompoundState>();
    auto *rv = compound->as<ob::RealVectorStateSpace::StateType>(0);
    for (std::size_t i = 0; i < config.size(); ++i)
        rv->values[i] = config[i];
}

void setTime(ob::State *state, double time_seconds) {
    auto *compound = state->as<ob::CompoundState>();
    compound->as<ob::TimeStateSpace::StateType>(1)->position = time_seconds;
}

void setState(ob::State *state, const std::vector<double> &config,
              double time_seconds) {
    setConfig(state, config);
    setTime(state, time_seconds);
}

double configDistance(const ob::State *a, const ob::State *b, int ndof) {
    return vectorDistance(getConfig(a, ndof), getConfig(b, ndof));
}

double finiteSpaceTimeDistance(const TreeSnapshot &tree, const ob::State *from,
                               const ob::State *to) {
    const double delta_space = tree.space->distanceSpace(from, to);
    const double delta_time = std::abs(getTime(to) - getTime(from));
    const double min_time_for_space =
        (tree.vmax > 0.0) ? (delta_space / tree.vmax)
                          : std::numeric_limits<double>::infinity();
    return std::max(delta_time, min_time_for_space);
}

double constraintTimeEpsilon(const TreeSnapshot &tree) {
    return 0.5 /
           static_cast<double>(std::max<std::size_t>(1, tree.resolution));
}

bool timeMatchesConstraint(double time_seconds, double constraint_time,
                           double time_eps) {
    return std::abs(time_seconds - constraint_time) <= time_eps;
}

bool intervalContainsConstraintTime(double t0, double t1,
                                    double constraint_time,
                                    double time_eps) {
    const double lo = std::min(t0, t1) - time_eps;
    const double hi = std::max(t0, t1) + time_eps;
    return constraint_time >= lo && constraint_time <= hi;
}

std::vector<double> configAtTimeOnEdge(const std::vector<double> &from,
                                       const std::vector<double> &to,
                                       double t0, double t1,
                                       double query_time) {
    if (std::abs(t1 - t0) <= 1e-12)
        return to;

    const double alpha =
        std::clamp((query_time - t0) / (t1 - t0), 0.0, 1.0);
    return interpolateConfig(from, to, alpha);
}

void throwIfDuplicateConstraint(const TreeSnapshot &tree,
                                const BranchConstraint &constraint) {
    for (const auto &existing : tree.constraints) {
        if (existing.constrained_agent_id == constraint.constrained_agent_id &&
            existing.other_agent_id == constraint.other_agent_id &&
            existing.timestep == constraint.timestep) {
            std::ostringstream msg;
            msg << "USTRRTstar: duplicate constraint for constrained_robot="
                << constraint.constrained_agent_id
                << ", other_robot=" << constraint.other_agent_id
                << ", timestep=" << constraint.timestep
                << ", existing_constraints=" << tree.constraints.size();
            throw std::runtime_error(msg.str());
        }
    }
}

bool sameLayer(double lhs, double rhs, double layer_dt) {
    return std::abs(lhs - rhs) <= std::max(1e-9, layer_dt * kTimeTolScale);
}

int timestepForTime(double time_seconds, std::size_t resolution) {
    return static_cast<int>(
        std::llround(time_seconds * static_cast<double>(resolution)));
}

bool configWithinGoalThreshold(const TreeSnapshot &tree,
                               const std::vector<double> &config) {
    return vectorDistance(config, tree.goal_config) <= tree.params.goal_threshold;
}

bool respectsMinSafeArrival(const TreeSnapshot &tree, double time_seconds) {
    return static_cast<std::size_t>(
               std::max(0, timestepForTime(time_seconds, tree.resolution))) >=
           tree.min_safe_arrival_timestep;
}

class ConstraintAwareStateValidityChecker final
    : public ob::StateValidityChecker {
public:
    ConstraintAwareStateValidityChecker(const ob::SpaceInformationPtr &si,
                                        const TreeSnapshot *tree)
        : ob::StateValidityChecker(si), tree_(tree) {}

    bool isValid(const ob::State *state) const override {
        if (tree_->goal_permanently_blocked)
            return false;
        const int ndof = tree_->model->numJoints();
        const auto config = getConfig(state, ndof);
        if (!tree_->collision_checker->isValidSingleFull(*tree_->model, config))
            return false;

        const double time_seconds = getTime(state);
        if (tree_->min_safe_arrival_timestep > 0 &&
            configWithinGoalThreshold(*tree_, config) &&
            !respectsMinSafeArrival(*tree_, time_seconds)) {
            return false;
        }
        const double time_eps = constraintTimeEpsilon(*tree_);
        for (const auto &constraint : tree_->constraints) {
            if (constraint.other_model == nullptr ||
                !timeMatchesConstraint(time_seconds, constraint.time_seconds,
                                       time_eps)) {
                continue;
            }
            if (!tree_->collision_checker->isValidPair(
                    *tree_->model, config, *constraint.other_model,
                    constraint.other_config)) {
                return false;
            }
        }
        return true;
    }

private:
    const TreeSnapshot *tree_;
};

class ConstraintAwareMotionValidator final : public ob::MotionValidator {
public:
    ConstraintAwareMotionValidator(const ob::SpaceInformationPtr &si,
                                   const TreeSnapshot *tree)
        : ob::MotionValidator(si), tree_(tree),
          state_space_(si->getStateSpace().get()) {}

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override {
        if (tree_->goal_permanently_blocked)
            return false;
        if (!si_->isValid(s2))
            return false;

        auto *space = state_space_->as<ob::SpaceTimeStateSpace>();
        const double delta_pos = space->distanceSpace(s1, s2);
        const double t1 = getTime(s1);
        const double t2 = getTime(s2);
        const double delta_t = t2 - t1;
        if (!(delta_t > 0.0 && delta_pos / delta_t <= tree_->vmax + 1e-9))
            return false;

        const int ndof = tree_->model->numJoints();
        const auto from = getConfig(s1, ndof);
        const auto to = getConfig(s2, ndof);
        const int num_checks =
            std::max(1, static_cast<int>(si_->getStateSpace()->validSegmentCount(
                            s1, s2)));
        if (!tree_->collision_checker->isMotionValid(*tree_->model, from, to,
                                                     num_checks)) {
            return false;
        }

        if (tree_->constraints.empty())
            return true;

        const double time_eps = constraintTimeEpsilon(*tree_);
        for (const auto &constraint : tree_->constraints) {
            if (constraint.other_model == nullptr ||
                !intervalContainsConstraintTime(
                    t1, t2, constraint.time_seconds, time_eps)) {
                continue;
            }
            const auto sample_config = configAtTimeOnEdge(
                from, to, t1, t2, constraint.time_seconds);
            if (!tree_->collision_checker->isValidPair(
                    *tree_->model, sample_config, *constraint.other_model,
                    constraint.other_config)) {
                return false;
            }
        }

        return true;
    }

    bool checkMotion(const ob::State *s1, const ob::State *s2,
                     std::pair<ob::State *, double> &lastValid) const override {
        lastValid.first = nullptr;
        lastValid.second = 0.0;
        return checkMotion(s1, s2);
    }

private:
    const TreeSnapshot *tree_;
    ob::StateSpace *state_space_;
};

std::shared_ptr<ob::RealVectorStateSpace> createVectorStateSpace(
    const RobotModel &model) {
    const int ndof = model.numJoints();
    auto vector_space = std::make_shared<ob::RealVectorStateSpace>(ndof);
    ob::RealVectorBounds bounds(ndof);
    for (int i = 0; i < ndof; ++i) {
        bounds.setLow(i, model.jointLower(i));
        bounds.setHigh(i, model.jointUpper(i));
    }
    vector_space->setBounds(bounds);
    return vector_space;
}

std::shared_ptr<TreeSnapshot::NN> makeNearestNeighbors(const TreeSnapshot &tree) {
    using NNGNAT = ompl::NearestNeighborsGNATNoThreadSafety<Motion *>;
    auto nn = std::make_shared<NNGNAT>();
    nn->setDistanceFunction(
        [&tree](const Motion *a, const Motion *b) {
            return finiteSpaceTimeDistance(tree, a->state, b->state);
        });
    return nn;
}

void initializeInfrastructure(TreeSnapshot &tree) {
    auto vector_space = createVectorStateSpace(*tree.model);
    tree.space = std::make_shared<ob::SpaceTimeStateSpace>(vector_space,
                                                           tree.vmax);
    tree.space->setTimeBounds(0.0, tree.max_time_bound);
    tree.space->updateEpsilon();

    tree.si = std::make_shared<ob::SpaceInformation>(tree.space);
    tree.si->setStateValidityChecker(
        std::make_shared<ConstraintAwareStateValidityChecker>(tree.si, &tree));
    tree.si->setMotionValidator(
        std::make_shared<ConstraintAwareMotionValidator>(tree.si, &tree));
    tree.si->setup();

    tree.nn = makeNearestNeighbors(tree);
}

Motion *appendMotion(TreeSnapshot &tree, const ob::State *state,
                     Motion *parent) {
    auto motion = std::make_unique<Motion>();
    motion->index = static_cast<int>(tree.motions.size());
    motion->state = tree.si->allocState();
    tree.si->copyState(motion->state, state);
    motion->parent = parent;
    motion->active = true;
    motion->marked = false;

    Motion *raw = motion.get();
    tree.motions.push_back(std::move(motion));
    if (parent != nullptr)
        parent->children.push_back(raw);
    if (tree.nn)
        tree.nn->add(raw);
    return raw;
}

double computePathCost(const Motion *motion, int ndof, double lambda) {
    double spatial = 0.0;
    const Motion *cur = motion;
    while (cur != nullptr && cur->parent != nullptr) {
        spatial += configDistance(cur->state, cur->parent->state, ndof);
        cur = cur->parent;
    }
    return lambda * spatial + (1.0 - lambda) * getTime(motion);
}

bool goalReached(const TreeSnapshot &tree, const Motion *motion) {
    const int ndof = tree.model->numJoints();
    if (motion == nullptr || motion->marked)
        return false;
    return vectorDistance(getConfig(motion->state, ndof), tree.goal_config) <=
               tree.params.goal_threshold &&
           respectsMinSafeArrival(tree, getTime(motion));
}

std::size_t deactivateSubtree(Motion *root, bool include_root = true) {
    std::queue<Motion *> q;
    if (include_root) {
        q.push(root);
    } else {
        for (Motion *child : root->children)
            q.push(child);
    }
    std::size_t deactivated = 0;
    while (!q.empty()) {
        Motion *motion = q.front();
        q.pop();
        if (!motion->active)
            continue;
        motion->active = false;
        ++deactivated;
        for (Motion *child : motion->children)
            q.push(child);
    }
    return deactivated;
}

bool motionViolatesConstraint(const TreeSnapshot &tree, const Motion *motion,
                              const BranchConstraint &constraint) {
    if (constraint.other_model == nullptr || !motion->active)
        return false;

    const int ndof = tree.model->numJoints();
    const auto config = getConfig(motion->state, ndof);
    const double motion_time = getTime(motion);
    const double time_eps = constraintTimeEpsilon(tree);
    if (timeMatchesConstraint(motion_time, constraint.time_seconds, time_eps) &&
        !tree.collision_checker->isValidPair(*tree.model, config,
                                             *constraint.other_model,
                                             constraint.other_config)) {
        return true;
    }

    if (motion->parent == nullptr || !motion->parent->active)
        return false;

    const auto parent_config = getConfig(motion->parent->state, ndof);
    const auto child_config = config;
    const double t1 = getTime(motion->parent);
    const double t2 = getTime(motion);
    if (!intervalContainsConstraintTime(
            t1, t2, constraint.time_seconds, time_eps)) {
        return false;
    }
    const auto sample_config = configAtTimeOnEdge(
        parent_config, child_config, t1, t2, constraint.time_seconds);
    return !tree.collision_checker->isValidPair(
        *tree.model, sample_config, *constraint.other_model,
        constraint.other_config);
}

Motion *findNearestTimeCausal(TreeSnapshot &tree, const ob::State *sample) {
    if (tree.nn == nullptr || tree.nn->size() == 0)
        return nullptr;

    Motion query;
    query.state = const_cast<ob::State *>(sample);

    std::vector<Motion *> candidates;
    const std::size_t request = std::min<std::size_t>(
        std::max<std::size_t>(32u, static_cast<std::size_t>(tree.params.rewire_k * 2)),
        tree.nn->size());
    tree.nn->nearestK(&query, request, candidates);

    const double sample_time = getTime(sample);
    Motion *best = nullptr;
    double best_dist = std::numeric_limits<double>::infinity();
    for (Motion *candidate : candidates) {
        if (!candidate->active || candidate->marked ||
            !(getTime(candidate) + 1e-9 < sample_time))
            continue;
        const double dist =
            finiteSpaceTimeDistance(tree, candidate->state, sample);
        if (dist < best_dist) {
            best_dist = dist;
            best = candidate;
        }
    }
    if (best != nullptr)
        return best;

    candidates.clear();
    tree.nn->list(candidates);
    for (Motion *candidate : candidates) {
        if (!candidate->active || candidate->marked ||
            !(getTime(candidate) + 1e-9 < sample_time))
            continue;
        const double dist =
            finiteSpaceTimeDistance(tree, candidate->state, sample);
        if (dist < best_dist) {
            best_dist = dist;
            best = candidate;
        }
    }
    return best;
}

Motion *chooseBestParent(TreeSnapshot &tree, const ob::State *state,
                         Motion *fallback, double lambda) {
    Motion *best = nullptr;
    double best_cost = std::numeric_limits<double>::infinity();

    const double new_time = getTime(state);
    const double max_step = tree.vmax * tree.params.layer_dt_seconds + 1e-9;
    const int ndof = tree.model->numJoints();

    for (const auto &motion_ptr : tree.motions) {
        Motion *candidate = motion_ptr.get();
        if (!candidate->active || candidate->marked)
            continue;
        const double candidate_time = getTime(candidate);
        if (!(candidate_time + 1e-9 < new_time))
            continue;
        if (!sameLayer(new_time - candidate_time, tree.params.layer_dt_seconds,
                       tree.params.layer_dt_seconds))
            continue;
        if (configDistance(candidate->state, state, ndof) > max_step)
            continue;
        if (!tree.si->checkMotion(candidate->state, state))
            continue;

        const double candidate_cost =
            computePathCost(candidate, ndof, lambda) +
            lambda * configDistance(candidate->state, state, ndof) +
            (1.0 - lambda) * (new_time - candidate_time);
        if (candidate_cost < best_cost) {
            best_cost = candidate_cost;
            best = candidate;
        }
    }

    if (best == nullptr && fallback != nullptr && fallback->active &&
        !fallback->marked &&
        tree.si->checkMotion(fallback->state, state)) {
        best = fallback;
    }

    return best;
}

int rewireFrom(TreeSnapshot &tree, Motion *added, double lambda) {
    if (tree.params.rewire_mode == USTRRTstar::RewireMode::Off)
        return 0;

    const double max_step = tree.vmax * tree.params.layer_dt_seconds + 1e-9;
    const double added_time = getTime(added);
    const int ndof = tree.model->numJoints();
    const double added_cost = computePathCost(added, ndof, lambda);
    int rewired = 0;

    for (const auto &motion_ptr : tree.motions) {
        Motion *candidate = motion_ptr.get();
        if (!candidate->active || candidate->marked || candidate == added ||
            candidate->parent == nullptr)
            continue;
        const double candidate_time = getTime(candidate);
        if (!sameLayer(candidate_time - added_time, tree.params.layer_dt_seconds,
                       tree.params.layer_dt_seconds))
            continue;

        const double edge_dist = configDistance(candidate->state, added->state, ndof);
        if (edge_dist > max_step)
            continue;
        if (tree.params.rewire_mode == USTRRTstar::RewireMode::Radius &&
            edge_dist > tree.params.rewire_radius) {
            continue;
        }

        if (!tree.si->checkMotion(added->state, candidate->state))
            continue;

        const double new_cost =
            added_cost + lambda * edge_dist +
            (1.0 - lambda) * (candidate_time - added_time);
        if (new_cost + 1e-9 >= computePathCost(candidate, ndof, lambda))
            continue;

        Motion *old_parent = candidate->parent;
        auto &siblings = old_parent->children;
        siblings.erase(std::remove(siblings.begin(), siblings.end(), candidate),
                       siblings.end());
        candidate->parent = added;
        added->children.push_back(candidate);
        ++rewired;
    }
    return rewired;
}

USTRRTstar::Result runIterations(TreeSnapshot &tree, double lambda,
                                    int iterations) {
    USTRRTstar::Result result;
    if (tree.goal_permanently_blocked || tree.start_motion == nullptr ||
        !tree.start_motion->active || tree.start_motion->marked ||
        tree.usableMotionCount() == 0 || iterations <= 0) {
        return USTRRTstar::extractBestPath(tree, lambda);
    }

    auto sampler = tree.si->allocStateSampler();
    ob::State *sample = tree.si->allocState();
    ob::State *temp = tree.si->allocState();

    ompl::RNG &rng = tree.iteration_rng;
    const int ndof = tree.model->numJoints();
    const double max_step = tree.vmax * tree.params.layer_dt_seconds;

    for (int iter = 0;
         iter < iterations &&
         static_cast<int>(tree.motions.size()) < tree.params.max_samples;
         ++iter) {
        if (rng.uniform01() < kGoalSampleProbability) {
            setState(sample, tree.goal_config, tree.current_time_upper_bound);
        } else {
            tree.space->getTimeComponent()->setBounds(0.0,
                                                      tree.current_time_upper_bound);
            sampler->sampleUniform(sample);
        }
        if (!tree.si->isValid(sample)) {
            continue;
        }

        Motion *nearest = findNearestTimeCausal(tree, sample);
        if (nearest == nullptr) {
            continue;
        }

        const auto nearest_config = getConfig(nearest->state, ndof);
        const auto sample_config = getConfig(sample, ndof);
        const double total_dist = vectorDistance(nearest_config, sample_config);
        if (total_dist < 1e-9) {
            continue;
        }

        const double steer_dist = std::min(total_dist, tree.params.range);
        const int n_steps =
            std::max(1, static_cast<int>(std::ceil(steer_dist / max_step)));

        Motion *last_added = nearest;
        bool trapped = false;
        for (int step = 0; step < n_steps; ++step) {
            const double cumulative =
                std::min(steer_dist, max_step * static_cast<double>(step + 1));
            const double alpha = cumulative / total_dist;
            const auto new_config =
                interpolateConfig(nearest_config, sample_config, alpha);
            const double new_time =
                getTime(nearest) +
                static_cast<double>(step + 1) * tree.params.layer_dt_seconds;
            if (new_time > tree.max_time_bound + 1e-9) {
                trapped = true;
                break;
            }

            setState(temp, new_config, new_time);
            if (!tree.si->satisfiesBounds(temp)) {
                trapped = true;
                break;
            }

            Motion *best_parent =
                chooseBestParent(tree, temp, last_added, lambda);
            if (best_parent == nullptr) {
                trapped = true;
                break;
            }

            Motion *added = appendMotion(tree, temp, best_parent);
            rewireFrom(tree, added, lambda);

            tree.current_time_upper_bound = std::min(
                tree.max_time_bound,
                std::max(tree.current_time_upper_bound,
                         getTime(added) + tree.params.layer_dt_seconds));

            last_added = added;
            if (goalReached(tree, added)) {
                const auto exact = USTRRTstar::extractBestPath(tree, lambda);
                tree.si->freeState(sample);
                tree.si->freeState(temp);
                return exact;
            }
        }

        if (trapped && rng.uniform01() < 0.01) {
            tree.rebuildNearestNeighbors();
        }
    }

    tree.si->freeState(sample);
    tree.si->freeState(temp);
    return USTRRTstar::extractBestPath(tree, lambda);
}

} // namespace

TreeSnapshot::TreeSnapshot() = default;

TreeSnapshot::~TreeSnapshot() {
    if (si != nullptr) {
        for (auto &motion : motions) {
            if (motion && motion->state != nullptr) {
                si->freeState(motion->state);
                motion->state = nullptr;
            }
        }
    }
}

std::shared_ptr<TreeSnapshot> TreeSnapshot::clone() const {
    auto copy = std::make_shared<TreeSnapshot>();
    copy->model = model;
    copy->collision_checker = collision_checker;
    copy->agent_index = agent_index;
    copy->resolution = resolution;
    copy->vmax = vmax;
    copy->params = params;
    copy->start_config = start_config;
    copy->goal_config = goal_config;
    copy->current_time_upper_bound = current_time_upper_bound;
    copy->max_time_bound = max_time_bound;
    copy->min_safe_arrival_timestep = min_safe_arrival_timestep;
    copy->goal_permanently_blocked = goal_permanently_blocked;
    copy->constraints = constraints;
    copy->iteration_rng = iteration_rng;

    initializeInfrastructure(*copy);

    std::vector<Motion *> remap(motions.size(), nullptr);
    for (const auto &motion : motions) {
        auto new_motion = std::make_unique<Motion>();
        new_motion->index = motion->index;
        new_motion->state = copy->si->allocState();
        copy->si->copyState(new_motion->state, motion->state);
        new_motion->active = motion->active;
        new_motion->marked = motion->marked;
        remap[static_cast<std::size_t>(motion->index)] = new_motion.get();
        copy->motions.push_back(std::move(new_motion));
    }

    for (const auto &motion : motions) {
        Motion *new_motion = remap[static_cast<std::size_t>(motion->index)];
        if (motion->parent != nullptr)
            new_motion->parent =
                remap[static_cast<std::size_t>(motion->parent->index)];
        for (Motion *child : motion->children)
            new_motion->children.push_back(
                remap[static_cast<std::size_t>(child->index)]);
    }

    if (start_motion != nullptr)
        copy->start_motion = remap[static_cast<std::size_t>(start_motion->index)];

    copy->rebuildNearestNeighbors();
    return copy;
}

void TreeSnapshot::rebuildNearestNeighbors() {
    nn = makeNearestNeighbors(*this);
    for (const auto &motion : motions) {
        if (motion->active && !motion->marked)
            nn->add(motion.get());
    }
}

std::size_t TreeSnapshot::activeMotionCount() const {
    std::size_t count = 0;
    for (const auto &motion : motions) {
        if (motion->active)
            ++count;
    }
    return count;
}

std::size_t TreeSnapshot::usableMotionCount() const {
    std::size_t count = 0;
    for (const auto &motion : motions) {
        if (motion->active && !motion->marked)
            ++count;
    }
    return count;
}

bool TreeSnapshot::isSearchFeasible() const {
    return !goal_permanently_blocked && start_motion != nullptr &&
           start_motion->active && !start_motion->marked &&
           usableMotionCount() > 0;
}

double USTRRTstar::configDistance(const std::vector<double> &a,
                                     const std::vector<double> &b) {
    double sum = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

std::pair<std::shared_ptr<TreeSnapshot>, USTRRTstar::Result>
USTRRTstar::buildTree(int agent_index, const std::vector<double> &start,
                         const std::vector<double> &goal,
                         const RobotModel &model,
                         const CollisionChecker &collision_checker,
                         const Params &params, std::size_t resolution,
                         double vmax, double lambda,
                         std::uint32_t planning_seed) {
    auto tree = std::make_shared<TreeSnapshot>();
    tree->model = &model;
    tree->collision_checker = &collision_checker;
    tree->agent_index = agent_index;
    tree->resolution = resolution;
    tree->vmax = vmax;
    tree->params = params;
    tree->start_config = start;
    tree->goal_config = goal;
    tree->iteration_rng.setLocalSeed(
        comotion::omplLocalSeedFromUserPlanningSeed(planning_seed, agent_index));

    const double direct_goal_time =
        vmax > 0.0 ? configDistance(start, goal) / vmax : 0.0;
    tree->current_time_upper_bound = params.layer_dt_seconds;
    tree->max_time_bound =
        std::max({params.layer_dt_seconds *
                      static_cast<double>(std::max(params.max_iterations + 1,
                                                   params.max_samples)),
                  direct_goal_time * 2.0 + params.layer_dt_seconds,
                  params.layer_dt_seconds});

    initializeInfrastructure(*tree);

    ob::State *start_state = tree->si->allocState();
    setState(start_state, start, 0.0);
    if (tree->si->isValid(start_state)) {
        tree->start_motion = appendMotion(*tree, start_state, nullptr);
    }
    tree->si->freeState(start_state);

    if (tree->start_motion == nullptr) {
        return {std::move(tree), {}};
    }

    auto result = runIterations(*tree, lambda, params.max_iterations);
    return {std::move(tree), std::move(result)};
}

USTRRTstar::Result USTRRTstar::extractBestPath(const TreeSnapshot &tree,
                                                     double lambda) {
    Result best;
    if (tree.goal_permanently_blocked || tree.start_motion == nullptr ||
        !tree.start_motion->active || tree.start_motion->marked) {
        return best;
    }

    const int ndof = tree.model->numJoints();
    Motion *best_motion = nullptr;
    for (const auto &motion_ptr : tree.motions) {
        Motion *motion = motion_ptr.get();
        if (!motion->active || motion->marked || !goalReached(tree, motion))
            continue;

        bool valid_chain = true;
        Motion *cur = motion;
        while (cur != nullptr) {
            if (!cur->active || cur->marked) {
                valid_chain = false;
                break;
            }
            cur = cur->parent;
        }
        if (!valid_chain)
            continue;

        const double cost = computePathCost(motion, ndof, lambda);
        if (cost < best.st_cost) {
            best.st_cost = cost;
            best_motion = motion;
        }
    }

    if (best_motion == nullptr) {
        return best;
    }

    std::vector<Motion *> chain;
    for (Motion *cur = best_motion; cur != nullptr; cur = cur->parent)
        chain.push_back(cur);
    std::reverse(chain.begin(), chain.end());

    best.raw_path.reserve(chain.size());
    best.raw_times_seconds.reserve(chain.size());
    best.motion_indices.reserve(chain.size());
    for (Motion *motion : chain) {
        best.raw_path.push_back(getConfig(motion->state, ndof));
        best.raw_times_seconds.push_back(getTime(motion));
        best.motion_indices.push_back(motion->index);
    }
    best.arrival_time_seconds = best.raw_times_seconds.back();
    best.raw_path.set_waypoint_timesteps_from_tau(best.raw_times_seconds,
                                                  tree.resolution, 1.0);

    best.dense_path = best.raw_path;
    best.dense_path.interpolate_to_timesteps(tree.resolution, tree.vmax);
    best.dense_timestep_to_motion_index.resize(best.dense_path.size());

    std::size_t seg = 0;
    for (std::size_t ts = 0; ts < best.dense_timestep_to_motion_index.size();
         ++ts) {
        while (seg + 1 < best.raw_path.size() &&
               best.raw_path.timestep_at(seg + 1, tree.resolution) <= ts) {
            ++seg;
        }
        int motion_index = best.motion_indices[seg];
        if (seg + 1 < best.motion_indices.size() &&
            ts > best.raw_path.timestep_at(seg, tree.resolution)) {
            motion_index = best.motion_indices[seg + 1];
        }
        best.dense_timestep_to_motion_index[ts] = motion_index;
    }
    best.exact = true;
    return best;
}

USTRRTstar::Result USTRRTstar::extendTree(TreeSnapshot &tree,
                                                double lambda,
                                                int iterations) {
    const int iters =
        iterations > 0 ? iterations : tree.params.max_iterations;
    tree.rebuildNearestNeighbors();
    return runIterations(tree, lambda, iters);
}

void USTRRTstar::pruneDescendants(TreeSnapshot &tree, int motion_index) {
    if (motion_index < 0 ||
        static_cast<std::size_t>(motion_index) >= tree.motions.size()) {
        return;
    }
    Motion *motion = tree.motions[static_cast<std::size_t>(motion_index)].get();
    if (motion == nullptr)
        return;
    deactivateSubtree(motion, false);
}

void USTRRTstar::pruneNeighbors(TreeSnapshot &tree, int motion_index,
                                   double occupied_radius) {
    if (motion_index < 0 ||
        static_cast<std::size_t>(motion_index) >= tree.motions.size() ||
        occupied_radius < 0.0) {
        return;
    }

    Motion *constraint_motion =
        tree.motions[static_cast<std::size_t>(motion_index)].get();
    if (constraint_motion == nullptr || !constraint_motion->active)
        return;

    const double constraint_time = getTime(constraint_motion);
    const int ndof = tree.model->numJoints();
    const auto constraint_config = getConfig(constraint_motion->state, ndof);

    for (const auto &motion_ptr : tree.motions) {
        Motion *candidate = motion_ptr.get();
        if (candidate == nullptr || candidate == constraint_motion ||
            !candidate->active) {
            continue;
        }
        if (!sameLayer(getTime(candidate), constraint_time,
                       tree.params.layer_dt_seconds)) {
            continue;
        }
        const auto candidate_config = getConfig(candidate->state, ndof);
        if (vectorDistance(candidate_config, constraint_config) <=
            occupied_radius) {
            deactivateSubtree(candidate);
        }
    }
}

void USTRRTstar::markMotion(TreeSnapshot &tree, int motion_index) {
    if (motion_index < 0 ||
        static_cast<std::size_t>(motion_index) >= tree.motions.size()) {
        return;
    }
    Motion *motion = tree.motions[static_cast<std::size_t>(motion_index)].get();
    if (motion != nullptr)
        motion->marked = true;
}

bool USTRRTstar::updateMinSafeArrivalTimestep(
    TreeSnapshot &tree, std::size_t min_safe_arrival_timestep) {
    tree.min_safe_arrival_timestep =
        std::max(tree.min_safe_arrival_timestep, min_safe_arrival_timestep);

    const double min_safe_arrival_time =
        static_cast<double>(tree.min_safe_arrival_timestep) /
        static_cast<double>(std::max<std::size_t>(1, tree.resolution));
    if (min_safe_arrival_time > tree.max_time_bound + 1e-9) {
        tree.goal_permanently_blocked = true;
        return false;
    }

    tree.current_time_upper_bound = std::min(
        tree.max_time_bound,
        std::max(tree.current_time_upper_bound,
                 min_safe_arrival_time + tree.params.layer_dt_seconds));
    return true;
}

bool USTRRTstar::pruneWithConstraint(TreeSnapshot &tree,
                                        const BranchConstraint &constraint) {
    throwIfDuplicateConstraint(tree, constraint);
    tree.constraints.push_back(constraint);

    std::vector<Motion *> violating;
    for (const auto &motion_ptr : tree.motions) {
        Motion *motion = motion_ptr.get();
        if (!motion->active)
            continue;
        if (motionViolatesConstraint(tree, motion, constraint))
            violating.push_back(motion);
    }

    for (Motion *motion : violating)
        deactivateSubtree(motion);

    tree.rebuildNearestNeighbors();
    const bool feasible = tree.start_motion != nullptr && tree.start_motion->active &&
                          tree.activeMotionCount() > 0;
    return feasible;
}

} // namespace comotion
