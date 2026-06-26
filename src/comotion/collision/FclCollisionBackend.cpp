#include "comotion/collision/detail/CollisionBackend.h"
#include "comotion/collision/detail/ValidationUtils.h"

#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/math/bv/OBBRSS.h>
#include <fcl/math/triangle.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace comotion {
namespace detail {

namespace {

using S = double;
using Vector3 = fcl::Vector3<S>;
using Transform3 = fcl::Transform3<S>;
using CollisionGeometryPtr = std::shared_ptr<fcl::CollisionGeometry<S>>;
using BVHModel = fcl::BVHModel<fcl::OBBRSS<S>>;

static bool loadObjMeshes(const std::string &path,
                          std::vector<Vector3> &out_vertices,
                          std::vector<fcl::Triangle> &out_tris) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("FCL backend: cannot open mesh: " + path);

    std::vector<Vector3> raw_v;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream ls(line);
        std::string tag;
        ls >> tag;
        if (tag == "v") {
            double x, y, z;
            if (!(ls >> x >> y >> z))
                continue;
            raw_v.emplace_back(x, y, z);
        }
    }

    in.clear();
    in.seekg(0);
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream ls(line);
        std::string tag;
        ls >> tag;
        if (tag != "f")
            continue;

        std::vector<int> corner;
        std::string tok;
        while (ls >> tok) {
            std::string first = tok;
            auto slash = first.find('/');
            if (slash != std::string::npos)
                first = first.substr(0, slash);
            int idx = std::stoi(first);
            if (idx < 0)
                idx = static_cast<int>(raw_v.size()) + idx + 1;
            corner.push_back(idx - 1);
        }
        if (corner.size() < 3)
            continue;
        for (size_t k = 1; k + 1 < corner.size(); ++k) {
            int i0 = corner[0];
            int i1 = corner[k];
            int i2 = corner[k + 1];
            if (i0 < 0 || i1 < 0 || i2 < 0 ||
                i0 >= static_cast<int>(raw_v.size()) ||
                i1 >= static_cast<int>(raw_v.size()) ||
                i2 >= static_cast<int>(raw_v.size()))
                continue;
            out_tris.emplace_back(static_cast<std::size_t>(i0),
                                  static_cast<std::size_t>(i1),
                                  static_cast<std::size_t>(i2));
        }
    }
    out_vertices = std::move(raw_v);
    return !out_tris.empty();
}

static CollisionGeometryPtr makeMeshGeometry(const CollisionMesh &cm) {
    std::vector<Vector3> verts;
    std::vector<fcl::Triangle> tris;
    if (!loadObjMeshes(cm.resolved_path, verts, tris))
        throw std::runtime_error("FCL backend: no triangles in mesh: " +
                                 cm.resolved_path);

    Eigen::Matrix3d R = cm.origin.linear();
    Eigen::Vector3d t = cm.origin.translation();
    for (auto &v : verts) {
        Eigen::Vector3d p(v[0], v[1], v[2]);
        p = p.cwiseProduct(Eigen::Vector3d(cm.scale.x(), cm.scale.y(),
                                           cm.scale.z()));
        Eigen::Vector3d q = R * p + t;
        v = Vector3(q.x(), q.y(), q.z());
    }

    auto model = std::make_shared<BVHModel>();
    model->beginModel(static_cast<int>(tris.size()),
                      static_cast<int>(verts.size()));
    for (const auto &v : verts)
        model->addVertex(v);
    for (const auto &tri : tris)
        model->addTriangle(verts[tri[0]], verts[tri[1]], verts[tri[2]]);
    if (model->endModel() != fcl::BVH_OK)
        throw std::runtime_error("FCL backend: BVH build failed for " +
                                 cm.resolved_path);
    return model;
}

struct GeomPrim {
    CollisionGeometryPtr geom;
    Transform3 tf_link;
    int link_index = -1;
};

struct RobotPrimCache {
    std::vector<GeomPrim> prims;
};

static std::vector<GeomPrim> buildPrimsForRobot(const RobotModel &robot) {
    std::vector<GeomPrim> prims;
    for (const auto &link : robot.links()) {
        for (const auto &sph : link.collision_spheres) {
            GeomPrim g;
            g.geom = std::make_shared<fcl::Sphere<S>>(sph.radius);
            g.tf_link = Transform3(fcl::Translation3<S>(
                Vector3(sph.center.x(), sph.center.y(), sph.center.z())));
            g.link_index = link.index;
            prims.push_back(std::move(g));
        }
        for (const auto &mesh : link.collision_meshes) {
            GeomPrim g;
            g.geom = makeMeshGeometry(mesh);
            g.tf_link = Transform3::Identity();
            g.link_index = link.index;
            prims.push_back(std::move(g));
        }
    }
    return prims;
}

static Transform3 eigenAffineToFcl(const Eigen::Affine3d &T) {
    Transform3 out = Transform3::Identity();
    out.linear() = T.rotation();
    out.translation() =
        Vector3(T.translation().x(), T.translation().y(), T.translation().z());
    return out;
}

static bool fclPairCollide(const CollisionGeometryPtr &ga, const Transform3 &Ta,
                           const CollisionGeometryPtr &gb, const Transform3 &Tb) {
    fcl::CollisionObject<S> oa(ga, Ta);
    fcl::CollisionObject<S> ob(gb, Tb);
    fcl::CollisionRequest<S> req(1, false);
    fcl::CollisionResult<S> res;
    return fcl::collide(&oa, &ob, req, res) > 0;
}

static Transform3 cylinderWorldPose(const ObstacleCylinder &cyl) {
    Eigen::Vector3d ez(0, 0, 1);
    Eigen::Matrix3d Rmat = Eigen::Matrix3d::Identity();
    Eigen::Vector3d ax = cyl.axis.normalized();
    if (ax.dot(ez) < -1.0 + 1e-9) {
        Rmat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    } else if (ax.dot(ez) < 1.0 - 1e-9) {
        Eigen::Quaterniond q =
            Eigen::Quaterniond::FromTwoVectors(ez, ax);
        Rmat = q.toRotationMatrix();
    }
    Transform3 T = Transform3::Identity();
    T.linear() = Rmat;
    T.translation() = Vector3(cyl.center.x(), cyl.center.y(), cyl.center.z());
    return T;
}

class FclBackendImpl final : public CollisionBackend {
public:
    std::unique_ptr<CollisionBackend> clone() const override {
        auto n = std::make_unique<FclBackendImpl>();
        n->env_spheres_ = env_spheres_;
        n->env_cylinders_ = env_cylinders_;
        n->rebuildEnvironmentObjects();
        return n;
    }

    void onEnvironmentChanged(
        const std::vector<ObstacleSphere> &spheres,
        const std::vector<ObstacleCylinder> &cylinders) override {
        env_spheres_ = spheres;
        env_cylinders_ = cylinders;
        rebuildEnvironmentObjects();
    }

    void rebuildEnvironmentObjects() {
        env_geoms_.clear();
        env_transforms_.clear();
        for (const auto &o : env_spheres_) {
            auto g = std::make_shared<fcl::Sphere<S>>(o.radius);
            Transform3 T = Transform3::Identity();
            T.translation() =
                Vector3(o.center.x(), o.center.y(), o.center.z());
            env_geoms_.push_back(std::move(g));
            env_transforms_.push_back(T);
        }
        for (const auto &c : env_cylinders_) {
            double lz = 2.0 * c.half_height;
            auto g = std::make_shared<fcl::Cylinder<S>>(c.radius, lz);
            env_geoms_.push_back(std::move(g));
            env_transforms_.push_back(cylinderWorldPose(c));
        }
    }

    const RobotPrimCache &robotCache(const RobotModel &robot) const {
        const RobotModel *key = &robot;
        auto it = robot_cache_.find(key);
        if (it != robot_cache_.end())
            return it->second;
        RobotPrimCache entry;
        entry.prims = buildPrimsForRobot(robot);
        auto ins = robot_cache_.emplace(key, std::move(entry));
        return ins.first->second;
    }

    bool isValidSingle(const RobotModel &robot,
                       const std::vector<double> &config,
                       const std::vector<ObstacleSphere> &,
                       const std::vector<ObstacleCylinder> &) const override {
        const auto &cache = robotCache(robot);
        if (cache.prims.empty())
            return true;

        auto link_tf = robot.getLinkTransforms(config);
        for (const auto &prim : cache.prims) {
            Transform3 world =
                eigenAffineToFcl(link_tf[static_cast<size_t>(prim.link_index)]) *
                prim.tf_link;
            for (size_t e = 0; e < env_geoms_.size(); ++e) {
                if (fclPairCollide(prim.geom, world, env_geoms_[e],
                                   env_transforms_[e]))
                    return false;
            }
        }
        return true;
    }

    bool isSelfCollisionFree(
        const RobotModel &robot,
        const std::vector<double> &config) const override {
        const auto &cache = robotCache(robot);
        const auto &prims = cache.prims;
        if (prims.size() < 2)
            return true;

        auto link_tf = robot.getLinkTransforms(config);
        for (size_t i = 0; i < prims.size(); ++i) {
            Transform3 Twi =
                eigenAffineToFcl(
                    link_tf[static_cast<size_t>(prims[i].link_index)]) *
                prims[i].tf_link;
            for (size_t j = i + 1; j < prims.size(); ++j) {
                if (prims[i].link_index == prims[j].link_index)
                    continue;
                const auto &li = robot.links()[static_cast<size_t>(prims[i].link_index)];
                const auto &lj = robot.links()[static_cast<size_t>(prims[j].link_index)];
                if (robot.isSelfCollisionDisabled(li.name, lj.name))
                    continue;
                Transform3 Twj =
                    eigenAffineToFcl(
                        link_tf[static_cast<size_t>(prims[j].link_index)]) *
                    prims[j].tf_link;
                if (fclPairCollide(prims[i].geom, Twi, prims[j].geom, Twj))
                    return false;
            }
        }
        return true;
    }

    bool isValidPair(const RobotModel &robot_a,
                     const std::vector<double> &config_a,
                     const RobotModel &robot_b,
                     const std::vector<double> &config_b) const override {
        const auto &ca = robotCache(robot_a);
        const auto &cb = robotCache(robot_b);
        if (ca.prims.empty() || cb.prims.empty())
            return true;

        auto tfa = robot_a.getLinkTransforms(config_a);
        auto tfb = robot_b.getLinkTransforms(config_b);
        for (const auto &pa : ca.prims) {
            Transform3 Twa =
                eigenAffineToFcl(tfa[static_cast<size_t>(pa.link_index)]) *
                pa.tf_link;
            for (const auto &pb : cb.prims) {
                Transform3 Twb =
                    eigenAffineToFcl(tfb[static_cast<size_t>(pb.link_index)]) *
                    pb.tf_link;
                if (fclPairCollide(pa.geom, Twa, pb.geom, Twb))
                    return false;
            }
        }
        return true;
    }

    bool isMotionValid(const RobotModel &robot,
                       const std::vector<double> &from,
                       const std::vector<double> &to, int num_checks,
                       const std::vector<ObstacleSphere> &obstacles,
                       const std::vector<ObstacleCylinder> &cylinders) const override {
        const int steps = std::max(1, num_checks);
        int dim = static_cast<int>(from.size());
        std::vector<double> interp(dim);
        for (int step = 0; step <= steps; ++step) {
            double t = static_cast<double>(step) / steps;
            for (int d = 0; d < dim; ++d)
                interp[d] = from[d] + t * (to[d] - from[d]);
            if (!isValidSingle(robot, interp, obstacles, cylinders) ||
                !isSelfCollisionFree(robot, interp))
                return false;
        }
        return true;
    }

    bool isRobotPathValid(const RobotModel &robot, const Path &path,
                          const std::vector<ObstacleSphere> &obstacles,
                          const std::vector<ObstacleCylinder> &cylinders) const override {
        for (const auto &config : path) {
            if (!isValidSingle(robot, config, obstacles, cylinders) ||
                !isSelfCollisionFree(robot, config)) {
                return false;
            }
        }
        return true;
    }

    bool isPairPathValid(const RobotModel &robot_a, const Path &path_a,
                         const RobotModel &robot_b, const Path &path_b,
                         std::size_t t_begin, std::size_t t_end) const override {
        return !findFirstPairPathConflict(robot_a, path_a, robot_b, path_b,
                                          t_begin, t_end).has_value();
    }

    std::optional<PairPathConflict> findFirstPairPathConflict(
        const RobotModel &robot_a, const Path &path_a,
        const RobotModel &robot_b, const Path &path_b,
        std::size_t t_begin, std::size_t t_end) const override {
        if (path_a.empty() || path_b.empty())
            return std::nullopt;

        const std::size_t max_t = std::max(path_a.size(), path_b.size());
        const std::size_t end = std::min(max_t, t_end);
        for (std::size_t t = t_begin; t < end; ++t) {
            const auto &config_a = configAt(path_a, t);
            const auto &config_b = configAt(path_b, t);
            if (!isValidPair(robot_a, config_a, robot_b, config_b)) {
                return PairPathConflict{t, 0.0, ConflictKind::Vertex,
                                        config_a, config_b};
            }
        }
        return std::nullopt;
    }

    GoalHoldConstraint computeGoalHoldConstraint(
        const RobotModel &goal_robot,
        const std::vector<double> &goal_config,
        const RobotModel &prior_robot,
        const Path &prior_path) const override {
        if (prior_path.empty())
            return {};

        for (std::size_t t = prior_path.size(); t-- > 0;) {
            if (isValidPair(goal_robot, goal_config, prior_robot, prior_path[t]))
                continue;

            if (t + 1 == prior_path.size())
                return GoalHoldConstraint{0, true};
            return GoalHoldConstraint{t + 1, false};
        }

        return {};
    }

    bool isCompositeMotionValid(
        const std::vector<const RobotModel *> &robots,
        const std::vector<std::vector<double>> &from,
        const std::vector<std::vector<double>> &to,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const override {
        if (robots.size() != from.size() || robots.size() != to.size())
            return false;

        const int num_checks =
            options.discrete_num_checks_hint > 0 ? options.discrete_num_checks_hint : 10;
        if (options.check_environment) {
            for (std::size_t i = 0; i < robots.size(); ++i) {
                if (!isMotionValid(*robots[i], from[i], to[i], num_checks, obstacles,
                                   cylinders)) {
                    return false;
                }
            }
        }

        std::vector<std::vector<double>> interp(from.size());
        for (int step = 0; step <= num_checks; ++step) {
            const double alpha =
                static_cast<double>(step) / static_cast<double>(num_checks);
            for (std::size_t i = 0; i < from.size(); ++i)
                interpolateConfigInto(from[i], to[i], alpha, interp[i]);

            for (std::size_t i = 0; i < robots.size(); ++i) {
                for (std::size_t j = i + 1; j < robots.size(); ++j) {
                    if (!isValidPair(*robots[i], interp[i], *robots[j], interp[j])) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    std::optional<CompositeConflict> findFirstCompositeMotionConflict(
        const std::vector<const RobotModel *> &robots,
        const std::vector<std::vector<double>> &from,
        const std::vector<std::vector<double>> &to,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const override {
        if (robots.size() != from.size() || robots.size() != to.size())
            return std::nullopt;

        const int num_checks =
            options.discrete_num_checks_hint > 0 ? options.discrete_num_checks_hint : 10;
        std::vector<std::vector<double>> interp(from.size());
        for (int step = 0; step <= num_checks; ++step) {
            const double alpha =
                static_cast<double>(step) / static_cast<double>(num_checks);
            for (std::size_t i = 0; i < from.size(); ++i)
                interpolateConfigInto(from[i], to[i], alpha, interp[i]);

            if (options.check_environment) {
                for (std::size_t i = 0; i < robots.size(); ++i) {
                    if (!isValidSingle(*robots[i], interp[i], obstacles, cylinders)) {
                        return CompositeConflict{ConflictScope::Environment,
                                                 static_cast<int>(i), -1,
                                                 static_cast<std::size_t>(step),
                                                 0.0, ConflictKind::Vertex,
                                                 interp[i], {}};
                    }
                    if (!isSelfCollisionFree(*robots[i], interp[i])) {
                        return CompositeConflict{ConflictScope::Self,
                                                 static_cast<int>(i), -1,
                                                 static_cast<std::size_t>(step),
                                                 0.0, ConflictKind::Vertex,
                                                 interp[i], {}};
                    }
                }
            }

            for (std::size_t i = 0; i < robots.size(); ++i) {
                for (std::size_t j = i + 1; j < robots.size(); ++j) {
                    if (!isValidPair(*robots[i], interp[i], *robots[j], interp[j])) {
                        return CompositeConflict{ConflictScope::InterRobot,
                                                 static_cast<int>(i),
                                                 static_cast<int>(j),
                                                 static_cast<std::size_t>(step),
                                                 0.0, ConflictKind::Vertex,
                                                 interp[i], interp[j]};
                    }
                }
            }
        }
        return std::nullopt;
    }

    bool validateCompositePaths(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders) const override {
        if (paths.size() != robots.size())
            return false;

        const auto effective_starts = effectivePathStarts(paths.size(), options);
        const auto effective_pair_starts =
            effectivePairStarts(paths.size(), options, effective_starts);
        const std::size_t max_t = maxPathLength(paths);
        const std::size_t end = std::min(max_t, options.t_end);
        if (options.check_environment) {
            for (std::size_t i = 0; i < paths.size(); ++i) {
                if (effective_starts[i] >= end)
                    continue;
                for (std::size_t t = effective_starts[i]; t < end; ++t) {
                    const auto &config = configAt(paths[i], t);
                    if (!isValidSingle(*robots[i], config, obstacles, cylinders) ||
                        !isSelfCollisionFree(*robots[i], config)) {
                        return false;
                    }
                }
            }
        }

        for (std::size_t i = 0; i < paths.size(); ++i) {
            for (std::size_t j = i + 1; j < paths.size(); ++j) {
                const std::size_t pair_begin = effective_pair_starts[
                    pairFrontierIndex(i, j, paths.size())];
                if (pair_begin >= end)
                    continue;
                if (!isPairPathValid(*robots[i], paths[i], *robots[j], paths[j],
                                     pair_begin, end)) {
                    return false;
                }
            }
        }
        return true;
    }

    std::optional<CompositeConflict> findFirstCompositePathConflict(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        const std::vector<ObstacleSphere> &obstacles,
        const std::vector<ObstacleCylinder> &cylinders,
        std::vector<std::size_t> *next_t_begin_by_robot_out) const override {
        if (paths.size() != robots.size())
            return std::nullopt;

        const auto effective_starts = effectivePathStarts(paths.size(), options);
        const auto effective_pair_starts =
            effectivePairStarts(paths.size(), options, effective_starts);
        initializeNextPathStarts(next_t_begin_by_robot_out, effective_starts);
        const std::size_t max_t = maxPathLength(paths);
        const std::size_t end = std::min(max_t, options.t_end);
        if (effective_starts.empty()) {
            return std::nullopt;
        }

        const std::size_t global_begin =
            *std::min_element(effective_starts.begin(), effective_starts.end());
        ActiveRobotSchedule schedule(effective_starts);
        std::vector<std::size_t> active;
        for (std::size_t t = global_begin; t < end; ++t) {
            schedule.activateThrough(t, active);
            if (options.check_environment) {
                for (const std::size_t robot : active) {
                    const auto &config = configAt(paths[robot], t);
                    if (!isValidSingle(*robots[robot], config, obstacles, cylinders)) {
                        return CompositeConflict{ConflictScope::Environment,
                                                 static_cast<int>(robot), -1, t, 0.0,
                                                 ConflictKind::Vertex, config, {}};
                    }
                    if (!isSelfCollisionFree(*robots[robot], config)) {
                        return CompositeConflict{ConflictScope::Self,
                                                 static_cast<int>(robot), -1, t, 0.0,
                                                 ConflictKind::Vertex, config, {}};
                    }
                }
            }

            for (std::size_t ai = 0; ai < active.size(); ++ai) {
                const std::size_t i = active[ai];
                for (std::size_t aj = ai + 1; aj < active.size(); ++aj) {
                    const std::size_t j = active[aj];
                    if (effective_pair_starts[pairFrontierIndex(
                            i, j, paths.size())] > t)
                        continue;
                    const auto &config_i = configAt(paths[i], t);
                    const auto &config_j = configAt(paths[j], t);
                    if (!isValidPair(*robots[i], config_i, *robots[j], config_j)) {
                        return CompositeConflict{ConflictScope::InterRobot,
                                                 static_cast<int>(i),
                                                 static_cast<int>(j), t, 0.0,
                                                 ConflictKind::Vertex, config_i,
                                                 config_j};
                    }
                }
            }
            assignActivePathStarts(next_t_begin_by_robot_out, active, t + 1);
        }
        return std::nullopt;
    }

    std::vector<CompositeConflict> findInterRobotPathConflictsCompositeScan(
        const std::vector<Path> &paths,
        const std::vector<const RobotModel *> &robots,
        const CompositePathValidationOptions &options,
        std::size_t max_conflicts, bool unique,
        const InterRobotConflictCallback &on_conflict,
        const std::vector<ObstacleSphere> &,
        const std::vector<ObstacleCylinder> &,
        std::vector<std::size_t> *next_t_begin_by_robot_out,
        std::vector<std::size_t> *next_t_begin_by_pair_out) const override {
        std::vector<CompositeConflict> out;
        if (paths.size() != robots.size())
            return out;

        const auto effective_starts = effectivePathStarts(paths.size(), options);
        const auto effective_pair_starts =
            effectivePairStarts(paths.size(), options, effective_starts);
        initializeNextPathStarts(next_t_begin_by_robot_out, effective_starts);
        initializeNextPairStarts(next_t_begin_by_pair_out,
                                 effective_pair_starts);
        const std::size_t max_t = maxPathLength(paths);
        const std::size_t end = std::min(max_t, options.t_end);
        if (effective_pair_starts.empty()) {
            return out;
        }

        const std::size_t global_begin =
            *std::min_element(effective_pair_starts.begin(),
                              effective_pair_starts.end());
        if (global_begin >= end) {
            return out;
        }
        std::vector<char> robot_used(paths.size(), 0);
        std::vector<char> pair_has_accepted(effective_pair_starts.size(), 0);
        std::vector<AcceptedInterRobotConflictClaim> accepted_claims;

        for (std::size_t t = global_begin; t < end; ++t) {
            for (std::size_t i = 0; i < paths.size(); ++i) {
                for (std::size_t j = i + 1; j < paths.size(); ++j) {
                    const std::size_t pair_index =
                        pairFrontierIndex(i, j, paths.size());
                    if (effective_pair_starts[pair_index] > t)
                        continue;
                    if (unique && (robot_used[i] || robot_used[j]))
                        continue;
                    const auto &config_i = configAt(paths[i], t);
                    const auto &config_j = configAt(paths[j], t);
                    if (!isValidPair(*robots[i], config_i, *robots[j],
                                     config_j)) {
                        const CompositeConflict conflict{
                            ConflictScope::InterRobot, static_cast<int>(i),
                            static_cast<int>(j), t, 0.0, ConflictKind::Vertex,
                            config_i, config_j};
                        if (acceptInterRobotConflictCandidate(
                                conflict, on_conflict, unique, robot_used, out,
                                accepted_claims)) {
                            if (next_t_begin_by_pair_out &&
                                !pair_has_accepted[pair_index]) {
                                (*next_t_begin_by_pair_out)[pair_index] = t;
                                pair_has_accepted[pair_index] = 1;
                            }
                            if (max_conflicts > 0 &&
                                out.size() >= max_conflicts) {
                                if (unique) {
                                    assignUniqueConflictFrontier(
                                        next_t_begin_by_robot_out,
                                        effective_starts, accepted_claims);
                                }
                                return out;
                            }
                        }
                    } else if (next_t_begin_by_pair_out &&
                               !pair_has_accepted[pair_index]) {
                        (*next_t_begin_by_pair_out)[pair_index] = t + 1;
                    }
                }
            }
        }
        if (unique) {
            assignUniqueConflictFrontier(next_t_begin_by_robot_out,
                                         effective_starts, accepted_claims);
        }
        return out;
    }

private:
    std::vector<ObstacleSphere> env_spheres_;
    std::vector<ObstacleCylinder> env_cylinders_;
    std::vector<CollisionGeometryPtr> env_geoms_;
    std::vector<Transform3> env_transforms_;
    mutable std::unordered_map<const RobotModel *, RobotPrimCache> robot_cache_;
};

} // namespace

std::unique_ptr<CollisionBackend> makeFclBackend() {
    return std::make_unique<FclBackendImpl>();
}

} // namespace detail
} // namespace comotion
