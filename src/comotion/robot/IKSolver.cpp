#include "comotion/robot/IKSolver.h"
#include <Eigen/Dense>
#include <cmath>

namespace comotion {

IKSolver::IKSolver(const RobotModel &robot) : robot_(robot) {}

Eigen::Vector3d IKSolver::eePosition(const std::vector<double> &config,
                                      int ee_link_idx) const {
    auto transforms = robot_.getLinkTransforms(config);
    return transforms[ee_link_idx].translation();
}

Eigen::MatrixXd IKSolver::positionJacobian(const std::vector<double> &config,
                                            int ee_link_idx,
                                            double eps) const {
    int n = robot_.numJoints();
    Eigen::MatrixXd J(3, n);
    Eigen::Vector3d p0 = eePosition(config, ee_link_idx);

    std::vector<double> perturbed = config;
    for (int i = 0; i < n; ++i) {
        double orig = perturbed[i];
        perturbed[i] = orig + eps;
        Eigen::Vector3d p1 = eePosition(perturbed, ee_link_idx);
        J.col(i) = (p1 - p0) / eps;
        perturbed[i] = orig;
    }
    return J;
}

std::vector<double> IKSolver::randomConfig(std::mt19937 &rng) const {
    int n = robot_.numJoints();
    std::vector<double> cfg(n);
    for (int i = 0; i < n; ++i)
        cfg[i] = std::uniform_real_distribution<double>(
                     robot_.jointLower(i), robot_.jointUpper(i))(rng);
    return cfg;
}

void IKSolver::clampToLimits(std::vector<double> &config) const {
    int n = robot_.numJoints();
    for (int i = 0; i < n; ++i)
        config[i] = std::clamp(config[i], robot_.jointLower(i),
                               robot_.jointUpper(i));
}

IKResult IKSolver::solve(const IKRequest &req) const {
    int ee_idx = robot_.linkIndex(req.ee_link_name);
    if (ee_idx < 0)
        return {};

    int n = robot_.numJoints();
    std::vector<double> q = req.seed_config;
    if (static_cast<int>(q.size()) != n)
        return {};
    clampToLimits(q);

    for (int iter = 0; iter < req.max_iterations; ++iter) {
        Eigen::Vector3d p = eePosition(q, ee_idx);
        Eigen::Vector3d err = req.target_position - p;
        double residual = err.norm();

        if (residual < req.position_tolerance) {
            IKResult res;
            res.success = true;
            res.config = q;
            res.residual = residual;
            return res;
        }

        Eigen::MatrixXd J = positionJacobian(q, ee_idx);

        // Damped least-squares: dq = J^T (J J^T + λ²I)^{-1} e
        Eigen::Matrix3d JJt = J * J.transpose();
        JJt.diagonal().array() += req.damping * req.damping;
        Eigen::Vector3d v = JJt.ldlt().solve(err);
        Eigen::VectorXd dq = J.transpose() * v;

        for (int i = 0; i < n; ++i)
            q[i] += req.step_scale * dq(i);
        clampToLimits(q);
    }

    // Did not converge — return best effort
    Eigen::Vector3d p = eePosition(q, ee_idx);
    IKResult res;
    res.success = false;
    res.config = q;
    res.residual = (req.target_position - p).norm();
    return res;
}

IKResult IKSolver::solveWithRestarts(const IKRequest &req,
                                      std::mt19937 &rng,
                                      int num_restarts) const {
    IKRequest local = req;
    IKResult best;

    for (int r = 0; r < num_restarts; ++r) {
        local.seed_config = randomConfig(rng);
        IKResult res = solve(local);
        if (res.success)
            return res;
        if (res.residual < best.residual)
            best = res;
    }
    return best;
}

} // namespace comotion
