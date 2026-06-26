#pragma once

#include "comotion/robot/RobotModel.h"
#include <Eigen/Core>
#include <random>
#include <string>
#include <vector>

namespace comotion {

struct IKRequest {
    Eigen::Vector3d target_position;
    std::string ee_link_name;
    std::vector<double> seed_config;  // empty → random seed
    double position_tolerance = 1e-3;
    int max_iterations = 200;
    double step_scale = 0.5;
    double damping = 0.05;
};

struct IKResult {
    bool success = false;
    std::vector<double> config;
    double residual = std::numeric_limits<double>::infinity();
};

class IKSolver {
public:
    explicit IKSolver(const RobotModel &robot);

    IKResult solve(const IKRequest &req) const;

    // Solve with multiple random restarts, returning the first success.
    IKResult solveWithRestarts(const IKRequest &req,
                               std::mt19937 &rng,
                               int num_restarts = 50) const;

private:
    Eigen::Vector3d eePosition(const std::vector<double> &config,
                               int ee_link_idx) const;

    // 3 x numJoints position Jacobian via finite differences.
    Eigen::MatrixXd positionJacobian(const std::vector<double> &config,
                                     int ee_link_idx,
                                     double eps = 1e-6) const;

    std::vector<double> randomConfig(std::mt19937 &rng) const;
    void clampToLimits(std::vector<double> &config) const;

    const RobotModel &robot_;
};

} // namespace comotion
