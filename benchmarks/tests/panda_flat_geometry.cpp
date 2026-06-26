#include "panda_flat_builtin_tasks.hpp"

#include "cage_scene_json.hpp"
#include "comotion/collision/CollisionChecker.h"
#include "comotion/robot/RobotModel.h"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using json = nlohmann::json;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool near(double a, double b, double eps = 1e-9) {
    return std::abs(a - b) <= eps;
}

bool expect(bool condition, const std::string &message) {
    if (condition)
        return true;
    std::cerr << "panda_flat_geometry: " << message << "\n";
    return false;
}

std::string getResourcePath(const std::string &relative) {
    const char *prefixes[] = {"../resources/", "../../resources/",
                              "../../../resources/", "resources/"};
    for (const char *prefix : prefixes) {
        const std::string path = std::string(prefix) + relative;
        std::ifstream file(path);
        if (file.good())
            return path;
    }
    return std::string("resources/") + relative;
}

double yawFromBase(const json &base) {
    const auto &qj = base.at("quaternion_xyzw");
    Eigen::Quaterniond q(qj.at(3).get<double>(), qj.at(0).get<double>(),
                         qj.at(1).get<double>(), qj.at(2).get<double>());
    q.normalize();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

std::shared_ptr<comotion::RobotModel> loadPanda(const Eigen::Affine3d &base) {
    auto robot = std::make_shared<comotion::RobotModel>();
    robot->loadURDF(getResourcePath("panda/panda_spherized.urdf"));
    robot->loadSRDF(getResourcePath("panda/panda.srdf"));
    robot->setBaseTransform(base);
    return robot;
}

bool checkDoc(int num_robots, int expected_rows, int expected_cols) {
    const json doc = json::parse(
        comotion::benchmark_apps::panda_flat_builtin::taskJsonForRobotCount(
            num_robots));
    bool ok = true;
    ok &= expect(doc.at("num_robots").get<int>() == num_robots,
                 "num_robots mismatch");
    ok &= expect(doc.at("num_tasks").get<int>() == 5,
                 "num_tasks mismatch");
    ok &= expect(doc.at("tasks").size() == 5, "task count mismatch");
    ok &= expect(doc.at("layout").at("rows").get<int>() == expected_rows,
                 "layout rows mismatch");
    ok &= expect(doc.at("layout").at("cols").get<int>() == expected_cols,
                 "layout cols mismatch");
    ok &= expect(near(doc.at("layout").at("spacing").get<double>(), 0.75),
                 "layout spacing mismatch");

    const auto &flat_base = doc.at("flat_base");
    ok &= expect(flat_base.at("type").get<std::string>() == "cylinder",
                 "flat_base type mismatch");
    ok &= expect(near(flat_base.at("center").at(2).get<double>(), -0.10),
                 "flat_base center z mismatch");
    ok &= expect(near(flat_base.at("half_height").get<double>(), 0.05),
                 "flat_base half height mismatch");

    const auto &bases = doc.at("robot_bases");
    ok &= expect(bases.size() == static_cast<std::size_t>(num_robots),
                 "base count mismatch");
    const double spacing = 0.75;
    for (int row = 0; row < expected_rows; ++row) {
        const double y = (static_cast<double>(row) -
                          0.5 * static_cast<double>(expected_rows - 1)) *
                         spacing;
        const double expected_yaw = y < 0.0 ? 0.5 * kPi : -0.5 * kPi;
        for (int col = 0; col < expected_cols; ++col) {
            const std::size_t index =
                static_cast<std::size_t>(row * expected_cols + col);
            const double x = (static_cast<double>(col) -
                              0.5 * static_cast<double>(expected_cols - 1)) *
                             spacing;
            const auto &position = bases.at(index).at("position");
            ok &= expect(near(position.at(0).get<double>(), x),
                         "base x mismatch");
            ok &= expect(near(position.at(1).get<double>(), y),
                         "base y mismatch");
            ok &= expect(near(position.at(2).get<double>(), 0.0),
                         "base z mismatch");
            ok &= expect(near(yawFromBase(bases.at(index)), expected_yaw),
                         "base yaw mismatch");
        }
    }

    const auto base_transforms = cage_scene::parseRobotBasesArray(bases);
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    robots.reserve(base_transforms.size());
    for (const auto &base : base_transforms)
        robots.push_back(loadPanda(base));

    comotion::CollisionChecker checker(comotion::CollisionChecker::Backend::Spheres);
    checker.setCylinderObstacles(cage_scene::parseCylinderObstacles(doc));

    for (std::size_t task_index = 0; task_index < doc.at("tasks").size();
         ++task_index) {
        const auto starts =
            doc.at("tasks").at(task_index)
                .at("starts")
                .get<std::vector<std::vector<double>>>();
        const auto goals =
            doc.at("tasks").at(task_index)
                .at("goals")
                .get<std::vector<std::vector<double>>>();
        ok &= expect(starts.size() == robots.size(), "start vector size mismatch");
        ok &= expect(goals.size() == robots.size(), "goal vector size mismatch");
        for (std::size_t i = 0; i < robots.size(); ++i) {
            ok &= expect(starts[i].size() == 7, "start config size mismatch");
            ok &= expect(goals[i].size() == 7, "goal config size mismatch");
            ok &= expect(checker.isValidSingleFull(*robots[i], starts[i]),
                         "start endpoint must be valid");
            ok &= expect(checker.isValidSingleFull(*robots[i], goals[i]),
                         "goal endpoint must be valid");
            for (std::size_t j = i + 1; j < robots.size(); ++j) {
                ok &= expect(checker.isValidPair(*robots[i], starts[i],
                                                 *robots[j], starts[j]),
                             "start pair must be valid");
                ok &= expect(checker.isValidPair(*robots[i], goals[i],
                                                 *robots[j], goals[j]),
                             "goal pair must be valid");
            }
        }
    }
    return ok;
}

} // namespace

int main() {
    bool ok = true;
    ok &= checkDoc(4, 2, 2);
    ok &= checkDoc(8, 2, 4);
    ok &= checkDoc(16, 4, 4);
    try {
        (void)comotion::benchmark_apps::panda_flat_builtin::taskJsonForRobotCount(2);
        ok &= expect(false, "unsupported robot count must throw");
    } catch (const std::runtime_error &) {
    }
    if (!ok)
        return 1;
    std::cout << "panda_flat_geometry: OK\n";
    return 0;
}
