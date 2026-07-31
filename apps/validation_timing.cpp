#include "benchmark_app_common.hpp"
#include "cage_scene_json.hpp"
#include "panda_cage_builtin_tasks.hpp"

#include "comotion/collision/CollisionChecker.h"
#include "comotion/collision/ValidationTrace.h"
#include "comotion/planning/CompositeRRT.h"
#include "comotion/planning/MultiRobotProblem.h"
#include "comotion/planning/PlanningRng.h"
#include "comotion/robot/RobotModel.h"

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <ompl/base/ScopedState.h>
#include <ompl/datastructures/NearestNeighborsGNATNoThreadSafety.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

using json = nlohmann::json;
namespace builtin = comotion::benchmark_apps::panda_cage_builtin;
namespace common = comotion::benchmark_apps::common;

namespace {

using Clock = std::chrono::steady_clock;

enum class TimingClock {
    Wall,
    ProcessCpu,
};

std::filesystem::path g_executable_dir;

struct AppOptions {
    std::string mode = "trace-replay";
    int num_robots = 4;
    int task_index = 0;
    std::vector<std::uint32_t> seeds{0, 1, 2};
    unsigned iterations = 1000;
    double time_limit = 1000.0;
    double composite_rrt_range = 0.0;
    bool composite_rrt_use_makespan_metric = false;
    std::size_t resolution = 128;
    comotion::CollisionChecker::Backend trace_backend =
        comotion::CollisionChecker::Backend::Spheres;
    std::string trace_output =
        "benchmarks/results/validation_timing_trace.json";
    std::string trace_input;
    std::string metrics_json =
        "benchmarks/results/validation_timing_metrics.json";
    std::vector<std::string> replay_variants;
    bool empty_environment = false;
    bool motion_only = false;
    bool classify_scopes = false;
    bool exhaustive_motion_validation = false;
    std::string fcl_urdf_rel = "panda/panda.urdf";
    std::size_t sample_motion_count = 1000;
    std::size_t sample_vertex_batch_size = 200;
    std::size_t sample_min_timesteps = 100;
    std::size_t sample_max_timesteps = 700;
    std::size_t sample_max_batches = 1000;
    std::size_t sample_max_state_attempts_per_batch = 10000000;
    std::size_t sample_uniform_first_batch_attempt_cap = 10000;
    std::size_t sample_incremental_first_batch_attempt_cap = 10000;
    std::size_t sample_progress_interval = 1000;
    TimingClock timing_clock = TimingClock::Wall;
    std::size_t progress_interval = 50;
    bool verbose = false;
};

struct PandaCageScenario {
    int num_robots = 0;
    int task_index = 0;
    std::string task_source;
    std::string urdf_rel = "panda/panda_spherized.urdf";
    std::string srdf_rel = "panda/panda.srdf";
    std::vector<std::vector<double>> starts;
    std::vector<std::vector<double>> goals;
    json robot_bases = json::array();
    cage_scene::Affine3dVector base_transforms;
    std::vector<comotion::ObstacleSphere> sphere_obstacles;
    std::vector<comotion::ObstacleCylinder> cylinder_obstacles;
};

struct TraceRecordWithSeed {
    std::uint32_t seed = 0;
    comotion::ValidationTraceRecord record;
};

struct TraceCorpus {
    json metadata = json::object();
    std::vector<TraceRecordWithSeed> records;
};

struct DistributionSummary {
    std::size_t count = 0;
    double min = 0.0;
    double p50 = 0.0;
    double p90 = 0.0;
    double p99 = 0.0;
    double max = 0.0;
    double mean = 0.0;
};

struct TypeStats {
    std::uint64_t count = 0;
    std::uint64_t valid = 0;
    std::uint64_t invalid = 0;
    std::uint64_t elapsed_ns = 0;
    std::uint64_t valid_elapsed_ns = 0;
    std::uint64_t invalid_elapsed_ns = 0;
    std::uint64_t result_mismatches = 0;
    std::map<std::string, std::uint64_t> scope_counts;
    std::vector<double> latencies_seconds;
    std::vector<double> path_lengths;
    std::vector<double> timestep_counts;
    comotion::ValidationWorkStats work;
};

struct VariantStats {
    std::string name;
    std::string backend;
    std::uint64_t elapsed_ns = 0;
    std::uint64_t valid_elapsed_ns = 0;
    std::uint64_t invalid_elapsed_ns = 0;
    std::uint64_t count = 0;
    std::uint64_t valid = 0;
    std::uint64_t invalid = 0;
    std::uint64_t result_mismatches = 0;
    std::map<std::string, TypeStats> by_type;
    comotion::ValidationWorkStats work;
};

std::uint64_t elapsedNanoseconds(Clock::time_point start) {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now() - start)
            .count());
}

std::uint64_t processCpuNanoseconds() {
    timespec ts{};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) != 0) {
        throw std::runtime_error("clock_gettime(CLOCK_PROCESS_CPUTIME_ID) failed");
    }
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull +
           static_cast<std::uint64_t>(ts.tv_nsec);
}

struct ValidationTimer {
    TimingClock clock = TimingClock::Wall;
    Clock::time_point wall_start;
    std::uint64_t cpu_start_ns = 0;
};

ValidationTimer startValidationTimer(TimingClock clock) {
    ValidationTimer timer;
    timer.clock = clock;
    if (clock == TimingClock::ProcessCpu) {
        timer.cpu_start_ns = processCpuNanoseconds();
    } else {
        timer.wall_start = Clock::now();
    }
    return timer;
}

std::uint64_t elapsedValidationNanoseconds(const ValidationTimer &timer) {
    if (timer.clock == TimingClock::ProcessCpu) {
        return processCpuNanoseconds() - timer.cpu_start_ns;
    }
    return elapsedNanoseconds(timer.wall_start);
}

std::string timingClockName(TimingClock clock) {
    switch (clock) {
    case TimingClock::Wall:
        return "wall";
    case TimingClock::ProcessCpu:
        return "cpu";
    }
    return "wall";
}

TimingClock parseTimingClock(const std::string &value) {
    if (value == "wall")
        return TimingClock::Wall;
    if (value == "cpu" || value == "process_cpu")
        return TimingClock::ProcessCpu;
    throw std::runtime_error("Unknown timing clock '" + value +
                             "'. Expected wall or cpu");
}

void setExecutablePath(const char *argv0) {
    if (!argv0 || std::string(argv0).empty())
        return;
    std::error_code ec;
    const auto path = std::filesystem::weakly_canonical(
        std::filesystem::absolute(argv0), ec);
    g_executable_dir = (ec ? std::filesystem::absolute(argv0) : path).parent_path();
}

std::string getResourcePath(const std::string &relative) {
    const std::filesystem::path rel(relative);
    std::vector<std::filesystem::path> candidates;
    if (rel.is_absolute()) {
        candidates.push_back(rel);
    } else if (!g_executable_dir.empty()) {
        candidates.push_back(g_executable_dir / ".." / "share" / "comotion" /
                             "resources" / rel);
        candidates.push_back(g_executable_dir / ".." / ".." / "resources" / rel);
        candidates.push_back(g_executable_dir / ".." / "resources" / rel);
    }
    for (const char *prefix : {"resources/", "../resources/",
                               "../../resources/"}) {
        candidates.emplace_back(std::string(prefix) + relative);
    }
    for (const auto &candidate : candidates) {
        const auto normalized = candidate.lexically_normal();
        std::ifstream file(normalized);
        if (file.good())
            return normalized.string();
    }
    return std::string("resources/") + relative;
}

std::string requireValue(int &i, int argc, char **argv,
                         const std::string &arg) {
    if (i + 1 >= argc)
        throw std::runtime_error(arg + " requires a value");
    return argv[++i];
}

std::vector<std::uint32_t> parseSeeds(const std::string &value) {
    std::vector<std::uint32_t> seeds;
    std::stringstream ss(value);
    std::string token;
    while (std::getline(ss, token, ',')) {
        if (token.empty())
            continue;
        seeds.push_back(static_cast<std::uint32_t>(std::stoul(token)));
    }
    if (seeds.empty())
        throw std::runtime_error("--seeds must provide at least one seed");
    return seeds;
}

std::vector<std::string> parseCsvStrings(const std::string &value) {
    std::vector<std::string> out;
    std::stringstream ss(value);
    std::string token;
    while (std::getline(ss, token, ',')) {
        if (!token.empty())
            out.push_back(token);
    }
    return out;
}

Eigen::Vector3d vector3FromJson(const json &value) {
    return Eigen::Vector3d(value.at(0).get<double>(),
                           value.at(1).get<double>(),
                           value.at(2).get<double>());
}

std::vector<comotion::ObstacleSphere> parseSphereObstacles(const json &doc) {
    std::vector<comotion::ObstacleSphere> spheres;
    if (!doc.contains("obstacles"))
        return spheres;
    for (const auto &obs : doc.at("obstacles")) {
        if (obs.value("type", "") != "sphere")
            continue;
        comotion::ObstacleSphere sphere;
        sphere.center = vector3FromJson(obs.at("center"));
        sphere.radius = obs.at("radius").get<double>();
        spheres.push_back(sphere);
    }
    return spheres;
}

PandaCageScenario loadPandaCageScenario(int num_robots, int task_index) {
    json doc = json::parse(builtin::taskJsonForRobotCount(num_robots));
    const auto &tasks = doc.at("tasks");
    if (task_index < 0 || task_index >= static_cast<int>(tasks.size())) {
        throw std::runtime_error("Panda cage task index out of range");
    }
    const auto &task = tasks.at(static_cast<std::size_t>(task_index));

    PandaCageScenario scenario;
    scenario.num_robots = num_robots;
    scenario.task_index = task_index;
    scenario.task_source = "panda_cage_builtin";
    scenario.urdf_rel = doc.value("urdf_path", scenario.urdf_rel);
    scenario.srdf_rel = doc.value("srdf_path", scenario.srdf_rel);
    scenario.starts = task.at("starts").get<std::vector<std::vector<double>>>();
    scenario.goals = task.at("goals").get<std::vector<std::vector<double>>>();
    scenario.robot_bases = doc.at("robot_bases");
    scenario.base_transforms =
        cage_scene::parseRobotBasesArray(scenario.robot_bases);
    scenario.sphere_obstacles = parseSphereObstacles(doc);
    scenario.cylinder_obstacles = cage_scene::parseCylinderObstacles(doc);

    if (static_cast<int>(scenario.starts.size()) != num_robots ||
        static_cast<int>(scenario.goals.size()) != num_robots ||
        static_cast<int>(scenario.base_transforms.size()) != num_robots) {
        throw std::runtime_error("Panda cage built-in task size mismatch");
    }
    return scenario;
}

std::vector<std::shared_ptr<comotion::RobotModel>>
loadPandaRobots(const PandaCageScenario &scenario,
                comotion::CollisionChecker::Backend backend,
                const std::string &fcl_urdf_rel) {
    const std::string &urdf_rel =
        backend == comotion::CollisionChecker::Backend::Fcl
            ? fcl_urdf_rel
            : scenario.urdf_rel;
    const std::string urdf = getResourcePath(urdf_rel);
    const std::string srdf = getResourcePath(scenario.srdf_rel);
    std::vector<std::shared_ptr<comotion::RobotModel>> robots;
    robots.reserve(scenario.base_transforms.size());
    for (const auto &base : scenario.base_transforms) {
        auto robot = std::make_shared<comotion::RobotModel>();
        robot->loadURDF(urdf);
        robot->loadSRDF(srdf);
        robot->setBaseTransform(base);
        robots.push_back(std::move(robot));
    }
    return robots;
}

std::shared_ptr<comotion::MultiRobotProblem>
makeProblem(const PandaCageScenario &scenario,
            comotion::CollisionChecker::Backend backend,
            std::size_t resolution, bool empty_environment,
            const std::string &fcl_urdf_rel) {
    auto problem = std::make_shared<comotion::MultiRobotProblem>(backend);
    problem->setResolution(resolution);
    const auto robots = loadPandaRobots(scenario, backend, fcl_urdf_rel);
    for (int i = 0; i < scenario.num_robots; ++i) {
        const auto index = static_cast<std::size_t>(i);
        problem->addRobot(robots[index], scenario.starts[index],
                          scenario.goals[index]);
    }
    if (!empty_environment) {
        problem->setObstacles(scenario.sphere_obstacles);
        problem->setCylinderObstacles(scenario.cylinder_obstacles);
    }
    return problem;
}

json optionsToJson(const comotion::CompositePathValidationOptions &options) {
    return {
        {"check_environment", options.check_environment},
        {"exhaustive", options.exhaustive},
        {"discrete_num_checks_hint", options.discrete_num_checks_hint},
        {"t_begin", options.t_begin},
        {"t_end", options.t_end},
        {"per_path_t_begin", options.per_path_t_begin},
        {"per_pair_t_begin", options.per_pair_t_begin},
        {"conflict_find_parallel_workers",
         options.conflict_find_parallel_workers},
        {"conflict_find_parallel_horizon",
         options.conflict_find_parallel_horizon},
        {"conflict_find_parallel_assignment",
         static_cast<int>(options.conflict_find_parallel_assignment)},
    };
}

comotion::CompositePathValidationOptions optionsFromJson(const json &value) {
    comotion::CompositePathValidationOptions options;
    options.check_environment = value.value("check_environment", true);
    options.exhaustive = value.value("exhaustive", false);
    options.discrete_num_checks_hint =
        value.value("discrete_num_checks_hint", -1);
    options.t_begin = value.value("t_begin", std::size_t{0});
    options.t_end = value.value(
        "t_end", std::numeric_limits<std::size_t>::max());
    options.per_path_t_begin =
        value.value("per_path_t_begin", std::vector<std::size_t>{});
    options.per_pair_t_begin =
        value.value("per_pair_t_begin", std::vector<std::size_t>{});
    options.conflict_find_parallel_workers =
        value.value("conflict_find_parallel_workers", std::size_t{1});
    options.conflict_find_parallel_horizon =
        value.value("conflict_find_parallel_horizon", std::size_t{0});
    options.conflict_find_parallel_assignment =
        static_cast<comotion::ConflictFindParallelAssignment>(
            value.value("conflict_find_parallel_assignment", 0));
    return options;
}

json recordToJson(std::uint32_t seed,
                  const comotion::ValidationTraceRecord &record) {
    json out;
    out["seed"] = seed;
    out["type"] = comotion::validationTraceCallTypeName(record.type);
    out["result"] = record.result;
    out["elapsed_nanoseconds"] = record.elapsed_nanoseconds;
    if (record.type == comotion::ValidationTraceCallType::CompositeState) {
        out["configs"] = record.configs;
    } else {
        out["from"] = record.from;
        out["to"] = record.to;
        out["options"] = optionsToJson(record.options);
    }
    return out;
}

TraceRecordWithSeed recordFromJson(const json &value) {
    TraceRecordWithSeed out;
    out.seed = value.at("seed").get<std::uint32_t>();
    out.record.type =
        comotion::parseValidationTraceCallType(value.at("type").get<std::string>());
    out.record.result = value.at("result").get<bool>();
    out.record.elapsed_nanoseconds =
        value.value("elapsed_nanoseconds", std::uint64_t{0});
    if (out.record.type == comotion::ValidationTraceCallType::CompositeState) {
        out.record.configs =
            value.at("configs").get<std::vector<std::vector<double>>>();
    } else {
        out.record.from =
            value.at("from").get<std::vector<std::vector<double>>>();
        out.record.to = value.at("to").get<std::vector<std::vector<double>>>();
        out.record.options = optionsFromJson(value.at("options"));
    }
    return out;
}

void writeJson(const json &doc, const std::filesystem::path &path) {
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    if (!out)
        throw std::runtime_error("Failed to open " + path.string());
    out << doc.dump(2) << "\n";
}

json runTrace(const AppOptions &options,
              const PandaCageScenario &scenario) {
    json records = json::array();
    json seed_summaries = json::array();

    for (std::uint32_t seed : options.seeds) {
        auto problem = makeProblem(
            scenario, options.trace_backend, options.resolution,
            options.empty_environment, options.fcl_urdf_rel);
        auto recorder = std::make_shared<comotion::ValidationTraceRecorder>();
        problem->collisionChecker().setValidationTraceRecorder(recorder);

        comotion::CompositeRRT planner;
        planner.setProblem(problem);
        planner.setPlanningSeed(seed);
        planner.setSimplifySolution(false);
        planner.setUseMakespanMetric(options.composite_rrt_use_makespan_metric);
        planner.setMaxRrtConnectIterations(options.iterations);
        planner.setContinueAfterSolutionUntilIterationCap(true);
        if (options.composite_rrt_range > 0.0)
            planner.setRange(options.composite_rrt_range);

        comotion::seedOmplGlobalFromUserPlanningSeed(seed);
        const auto status = planner.solve(options.time_limit);

        std::uint64_t seed_valid = 0;
        std::uint64_t seed_invalid = 0;
        for (const auto &record : recorder->records()) {
            records.push_back(recordToJson(seed, record));
            if (record.result)
                ++seed_valid;
            else
                ++seed_invalid;
        }

        seed_summaries.push_back({
            {"seed", seed},
            {"planner_status", status.asString()},
            {"record_count", recorder->records().size()},
            {"valid", seed_valid},
            {"invalid", seed_invalid},
            {"planner_stats", planner.plannerStatsJson()},
        });
        if (options.verbose) {
            std::cout << "trace seed=" << seed
                      << " status=" << status.asString()
                      << " records=" << recorder->records().size();
            const auto &stats = planner.plannerStatsJson();
            if (stats.contains("rrt_connect_iterations")) {
                std::cout << " rrt_iterations="
                          << stats.at("rrt_connect_iterations");
            }
            std::cout << "\n";
        }
    }

    json doc;
    doc["schema"] = "comotion.validation_trace.v1";
    doc["source"] = "composite_rrt_panda_cage";
    doc["scenario"] = {
        {"suite", "panda_cage"},
        {"num_robots", scenario.num_robots},
        {"task_index", scenario.task_index},
        {"task_source", scenario.task_source},
    };
    doc["trace_backend"] = common::backendName(options.trace_backend);
    doc["robot_urdf_resource"] =
        options.trace_backend == comotion::CollisionChecker::Backend::Fcl
            ? options.fcl_urdf_rel
            : scenario.urdf_rel;
    doc["robot_collision_geometry"] =
        options.trace_backend == comotion::CollisionChecker::Backend::Fcl
            ? "mesh"
            : "sphere";
    doc["empty_environment"] = options.empty_environment;
    doc["planner"] = "CompositeRRT";
    doc["iterations"] = options.iterations;
    doc["iteration_mode"] = "continue_after_solution_until_cap";
    doc["rrt_connect_range_requested"] = options.composite_rrt_range;
    doc["composite_rrt_use_makespan_metric"] =
        options.composite_rrt_use_makespan_metric;
    doc["time_limit_seconds"] = options.time_limit;
    doc["resolution"] = options.resolution;
    doc["seeds"] = options.seeds;
    doc["seed_summaries"] = seed_summaries;
    doc["records"] = records;
    writeJson(doc, options.trace_output);
    return doc;
}

TraceCorpus loadTraceCorpus(const std::filesystem::path &path) {
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("Failed to open trace " + path.string());
    json doc = json::parse(in);
    if (doc.value("schema", "") != "comotion.validation_trace.v1")
        throw std::runtime_error("Unsupported validation trace schema");

    TraceCorpus corpus;
    corpus.metadata = doc;
    for (const auto &entry : doc.at("records"))
        corpus.records.push_back(recordFromJson(entry));
    return corpus;
}

TraceCorpus loadTraceCorpora(const std::string &paths_csv) {
    const auto paths = parseCsvStrings(paths_csv);
    if (paths.empty())
        throw std::runtime_error("No trace input paths provided");

    TraceCorpus merged;
    bool first = true;
    json seeds = json::array();
    json seed_summaries = json::array();
    for (const auto &path : paths) {
        TraceCorpus corpus = loadTraceCorpus(path);
        const bool corpus_empty_environment =
            corpus.metadata.value("empty_environment", false);
        if (first) {
            merged.metadata = corpus.metadata;
            merged.metadata["records"] = json::array();
            first = false;
        } else if (merged.metadata.value("empty_environment", false) !=
                   corpus_empty_environment) {
            throw std::runtime_error(
                "Cannot merge traces with different empty_environment settings");
        }
        for (const auto &seed : corpus.metadata.value("seeds", json::array()))
            seeds.push_back(seed);
        for (const auto &summary :
             corpus.metadata.value("seed_summaries", json::array())) {
            seed_summaries.push_back(summary);
        }
        merged.records.insert(merged.records.end(), corpus.records.begin(),
                              corpus.records.end());
    }
    merged.metadata["seeds"] = seeds;
    merged.metadata["seed_summaries"] = seed_summaries;
    return merged;
}

double maxRobotConfigDistance(const std::vector<std::vector<double>> &from,
                              const std::vector<std::vector<double>> &to) {
    double max_distance = 0.0;
    for (std::size_t i = 0; i < from.size(); ++i) {
        double dist_sq = 0.0;
        for (std::size_t d = 0; d < from[i].size(); ++d) {
            const double diff = to[i][d] - from[i][d];
            dist_sq += diff * diff;
        }
        max_distance = std::max(max_distance, std::sqrt(dist_sq));
    }
    return max_distance;
}

struct SampledCompositeVertex {
    std::vector<std::vector<double>> configs;
};

struct SampledMotionCandidate {
    std::size_t first = 0;
    std::size_t second = 0;
    std::size_t timestep_count = 0;
    double distance = 0.0;
};

std::vector<std::size_t>
sampleDistributionBoundaries(std::size_t minimum,
                             std::size_t maximum) {
    if (minimum >= maximum)
        throw std::runtime_error(
            "sample minimum timesteps must be below maximum timesteps");
    const double span = static_cast<double>(maximum - minimum);
    const std::array<double, 5> interior_fractions{
        0.25, 1.0 / 2.4, 7.0 / 12.0, 0.75, 0.875};
    std::vector<std::size_t> boundaries{minimum};
    for (const double fraction : interior_fractions) {
        const auto boundary = minimum + static_cast<std::size_t>(
                                            std::llround(span * fraction));
        if (boundary <= boundaries.back() || boundary > maximum)
            throw std::runtime_error(
                "sample timestep range is too small for target bins");
        boundaries.push_back(boundary);
    }
    boundaries.push_back(maximum + 1);
    return boundaries;
}

std::vector<std::size_t>
sampleDistributionQuotas(std::size_t motion_count) {
    constexpr std::array<double, 6> weights{
        0.05, 0.10, 0.20, 0.40, 0.20, 0.05};
    std::vector<std::size_t> quotas(weights.size(), 0);
    std::vector<std::pair<double, std::size_t>> remainders;
    remainders.reserve(weights.size());
    std::size_t assigned = 0;
    for (std::size_t i = 0; i < weights.size(); ++i) {
        const double exact = weights[i] * static_cast<double>(motion_count);
        quotas[i] = static_cast<std::size_t>(std::floor(exact));
        assigned += quotas[i];
        remainders.emplace_back(exact - static_cast<double>(quotas[i]), i);
    }
    std::sort(remainders.begin(), remainders.end(),
              [](const auto &lhs, const auto &rhs) {
                  if (lhs.first != rhs.first)
                      return lhs.first > rhs.first;
                  return lhs.second < rhs.second;
              });
    for (std::size_t i = 0; assigned < motion_count; ++i, ++assigned)
        ++quotas[remainders[i % remainders.size()].second];
    return quotas;
}

std::vector<std::vector<double>>
compositeConfigsFromState(
    const ompl::base::State *state,
    const comotion::MultiRobotProblem &problem) {
    const auto *real_state =
        state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<std::vector<double>> configs;
    configs.reserve(static_cast<std::size_t>(problem.numRobots()));
    std::size_t offset = 0;
    for (int robot_index = 0; robot_index < problem.numRobots();
         ++robot_index) {
        const int dimensions = problem.robot(robot_index).model->numJoints();
        std::vector<double> config(static_cast<std::size_t>(dimensions));
        for (int dimension = 0; dimension < dimensions; ++dimension) {
            config[static_cast<std::size_t>(dimension)] =
                real_state->values[offset +
                                   static_cast<std::size_t>(dimension)];
        }
        offset += static_cast<std::size_t>(dimensions);
        configs.push_back(std::move(config));
    }
    return configs;
}

json runSampleTrace(const AppOptions &options,
                    const PandaCageScenario &scenario) {
    if (options.seeds.size() != 1) {
        throw std::runtime_error(
            "sample mode requires exactly one seed");
    }
    const std::uint32_t seed = options.seeds.front();
    comotion::seedOmplGlobalFromUserPlanningSeed(seed);
    auto problem = makeProblem(
        scenario, options.trace_backend, options.resolution,
        options.empty_environment, options.fcl_urdf_rel);
    const auto robots = problem->robotModelPtrs();
    std::vector<int> robot_indices(
        static_cast<std::size_t>(problem->numRobots()));
    std::iota(robot_indices.begin(), robot_indices.end(), 0);
    auto space_information =
        problem->createCompositeSpaceInfo(robot_indices);

    auto sampler = space_information->allocStateSampler();
    ompl::base::ScopedState<> sampled_state(
        space_information->getStateSpace());

    const auto boundaries = sampleDistributionBoundaries(
        options.sample_min_timesteps, options.sample_max_timesteps);
    const auto quotas =
        sampleDistributionQuotas(options.sample_motion_count);
    std::vector<std::vector<SampledMotionCandidate>> candidates(
        quotas.size());
    std::vector<SampledCompositeVertex> vertices;
    vertices.reserve(options.sample_vertex_batch_size);

    ompl::NearestNeighborsGNATNoThreadSafety<std::size_t> neighbors;
    neighbors.setDistanceFunction(
        [&vertices](const std::size_t &lhs, const std::size_t &rhs) {
            return maxRobotConfigDistance(vertices[lhs].configs,
                                          vertices[rhs].configs);
        });

    const double resolution = static_cast<double>(options.resolution);
    const double vmax = problem->vmax();
    if (vmax <= 0.0)
        throw std::runtime_error("sample mode requires positive vmax");
    const double radius =
        static_cast<double>(options.sample_max_timesteps - 1) * vmax /
        resolution;

    std::unordered_set<std::uint64_t> seen_pairs;
    std::size_t batches = 0;
    std::uint64_t state_attempts = 0;
    std::uint64_t uniform_state_attempts = 0;
    std::uint64_t first_batch_uniform_state_attempts = 0;
    std::uint64_t first_batch_uniform_valid_vertices = 0;
    std::uint64_t incremental_composite_attempts = 0;
    std::uint64_t incremental_completed_composites = 0;
    std::uint64_t incremental_restarts = 0;
    std::uint64_t incremental_valid_vertices = 0;
    std::uint64_t incremental_final_validation_failures = 0;
    std::uint64_t incremental_robot_config_attempts = 0;
    std::uint64_t anchored_robot_state_attempts = 0;
    std::uint64_t local_state_attempts = 0;
    std::uint64_t uniform_valid_vertices = 0;
    std::uint64_t anchored_robot_valid_vertices = 0;
    std::uint64_t local_valid_vertices = 0;
    std::uint64_t neighbor_pairs_considered = 0;
    std::vector<std::uint64_t> incremental_robot_attempts(robots.size(), 0);
    std::vector<std::uint64_t> incremental_robot_placements(robots.size(), 0);
    std::vector<std::uint64_t> incremental_robot_single_rejections(
        robots.size(), 0);
    std::vector<std::uint64_t> incremental_robot_pair_rejections(
        robots.size(), 0);
    std::vector<std::uint64_t> incremental_robot_retry_exhaustions(
        robots.size(), 0);
    std::vector<std::size_t> incremental_robot_retry_limits(
        robots.size(), 1);
    std::vector<std::size_t> robot_config_offsets(robots.size(), 0);
    std::vector<std::size_t> robot_config_dimensions(robots.size(), 0);
    std::size_t config_offset = 0;
    for (std::size_t robot_index = 0; robot_index < robots.size();
         ++robot_index) {
        robot_config_offsets[robot_index] = config_offset;
        robot_config_dimensions[robot_index] =
            static_cast<std::size_t>(robots[robot_index]->numJoints());
        config_offset += robot_config_dimensions[robot_index];
        incremental_robot_retry_limits[robot_index] =
            std::max<std::size_t>(
                1, static_cast<std::size_t>(std::ceil(std::pow(
                       10.0, static_cast<double>(robot_index) / 4.0))));
    }
    std::vector<std::vector<std::vector<double>>> valid_seed_anchors;
    for (const auto *configs : {&scenario.starts, &scenario.goals}) {
        if (configs->size() == robots.size() &&
            problem->collisionChecker().isValidComposite(
                robots, *configs)) {
            valid_seed_anchors.push_back(*configs);
        }
    }
    if (valid_seed_anchors.empty()) {
        throw std::runtime_error(
            "sample mode requires at least one valid scenario endpoint "
            "as a high-dimensional sampling anchor");
    }
    std::mt19937 proposal_rng(seed ^ 0x85ebca6bu);
    std::bernoulli_distribution choose_local(0.9);
    const auto quotas_met = [&]() {
        for (std::size_t i = 0; i < quotas.size(); ++i) {
            if (candidates[i].size() < quotas[i])
                return false;
        }
        return true;
    };
    bool incremental_summary_printed = false;
    const auto print_incremental_summary = [&]() {
        if (options.sample_progress_interval == 0 ||
            incremental_summary_printed ||
            incremental_composite_attempts == 0) {
            return;
        }
        incremental_summary_printed = true;
        const double composite_success_rate =
            static_cast<double>(incremental_valid_vertices) /
            static_cast<double>(incremental_composite_attempts);
        std::cout
            << "sample incremental summary composite_attempts="
            << incremental_composite_attempts
            << " completed_composites="
            << incremental_completed_composites
            << " accepted_vertices=" << incremental_valid_vertices
            << " restarts=" << incremental_restarts
            << " robot_config_attempts="
            << incremental_robot_config_attempts
            << " final_validation_failures="
            << incremental_final_validation_failures
            << " success_rate=" << std::fixed << std::setprecision(6)
            << composite_success_rate << "\n";
        for (std::size_t robot_index = 0;
             robot_index < robots.size(); ++robot_index) {
            const auto attempts =
                incremental_robot_attempts[robot_index];
            const double placement_rate =
                attempts == 0
                    ? 0.0
                    : static_cast<double>(
                          incremental_robot_placements[robot_index]) /
                          static_cast<double>(attempts);
            std::cout
                << "sample incremental robot=" << robot_index
                << " retry_limit="
                << incremental_robot_retry_limits[robot_index]
                << " attempts=" << attempts
                << " placements="
                << incremental_robot_placements[robot_index]
                << " single_rejections="
                << incremental_robot_single_rejections[robot_index]
                << " pair_rejections="
                << incremental_robot_pair_rejections[robot_index]
                << " retry_exhaustions="
                << incremental_robot_retry_exhaustions[robot_index]
                << " placement_rate=" << std::fixed
                << std::setprecision(6) << placement_rate << "\n";
        }
        std::cout << std::flush;
    };
    const auto print_first_batch_progress =
        [&](const char *stage, std::uint64_t stage_attempts,
            std::uint64_t stage_valid_vertices) {
            if (options.sample_progress_interval == 0 ||
                stage_attempts == 0 ||
                stage_attempts % options.sample_progress_interval != 0) {
                return;
            }
            const double success_rate =
                static_cast<double>(stage_valid_vertices) /
                static_cast<double>(stage_attempts);
            std::cout << "sample progress batch=1 stage=" << stage
                      << " stage_attempts=" << stage_attempts
                      << " stage_valid_vertices="
                      << stage_valid_vertices
                      << " total_valid_vertices=" << vertices.size()
                      << " success_rate=" << std::fixed
                      << std::setprecision(6) << success_rate;
            if (std::string(stage) == "incremental") {
                std::cout
                    << " completed_composites="
                    << incremental_completed_composites
                    << " restarts=" << incremental_restarts
                    << " robot_config_attempts="
                    << incremental_robot_config_attempts;
            }
            std::cout << "\n" << std::flush;
        };

    while (!quotas_met()) {
        if (batches >= options.sample_max_batches) {
            throw std::runtime_error(
                "sample mode exhausted --sample-max-batches before "
                "meeting the timestep distribution");
        }
        ++batches;
        const std::size_t first_new = vertices.size();
        std::size_t batch_attempts = 0;
        const auto print_later_batch_progress = [&]() {
            if (batches <= 1 ||
                options.sample_progress_interval == 0 ||
                batch_attempts == 0 ||
                batch_attempts %
                        options.sample_progress_interval !=
                    0) {
                return;
            }
            const std::size_t accepted_vertices =
                vertices.size() - first_new;
            const double success_rate =
                static_cast<double>(accepted_vertices) /
                static_cast<double>(batch_attempts);
            std::cout
                << "sample progress batch=" << batches
                << " stage=deficit_guided_mixed"
                << " batch_attempts=" << batch_attempts
                << " accepted_vertices=" << accepted_vertices
                << " target_vertices="
                << options.sample_vertex_batch_size
                << " success_rate=" << std::fixed
                << std::setprecision(6) << success_rate << "\n"
                << std::flush;
        };
        std::vector<double> deficit_weights(quotas.size(), 0.0);
        for (std::size_t bin = 0; bin < quotas.size(); ++bin) {
            if (candidates[bin].size() < quotas[bin]) {
                deficit_weights[bin] = static_cast<double>(
                    quotas[bin] - candidates[bin].size());
            }
        }
        std::discrete_distribution<std::size_t> deficit_bin(
            deficit_weights.begin(), deficit_weights.end());
        while (vertices.size() <
               first_new + options.sample_vertex_batch_size) {
            if (batch_attempts >=
                options.sample_max_state_attempts_per_batch) {
                throw std::runtime_error(
                    "sample mode could not find enough valid endpoint "
                    "configurations in one batch");
            }
            ++batch_attempts;
            ++state_attempts;
            const bool first_batch_uniform_proposal =
                batches == 1 &&
                batch_attempts <=
                    options.sample_uniform_first_batch_attempt_cap;
            const bool incremental_proposal =
                batches == 1 &&
                batch_attempts >
                    options.sample_uniform_first_batch_attempt_cap &&
                batch_attempts <=
                    options.sample_uniform_first_batch_attempt_cap +
                        options.sample_incremental_first_batch_attempt_cap;
            const bool anchored_robot_proposal =
                batches == 1 &&
                batch_attempts >
                    options.sample_uniform_first_batch_attempt_cap +
                        options.sample_incremental_first_batch_attempt_cap;
            const bool local_proposal =
                batches > 1 && !vertices.empty() &&
                choose_local(proposal_rng);
            if (batches == 1 && batch_attempts == 1 &&
                options.sample_progress_interval > 0) {
                std::cout
                    << "sample first-batch stages uniform_cap="
                    << options.sample_uniform_first_batch_attempt_cap
                    << " incremental_cap="
                    << options.sample_incremental_first_batch_attempt_cap
                    << " anchored_after="
                    << options.sample_uniform_first_batch_attempt_cap +
                           options.sample_incremental_first_batch_attempt_cap
                    << "\n"
                    << std::flush;
            }
            if (incremental_proposal) {
                if (incremental_composite_attempts == 0 &&
                    options.sample_progress_interval > 0) {
                    std::cout
                        << "sample stage transition stage=incremental "
                           "retry_limits=";
                    for (std::size_t robot_index = 0;
                         robot_index <
                         incremental_robot_retry_limits.size();
                         ++robot_index) {
                        if (robot_index > 0)
                            std::cout << ",";
                        std::cout
                            << incremental_robot_retry_limits[robot_index];
                    }
                    std::cout << "\n" << std::flush;
                }
                ++incremental_composite_attempts;
                std::vector<std::vector<double>> configs;
                configs.reserve(robots.size());
                bool completed = true;
                for (std::size_t robot_index = 0;
                     robot_index < robots.size(); ++robot_index) {
                    bool placed = false;
                    for (std::size_t retry = 0;
                         retry <
                         incremental_robot_retry_limits[robot_index];
                         ++retry) {
                        ++incremental_robot_config_attempts;
                        ++incremental_robot_attempts[robot_index];
                        sampler->sampleUniform(sampled_state.get());
                        const auto *real_state =
                            sampled_state.get()
                                ->as<ompl::base::RealVectorStateSpace::
                                         StateType>();
                        std::vector<double> config(
                            robot_config_dimensions[robot_index]);
                        for (std::size_t dimension = 0;
                             dimension < config.size(); ++dimension) {
                            config[dimension] =
                                real_state->values[
                                    robot_config_offsets[robot_index] +
                                    dimension];
                        }
                        auto &checker = problem->collisionChecker();
                        if (!checker.isValidSingleFull(
                                *robots[robot_index], config)) {
                            ++incremental_robot_single_rejections[
                                robot_index];
                            continue;
                        }
                        bool pair_valid = true;
                        for (std::size_t prior = 0;
                             prior < configs.size(); ++prior) {
                            if (!checker.isValidPair(
                                    *robots[robot_index], config,
                                    *robots[prior], configs[prior])) {
                                pair_valid = false;
                                break;
                            }
                        }
                        if (!pair_valid) {
                            ++incremental_robot_pair_rejections[
                                robot_index];
                            continue;
                        }
                        configs.push_back(std::move(config));
                        ++incremental_robot_placements[robot_index];
                        placed = true;
                        break;
                    }
                    if (!placed) {
                        ++incremental_robot_retry_exhaustions[
                            robot_index];
                        completed = false;
                        break;
                    }
                }
                bool accepted = false;
                if (completed) {
                    ++incremental_completed_composites;
                    accepted =
                        problem->collisionChecker().isValidComposite(
                            robots, configs);
                    if (!accepted)
                        ++incremental_final_validation_failures;
                }
                if (accepted) {
                    vertices.push_back({std::move(configs)});
                    ++incremental_valid_vertices;
                } else {
                    ++incremental_restarts;
                }
                print_first_batch_progress(
                    "incremental", incremental_composite_attempts,
                    incremental_valid_vertices);
                continue;
            }
            if (anchored_robot_proposal &&
                anchored_robot_state_attempts == 0) {
                print_incremental_summary();
                if (options.sample_progress_interval > 0) {
                    std::cout
                        << "sample stage transition stage=anchored_fallback"
                        << "\n"
                        << std::flush;
                }
            }
            if (!anchored_robot_proposal && !local_proposal) {
                ++uniform_state_attempts;
                if (first_batch_uniform_proposal)
                    ++first_batch_uniform_state_attempts;
                sampler->sampleUniform(sampled_state.get());
            } else if (anchored_robot_proposal) {
                ++anchored_robot_state_attempts;
                sampler->sampleUniform(sampled_state.get());
                const auto &anchor =
                    vertices.empty()
                        ? valid_seed_anchors[
                              std::uniform_int_distribution<std::size_t>(
                                  0, valid_seed_anchors.size() - 1)(
                                  proposal_rng)]
                        : vertices[
                              std::uniform_int_distribution<std::size_t>(
                                  0, vertices.size() - 1)(proposal_rng)]
                              .configs;
                const std::size_t updated_robot =
                    std::uniform_int_distribution<std::size_t>(
                        0, static_cast<std::size_t>(
                               problem->numRobots() - 1))(proposal_rng);
                auto *real_state = sampled_state.get()
                                       ->as<ompl::base::RealVectorStateSpace::
                                                StateType>();
                std::size_t offset = 0;
                for (int robot_index = 0;
                     robot_index < problem->numRobots();
                     ++robot_index) {
                    const std::size_t dimensions =
                        static_cast<std::size_t>(
                            problem->robot(robot_index)
                                .model->numJoints());
                    if (static_cast<std::size_t>(robot_index) !=
                        updated_robot) {
                        for (std::size_t dimension = 0;
                             dimension < dimensions; ++dimension) {
                            real_state->values[offset + dimension] =
                                anchor[static_cast<std::size_t>(
                                    robot_index)][dimension];
                        }
                    }
                    offset += dimensions;
                }
            } else {
                ++local_state_attempts;
                const std::size_t anchor_index =
                    std::uniform_int_distribution<std::size_t>(
                        0, vertices.size() - 1)(proposal_rng);
                std::size_t guide_index =
                    std::uniform_int_distribution<std::size_t>(
                        0, vertices.size() - 2)(proposal_rng);
                if (guide_index >= anchor_index)
                    ++guide_index;
                const std::size_t target_bin = deficit_bin(proposal_rng);
                const std::size_t target_timesteps =
                    std::uniform_int_distribution<std::size_t>(
                        boundaries[target_bin],
                        boundaries[target_bin + 1] - 1)(proposal_rng);
                const double target_distance =
                    (static_cast<double>(target_timesteps) - 1.5) *
                    vmax / resolution;
                const double guide_distance =
                    maxRobotConfigDistance(
                        vertices[anchor_index].configs,
                        vertices[guide_index].configs);
                if (guide_distance < target_distance ||
                    guide_distance <= 1e-12) {
                    print_later_batch_progress();
                    continue;
                }
                const double alpha =
                    target_distance / guide_distance;
                auto *real_state = sampled_state.get()
                                       ->as<ompl::base::RealVectorStateSpace::
                                                StateType>();
                std::size_t offset = 0;
                for (int robot_index = 0;
                     robot_index < problem->numRobots();
                     ++robot_index) {
                    const auto &robot = problem->robot(robot_index);
                    const std::size_t dimensions =
                        static_cast<std::size_t>(
                            robot.model->numJoints());
                    for (std::size_t dimension = 0;
                         dimension < dimensions; ++dimension) {
                        const double from =
                            vertices[anchor_index]
                                .configs[static_cast<std::size_t>(
                                    robot_index)][dimension];
                        const double to =
                            vertices[guide_index]
                                .configs[static_cast<std::size_t>(
                                    robot_index)][dimension];
                        real_state->values[offset + dimension] =
                            from + alpha * (to - from);
                    }
                    offset += dimensions;
                }
            }
            if (!space_information->isValid(sampled_state.get())) {
                if (first_batch_uniform_proposal) {
                    print_first_batch_progress(
                        "uniform", first_batch_uniform_state_attempts,
                        first_batch_uniform_valid_vertices);
                } else if (anchored_robot_proposal) {
                    print_first_batch_progress(
                        "anchored_fallback",
                        anchored_robot_state_attempts,
                        anchored_robot_valid_vertices);
                }
                print_later_batch_progress();
                continue;
            }
            vertices.push_back(
                {compositeConfigsFromState(sampled_state.get(), *problem)});
            if (anchored_robot_proposal) {
                ++anchored_robot_valid_vertices;
                print_first_batch_progress(
                    "anchored_fallback",
                    anchored_robot_state_attempts,
                    anchored_robot_valid_vertices);
            } else if (local_proposal) {
                ++local_valid_vertices;
            } else {
                ++uniform_valid_vertices;
                if (first_batch_uniform_proposal) {
                    ++first_batch_uniform_valid_vertices;
                    print_first_batch_progress(
                        "uniform", first_batch_uniform_state_attempts,
                        first_batch_uniform_valid_vertices);
                }
            }
            print_later_batch_progress();
        }
        if (batches == 1 &&
            options.sample_progress_interval > 0) {
            print_incremental_summary();
            std::cout
                << "sample first-batch summary valid_vertices="
                << vertices.size() - first_new
                << " uniform_attempts="
                << first_batch_uniform_state_attempts
                << " uniform_valid="
                << first_batch_uniform_valid_vertices
                << " incremental_attempts="
                << incremental_composite_attempts
                << " incremental_valid="
                << incremental_valid_vertices
                << " anchored_attempts="
                << anchored_robot_state_attempts
                << " anchored_valid="
                << anchored_robot_valid_vertices << "\n"
                << std::flush;
        }

        for (std::size_t index = first_new; index < vertices.size();
             ++index) {
            neighbors.add(index);
        }
        for (std::size_t index = first_new; index < vertices.size();
             ++index) {
            std::vector<std::size_t> nearby;
            neighbors.nearestR(index, radius, nearby);
            for (const std::size_t other : nearby) {
                if (other == index)
                    continue;
                const std::size_t first = std::min(index, other);
                const std::size_t second = std::max(index, other);
                const std::uint64_t key =
                    (static_cast<std::uint64_t>(first) << 32u) |
                    static_cast<std::uint64_t>(second);
                if (!seen_pairs.insert(key).second)
                    continue;
                ++neighbor_pairs_considered;
                const double distance = maxRobotConfigDistance(
                    vertices[first].configs, vertices[second].configs);
                const std::size_t timestep_count =
                    static_cast<std::size_t>(
                        std::ceil(distance * resolution / vmax)) +
                    1;
                if (timestep_count < options.sample_min_timesteps ||
                    timestep_count > options.sample_max_timesteps) {
                    continue;
                }
                const auto upper = std::upper_bound(
                    boundaries.begin(), boundaries.end(), timestep_count);
                if (upper == boundaries.begin() ||
                    upper == boundaries.end()) {
                    throw std::runtime_error(
                        "sample timestep did not map to a distribution bin");
                }
                const std::size_t bin = static_cast<std::size_t>(
                    std::distance(boundaries.begin(), upper) - 1);
                candidates[bin].push_back(
                    {first, second, timestep_count, distance});
            }
        }

        if (options.verbose) {
            std::cout << "sample batch=" << batches
                      << " valid_vertices=" << vertices.size()
                      << " state_attempts=" << state_attempts
                      << " candidates=";
            for (std::size_t i = 0; i < candidates.size(); ++i) {
                if (i > 0)
                    std::cout << ",";
                std::cout << candidates[i].size() << "/" << quotas[i];
            }
            std::cout << "\n" << std::flush;
        }
    }

    std::mt19937 selection_rng(seed ^ 0x9e3779b9u);
    std::vector<SampledMotionCandidate> selected;
    selected.reserve(options.sample_motion_count);
    for (std::size_t bin = 0; bin < candidates.size(); ++bin) {
        std::shuffle(candidates[bin].begin(), candidates[bin].end(),
                     selection_rng);
        selected.insert(selected.end(), candidates[bin].begin(),
                        candidates[bin].begin() +
                            static_cast<std::ptrdiff_t>(quotas[bin]));
    }
    std::shuffle(selected.begin(), selected.end(), selection_rng);

    json records = json::array();
    std::uint64_t endpoint_revalidation_failures = 0;
    std::vector<std::optional<bool>> endpoint_revalidation(
        vertices.size(), std::nullopt);
    const auto endpoint_is_valid = [&](std::size_t index) {
        auto &cached = endpoint_revalidation[index];
        if (!cached) {
            cached = problem->collisionChecker().isValidComposite(
                robots, vertices[index].configs);
            if (!*cached)
                ++endpoint_revalidation_failures;
        }
        return *cached;
    };
    for (const auto &candidate : selected) {
        const auto &from = vertices[candidate.first].configs;
        const auto &to = vertices[candidate.second].configs;
        if (!endpoint_is_valid(candidate.first) ||
            !endpoint_is_valid(candidate.second)) {
            continue;
        }
        comotion::CompositePathValidationOptions validation_options;
        validation_options.check_environment = true;
        validation_options.discrete_num_checks_hint =
            static_cast<int>(candidate.timestep_count - 1);
        comotion::ValidationTraceRecord record;
        record.type =
            comotion::ValidationTraceCallType::CompositeMotion;
        record.from = from;
        record.to = to;
        record.options = validation_options;
        record.result = false;
        record.elapsed_nanoseconds = 0;
        auto record_json = recordToJson(seed, record);
        record_json["result_labeled"] = false;
        records.push_back(std::move(record_json));
    }
    if (endpoint_revalidation_failures != 0 ||
        records.size() != options.sample_motion_count) {
        throw std::runtime_error(
            "sample mode endpoint revalidation failed");
    }

    json bins = json::array();
    for (std::size_t bin = 0; bin < quotas.size(); ++bin) {
        bins.push_back({
            {"min_timesteps", boundaries[bin]},
            {"max_timesteps", boundaries[bin + 1] - 1},
            {"target_count", quotas[bin]},
            {"available_candidate_count", candidates[bin].size()},
            {"selected_count", quotas[bin]},
        });
    }
    json incremental_robot_stats = json::array();
    for (std::size_t robot_index = 0; robot_index < robots.size();
         ++robot_index) {
        incremental_robot_stats.push_back({
            {"robot_index", robot_index},
            {"retry_limit",
             incremental_robot_retry_limits[robot_index]},
            {"config_attempts",
             incremental_robot_attempts[robot_index]},
            {"placements",
             incremental_robot_placements[robot_index]},
            {"single_rejections",
             incremental_robot_single_rejections[robot_index]},
            {"pair_rejections",
             incremental_robot_pair_rejections[robot_index]},
            {"retry_exhaustions",
             incremental_robot_retry_exhaustions[robot_index]},
        });
    }

    json doc;
    doc["schema"] = "comotion.validation_trace.v1";
    doc["source"] =
        "sampled_valid_endpoint_radius_neighborhood_panda_cage";
    doc["scenario"] = {
        {"suite", "panda_cage"},
        {"num_robots", scenario.num_robots},
        {"task_index", scenario.task_index},
        {"task_source", scenario.task_source},
    };
    doc["trace_backend"] = common::backendName(options.trace_backend);
    doc["robot_urdf_resource"] =
        options.trace_backend == comotion::CollisionChecker::Backend::Fcl
            ? options.fcl_urdf_rel
            : scenario.urdf_rel;
    doc["robot_collision_geometry"] =
        options.trace_backend == comotion::CollisionChecker::Backend::Fcl
            ? "mesh"
            : "sphere";
    doc["record_results_labeled"] = false;
    doc["result_reference"] = "replayed_fcl";
    doc["empty_environment"] = options.empty_environment;
    doc["planner"] = "";
    doc["iterations"] = 0;
    doc["iteration_mode"] = "sampled_valid_endpoint_neighborhood";
    doc["time_limit_seconds"] = 0.0;
    doc["resolution"] = options.resolution;
    doc["seeds"] = options.seeds;
    doc["seed_summaries"] = json::array();
    doc["sampling"] = {
        {"motion_count", options.sample_motion_count},
        {"vertex_batch_size", options.sample_vertex_batch_size},
        {"valid_vertex_count", vertices.size()},
        {"batch_count", batches},
        {"state_sample_attempts", state_attempts},
        {"uniform_state_attempts", uniform_state_attempts},
        {"first_batch_uniform_state_attempts",
         first_batch_uniform_state_attempts},
        {"first_batch_uniform_valid_vertex_count",
         first_batch_uniform_valid_vertices},
        {"incremental_composite_attempts",
         incremental_composite_attempts},
        {"incremental_completed_composites",
         incremental_completed_composites},
        {"incremental_restarts", incremental_restarts},
        {"incremental_valid_vertex_count",
         incremental_valid_vertices},
        {"incremental_final_validation_failures",
         incremental_final_validation_failures},
        {"incremental_robot_config_attempts",
         incremental_robot_config_attempts},
        {"incremental_robot_stats", incremental_robot_stats},
        {"anchored_robot_state_attempts",
         anchored_robot_state_attempts},
        {"local_state_attempts", local_state_attempts},
        {"uniform_valid_vertex_count", uniform_valid_vertices},
        {"anchored_robot_valid_vertex_count",
         anchored_robot_valid_vertices},
        {"local_valid_vertex_count", local_valid_vertices},
        {"neighbor_pairs_considered", neighbor_pairs_considered},
        {"candidate_pairs_in_range",
         std::accumulate(
             candidates.begin(), candidates.end(), std::size_t{0},
             [](std::size_t total, const auto &bin) {
                 return total + bin.size();
             })},
        {"endpoint_revalidation_failures",
         endpoint_revalidation_failures},
        {"endpoint_revalidation_unique_vertices",
         std::count_if(
             endpoint_revalidation.begin(),
             endpoint_revalidation.end(),
             [](const auto &result) { return result.has_value(); })},
        {"motion_results_labeled", false},
        {"distance_metric", "max_per_robot_euclidean"},
        {"neighborhood_index", "GNAT_radius"},
        {"vertex_sampling",
         "first batch uniform up to the configured cap, then incremental "
         "robot-by-robot composite construction up to its configured cap, "
         "then valid-anchor single-robot updates if needed; later batches "
         "use 90% deficit-guided interpolation between existing valid "
         "vertices and 10% uniform proposals"},
        {"uniform_first_batch_attempt_cap",
         options.sample_uniform_first_batch_attempt_cap},
        {"incremental_first_batch_attempt_cap",
         options.sample_incremental_first_batch_attempt_cap},
        {"sampling_progress_interval",
         options.sample_progress_interval},
        {"incremental_retry_formula",
         "ceil(10^(zero_based_robot_index/4.0))"},
        {"sampling_validation_backend",
         common::backendName(options.trace_backend)},
        {"radius_distance", radius},
        {"distance_to_timestep",
         "ceil(distance * resolution / vmax) + 1"},
        {"minimum_timesteps", options.sample_min_timesteps},
        {"maximum_timesteps", options.sample_max_timesteps},
        {"target_distribution",
         "asymmetric bins concentrated at 450-550 with a longer "
         "lower tail to 100"},
        {"bins", bins},
    };
    doc["records"] = records;
    writeJson(doc, options.trace_output);
    return doc;
}

std::size_t timestepCount(const comotion::ValidationTraceRecord &record) {
    if (record.type == comotion::ValidationTraceCallType::CompositeState)
        return 1;
    const int checks = record.options.discrete_num_checks_hint > 0
                           ? record.options.discrete_num_checks_hint
                           : 10;
    return static_cast<std::size_t>(checks) + 1;
}

double pathLength(const comotion::ValidationTraceRecord &record) {
    if (record.type == comotion::ValidationTraceCallType::CompositeState)
        return 0.0;
    return maxRobotConfigDistance(record.from, record.to);
}

std::string scopeName(comotion::ConflictScope scope) {
    switch (scope) {
    case comotion::ConflictScope::Environment:
        return "obstacle";
    case comotion::ConflictScope::Self:
        return "self";
    case comotion::ConflictScope::InterRobot:
        return "inter_robot";
    }
    return "unknown";
}

std::optional<std::string>
classifyCompositeState(const comotion::CollisionChecker &checker,
                       const std::vector<const comotion::RobotModel *> &robots,
                       const std::vector<std::vector<double>> &configs) {
    for (std::size_t i = 0; i < robots.size(); ++i) {
        if (!checker.isValidSingle(*robots[i], configs[i]))
            return "obstacle";
    }
    for (std::size_t i = 0; i < robots.size(); ++i) {
        if (!checker.isSelfCollisionFree(*robots[i], configs[i]))
            return "self";
    }
    for (std::size_t i = 0; i < robots.size(); ++i) {
        for (std::size_t j = i + 1; j < robots.size(); ++j) {
            if (!checker.isValidPair(*robots[i], configs[i],
                                     *robots[j], configs[j])) {
                return "inter_robot";
            }
        }
    }
    return std::nullopt;
}

std::optional<std::string>
classifyRecord(const comotion::CollisionChecker &checker,
               const std::vector<const comotion::RobotModel *> &robots,
               const comotion::ValidationTraceRecord &record) {
    if (record.type == comotion::ValidationTraceCallType::CompositeState) {
        return classifyCompositeState(checker, robots, record.configs);
    }
    auto conflict = checker.findFirstCompositeMotionConflict(
        robots, record.from, record.to, record.options);
    if (!conflict)
        return std::nullopt;
    return scopeName(conflict->scope);
}

DistributionSummary summarizeDistribution(std::vector<double> values) {
    DistributionSummary out;
    out.count = values.size();
    if (values.empty())
        return out;
    std::sort(values.begin(), values.end());
    const auto percentile = [&](double p) {
        const double pos = p * static_cast<double>(values.size() - 1);
        const auto lo = static_cast<std::size_t>(std::floor(pos));
        const auto hi = static_cast<std::size_t>(std::ceil(pos));
        if (lo == hi)
            return values[lo];
        const double alpha = pos - static_cast<double>(lo);
        return values[lo] * (1.0 - alpha) + values[hi] * alpha;
    };
    out.min = values.front();
    out.p50 = percentile(0.50);
    out.p90 = percentile(0.90);
    out.p99 = percentile(0.99);
    out.max = values.back();
    out.mean = std::accumulate(values.begin(), values.end(), 0.0) /
               static_cast<double>(values.size());
    return out;
}

json distributionToJson(const DistributionSummary &dist,
                        const std::vector<double> &values) {
    return {
        {"count", dist.count},
        {"min", dist.min},
        {"p50", dist.p50},
        {"p90", dist.p90},
        {"p99", dist.p99},
        {"max", dist.max},
        {"mean", dist.mean},
        {"values", values},
    };
}

void accumulateStats(VariantStats &variant,
                     const comotion::ValidationTraceRecord &record,
                     bool observed_result, bool reference_result,
                     std::uint64_t elapsed_ns,
                     std::optional<std::string> scope,
                     const comotion::ValidationWorkStats &work) {
    const std::string type_name =
        comotion::validationTraceCallTypeName(record.type);
    auto &type = variant.by_type[type_name];

    ++variant.count;
    ++type.count;
    variant.elapsed_ns += elapsed_ns;
    type.elapsed_ns += elapsed_ns;
    type.latencies_seconds.push_back(static_cast<double>(elapsed_ns) * 1e-9);
    type.path_lengths.push_back(pathLength(record));
    type.timestep_counts.push_back(static_cast<double>(timestepCount(record)));
    variant.work += work;
    type.work += work;

    if (observed_result) {
        ++variant.valid;
        ++type.valid;
        variant.valid_elapsed_ns += elapsed_ns;
        type.valid_elapsed_ns += elapsed_ns;
    } else {
        ++variant.invalid;
        ++type.invalid;
        variant.invalid_elapsed_ns += elapsed_ns;
        type.invalid_elapsed_ns += elapsed_ns;
        if (scope) {
            ++type.scope_counts[*scope];
        } else {
            ++type.scope_counts["unknown"];
        }
    }

    if (observed_result != reference_result) {
        ++variant.result_mismatches;
        ++type.result_mismatches;
    }
}

json validationWorkStatsToJson(const comotion::ValidationWorkStats &stats) {
    const auto ratio = [](std::uint64_t checked, std::uint64_t possible) {
        return possible == 0
                   ? 0.0
                   : static_cast<double>(checked) /
                         static_cast<double>(possible);
    };
    return {
        {"motion_timesteps_possible", stats.motion_timesteps_possible},
        {"motion_timesteps_checked", stats.motion_timesteps_checked},
        {"motion_timestep_coverage",
         ratio(stats.motion_timesteps_checked,
               stats.motion_timesteps_possible)},
        {"robot_state_checks_possible", stats.robot_state_checks_possible},
        {"robot_state_checks_completed", stats.robot_state_checks_completed},
        {"robot_pair_checks_possible", stats.robot_pair_checks_possible},
        {"robot_pair_checks_completed", stats.robot_pair_checks_completed},
        {"simd_packs_checked", stats.simd_packs_checked},
        {"simd_lanes_checked", stats.simd_lanes_checked},
    };
}

json typeStatsToJson(const TypeStats &stats) {
    const double total_seconds = static_cast<double>(stats.elapsed_ns) * 1e-9;
    return {
        {"count", stats.count},
        {"valid", stats.valid},
        {"invalid", stats.invalid},
        {"valid_ratio", stats.count == 0 ? 0.0
                                          : static_cast<double>(stats.valid) /
                                                static_cast<double>(stats.count)},
        {"total_validation_time_seconds", total_seconds},
        {"valid_validation_time_seconds",
         static_cast<double>(stats.valid_elapsed_ns) * 1e-9},
        {"invalid_validation_time_seconds",
         static_cast<double>(stats.invalid_elapsed_ns) * 1e-9},
        {"result_mismatches_vs_trace", stats.result_mismatches},
        {"collision_scope_counts", stats.scope_counts},
        {"latency_seconds",
         distributionToJson(summarizeDistribution(stats.latencies_seconds),
                            stats.latencies_seconds)},
        {"path_length",
         distributionToJson(summarizeDistribution(stats.path_lengths),
                            stats.path_lengths)},
        {"timestep_count",
         distributionToJson(summarizeDistribution(stats.timestep_counts),
                            stats.timestep_counts)},
        {"validation_work", validationWorkStatsToJson(stats.work)},
    };
}

json variantStatsToJson(const VariantStats &stats) {
    json by_type = json::object();
    std::vector<double> all_latencies;
    std::vector<double> all_lengths;
    std::vector<double> all_timesteps;
    std::map<std::string, std::uint64_t> all_scopes;
    for (const auto &[name, type] : stats.by_type) {
        by_type[name] = typeStatsToJson(type);
        all_latencies.insert(all_latencies.end(), type.latencies_seconds.begin(),
                             type.latencies_seconds.end());
        all_lengths.insert(all_lengths.end(), type.path_lengths.begin(),
                           type.path_lengths.end());
        all_timesteps.insert(all_timesteps.end(), type.timestep_counts.begin(),
                             type.timestep_counts.end());
        for (const auto &[scope, count] : type.scope_counts)
            all_scopes[scope] += count;
    }

    return {
        {"name", stats.name},
        {"backend", stats.backend},
        {"count", stats.count},
        {"valid", stats.valid},
        {"invalid", stats.invalid},
        {"valid_ratio", stats.count == 0 ? 0.0
                                          : static_cast<double>(stats.valid) /
                                                static_cast<double>(stats.count)},
        {"total_validation_time_seconds",
         static_cast<double>(stats.elapsed_ns) * 1e-9},
        {"valid_validation_time_seconds",
         static_cast<double>(stats.valid_elapsed_ns) * 1e-9},
        {"invalid_validation_time_seconds",
         static_cast<double>(stats.invalid_elapsed_ns) * 1e-9},
        {"result_mismatches_vs_trace", stats.result_mismatches},
        {"collision_scope_counts", all_scopes},
        {"latency_seconds",
         distributionToJson(summarizeDistribution(all_latencies),
                            all_latencies)},
        {"path_length",
         distributionToJson(summarizeDistribution(all_lengths), all_lengths)},
        {"timestep_count",
         distributionToJson(summarizeDistribution(all_timesteps),
                            all_timesteps)},
        {"by_type", by_type},
        {"validation_work", validationWorkStatsToJson(stats.work)},
    };
}

struct ReplayVariant {
    std::string name;
    comotion::CollisionChecker::Backend backend;
    std::optional<comotion::VampValidationStrategy> vamp_strategy;
};

bool shouldReplayRecord(const comotion::ValidationTraceRecord &record,
                        bool motion_only) {
    return !motion_only ||
           record.type == comotion::ValidationTraceCallType::CompositeMotion;
}

std::vector<std::size_t> replayRecordIndices(const TraceCorpus &corpus,
                                             bool motion_only) {
    std::vector<std::size_t> indices;
    indices.reserve(corpus.records.size());
    for (std::size_t i = 0; i < corpus.records.size(); ++i) {
        if (shouldReplayRecord(corpus.records[i].record, motion_only))
            indices.push_back(i);
    }
    return indices;
}

std::size_t countMotionRecords(const TraceCorpus &corpus,
                               const std::vector<std::size_t> &indices) {
    std::size_t count = 0;
    for (const std::size_t index : indices) {
        if (corpus.records[index].record.type ==
            comotion::ValidationTraceCallType::CompositeMotion) {
            ++count;
        }
    }
    return count;
}

std::vector<ReplayVariant> replayVariants() {
    using comotion::VampBatchOrdering;
    using comotion::VampBatchPacking;
    return {
        {"fcl", comotion::CollisionChecker::Backend::Fcl, std::nullopt},
        {"sphere", comotion::CollisionChecker::Backend::Spheres, std::nullopt},
        {"vamp_combined_rake", comotion::CollisionChecker::Backend::Vamp,
         comotion::VampValidationStrategy{VampBatchOrdering::Combined,
                                           VampBatchPacking::Rake}},
        {"vamp_combined_linear", comotion::CollisionChecker::Backend::Vamp,
         comotion::VampValidationStrategy{VampBatchOrdering::Combined,
                                           VampBatchPacking::Linear}},
        {"vamp_hierarchical_rake", comotion::CollisionChecker::Backend::Vamp,
         comotion::VampValidationStrategy{VampBatchOrdering::Hierarchical,
                                           VampBatchPacking::Rake}},
        {"vamp_hierarchical_linear", comotion::CollisionChecker::Backend::Vamp,
         comotion::VampValidationStrategy{VampBatchOrdering::Hierarchical,
                                           VampBatchPacking::Linear}},
    };
}

std::vector<ReplayVariant>
selectReplayVariants(const std::vector<std::string> &names) {
    auto variants = replayVariants();
    if (names.empty())
        return variants;

    std::vector<ReplayVariant> selected;
    for (const auto &name : names) {
        const auto it = std::find_if(
            variants.begin(), variants.end(),
            [&](const ReplayVariant &variant) { return variant.name == name; });
        if (it == variants.end()) {
            std::ostringstream msg;
            msg << "Unknown replay variant '" << name
                << "'. Available variants:";
            for (const auto &variant : variants)
                msg << " " << variant.name;
            throw std::runtime_error(msg.str());
        }
        selected.push_back(*it);
    }
    return selected;
}

std::vector<std::string>
replayVariantNames(const std::vector<ReplayVariant> &variants) {
    std::vector<std::string> names;
    names.reserve(variants.size());
    for (const auto &variant : variants)
        names.push_back(variant.name);
    return names;
}

json runReplay(const TraceCorpus &corpus, const AppOptions &options,
               const PandaCageScenario &scenario) {
    json variants_json = json::array();
    std::map<std::string, double> total_seconds_by_name;
    const auto variants = selectReplayVariants(options.replay_variants);
    const auto record_indices =
        replayRecordIndices(corpus, options.motion_only);
    const bool trace_results_labeled =
        corpus.metadata.value("record_results_labeled", true);
    if (!trace_results_labeled &&
        (variants.empty() || variants.front().name != "fcl")) {
        throw std::runtime_error(
            "unlabeled sampled traces require fcl as the first replay "
            "variant");
    }
    std::vector<bool> replay_fcl_results(record_indices.size(), false);
    bool replay_fcl_results_ready = trace_results_labeled;
    const std::size_t motion_record_count =
        countMotionRecords(corpus, record_indices);
    if (record_indices.empty()) {
        throw std::runtime_error(
            options.motion_only
                ? "No composite motion records found in trace"
                : "No validation records found in trace");
    }

    std::cout << "replay_records: " << record_indices.size()
              << " of " << corpus.records.size()
              << " trace records";
    if (options.motion_only)
        std::cout << " (motion-only)";
    std::cout << ", motions=" << motion_record_count << "\n" << std::flush;

    std::vector<std::optional<std::string>> scopes(record_indices.size(),
                                                   std::nullopt);
    const bool empty_environment =
        corpus.metadata.value("empty_environment", options.empty_environment);
    if (options.classify_scopes) {
        auto scope_problem = makeProblem(
            scenario, comotion::CollisionChecker::Backend::Spheres,
            corpus.metadata.value("resolution", options.resolution),
            empty_environment, options.fcl_urdf_rel);
        const auto scope_robots = scope_problem->robotModelPtrs();
        auto &scope_checker = scope_problem->collisionChecker();
        const auto scope_wall_start = Clock::now();
        std::size_t motions_done = 0;
        std::cout << "[replay] start scope_classification records="
                  << record_indices.size() << " motions="
                  << motion_record_count << "\n"
                  << std::flush;
        for (std::size_t local_index = 0; local_index < record_indices.size();
             ++local_index) {
            const auto &entry = corpus.records[record_indices[local_index]];
            auto record = entry.record;
            if (options.exhaustive_motion_validation &&
                record.type ==
                    comotion::ValidationTraceCallType::CompositeMotion) {
                record.options.exhaustive = true;
            }
            bool result = true;
            if (record.type ==
                comotion::ValidationTraceCallType::CompositeState) {
                result = scope_checker.isValidComposite(scope_robots,
                                                        record.configs);
            } else {
                result = scope_checker.isCompositeMotionValid(
                    scope_robots, record.from, record.to, record.options);
            }
            scopes[local_index] =
                result ? std::nullopt
                       : classifyRecord(scope_checker, scope_robots, record);
            if (record.type ==
                comotion::ValidationTraceCallType::CompositeMotion) {
                ++motions_done;
                if (options.progress_interval > 0 &&
                    (motions_done % options.progress_interval == 0 ||
                     motions_done == motion_record_count)) {
                    const auto scope_wall_ns =
                        elapsedNanoseconds(scope_wall_start);
                    std::cout << "[replay] scope_classification motions="
                              << motions_done << "/" << motion_record_count
                              << " elapsed_s=" << std::fixed
                              << std::setprecision(3)
                              << static_cast<double>(scope_wall_ns) * 1e-9
                              << "\n"
                              << std::flush;
                }
            }
        }
        const auto scope_wall_ns = elapsedNanoseconds(scope_wall_start);
        std::cout << "[replay] done scope_classification motions="
                  << motions_done << "/" << motion_record_count
                  << " elapsed_s=" << std::fixed << std::setprecision(3)
                  << static_cast<double>(scope_wall_ns) * 1e-9 << "\n"
                  << std::flush;
    }

    for (const auto &variant : variants) {
        auto problem = makeProblem(
            scenario, variant.backend,
            corpus.metadata.value("resolution", options.resolution),
            empty_environment, options.fcl_urdf_rel);
        if (variant.vamp_strategy) {
            problem->collisionChecker().setVampValidationStrategy(
                *variant.vamp_strategy);
        }
        const auto robots = problem->robotModelPtrs();
        auto &checker = problem->collisionChecker();

        VariantStats stats;
        stats.name = variant.name;
        stats.backend = common::backendName(variant.backend);

        const auto variant_wall_start = Clock::now();
        std::size_t motions_done = 0;
        std::cout << "[replay] start " << variant.name
                  << " records=" << record_indices.size()
                  << " motions=" << motion_record_count << "\n"
                  << std::flush;

        for (std::size_t local_index = 0; local_index < record_indices.size();
             ++local_index) {
            const auto &entry = corpus.records[record_indices[local_index]];
            auto record = entry.record;
            if (options.exhaustive_motion_validation &&
                record.type ==
                    comotion::ValidationTraceCallType::CompositeMotion) {
                record.options.exhaustive = true;
            }
            const auto start = startValidationTimer(options.timing_clock);
            bool result = false;
            comotion::ValidationWorkStats work;
            if (record.type ==
                comotion::ValidationTraceCallType::CompositeState) {
                result = checker.isValidComposite(robots, record.configs);
            } else {
                result = checker.isCompositeMotionValid(
                    robots, record.from, record.to, record.options);
                work = checker.lastValidationWorkStats();
            }
            const auto elapsed_ns = elapsedValidationNanoseconds(start);

            bool reference_result = record.result;
            if (!trace_results_labeled) {
                if (variant.name == "fcl") {
                    replay_fcl_results[local_index] = result;
                    reference_result = result;
                } else {
                    if (!replay_fcl_results_ready) {
                        throw std::runtime_error(
                            "FCL replay reference results are unavailable");
                    }
                    reference_result =
                        replay_fcl_results[local_index];
                }
            }
            accumulateStats(stats, record, result, reference_result,
                            elapsed_ns,
                            result ? std::nullopt : scopes[local_index], work);
            if (record.type ==
                comotion::ValidationTraceCallType::CompositeMotion) {
                ++motions_done;
                if (options.progress_interval > 0 &&
                    (motions_done % options.progress_interval == 0 ||
                     motions_done == motion_record_count)) {
                    const auto variant_wall_ns =
                        elapsedNanoseconds(variant_wall_start);
                    std::cout << "[replay] " << variant.name
                              << " motions=" << motions_done << "/"
                              << motion_record_count
                              << " elapsed_s=" << std::fixed
                              << std::setprecision(3)
                              << static_cast<double>(variant_wall_ns) * 1e-9
                              << " valid=" << stats.valid
                              << " invalid=" << stats.invalid << "\n"
                              << std::flush;
                }
            }
        }

        const double total_seconds =
            static_cast<double>(stats.elapsed_ns) * 1e-9;
        if (!trace_results_labeled && variant.name == "fcl")
            replay_fcl_results_ready = true;
        total_seconds_by_name[variant.name] = total_seconds;
        auto variant_json = variantStatsToJson(stats);
        const std::string &robot_urdf_resource =
            variant.backend == comotion::CollisionChecker::Backend::Fcl
                ? options.fcl_urdf_rel
                : scenario.urdf_rel;
        variant_json["robot_urdf_resource"] = robot_urdf_resource;
        variant_json["robot_urdf_path"] =
            getResourcePath(robot_urdf_resource);
        variant_json["robot_collision_geometry"] =
            variant.backend == comotion::CollisionChecker::Backend::Fcl
                ? "mesh"
                : "sphere";
        variants_json.push_back(std::move(variant_json));
        std::cout << "[replay] done " << variant.name
                  << " motions=" << motions_done << "/"
                  << motion_record_count
                  << " validation_s=" << std::fixed << std::setprecision(3)
                  << total_seconds << "\n"
                  << std::flush;
    }

    json speedups = json::array();
    const double fcl_seconds = total_seconds_by_name["fcl"];
    const double sphere_seconds = total_seconds_by_name["sphere"];
    for (const auto &[name, seconds] : total_seconds_by_name) {
        if (name.rfind("vamp_", 0) != 0)
            continue;
        speedups.push_back({
            {"variant", name},
            {"vs_fcl", seconds > 0.0 ? fcl_seconds / seconds : 0.0},
            {"vs_sphere", seconds > 0.0 ? sphere_seconds / seconds : 0.0},
        });
    }

    json metrics;
    metrics["schema"] = "comotion.validation_timing_metrics.v1";
    metrics["trace"] = {
        {"source", corpus.metadata.value("source", "")},
        {"record_count", record_indices.size()},
        {"source_record_count", corpus.records.size()},
        {"motion_only", options.motion_only},
        {"exhaustive_motion_validation",
         options.exhaustive_motion_validation},
        {"timing_clock", timingClockName(options.timing_clock)},
        {"scope_classification", options.classify_scopes},
        {"variants", replayVariantNames(variants)},
        {"motion_record_count", motion_record_count},
        {"planner", corpus.metadata.value("planner", "")},
        {"iterations", corpus.metadata.value("iterations", 0)},
        {"iteration_mode", corpus.metadata.value("iteration_mode", "")},
        {"time_limit_seconds",
         corpus.metadata.value("time_limit_seconds", 0.0)},
        {"resolution", corpus.metadata.value("resolution", 0)},
        {"trace_backend", corpus.metadata.value("trace_backend", "")},
        {"result_reference",
         trace_results_labeled ? "trace" : "replayed_fcl"},
        {"empty_environment", empty_environment},
        {"seeds", corpus.metadata.value("seeds", json::array())},
        {"seed_summaries",
         corpus.metadata.value("seed_summaries", json::array())},
        {"scenario", corpus.metadata.value("scenario", json::object())},
        {"fcl_urdf_resource", options.fcl_urdf_rel},
        {"sphere_urdf_resource", scenario.urdf_rel},
    };
    metrics["variants"] = variants_json;
    metrics["speedups"] = speedups;
    writeJson(metrics, options.metrics_json);
    return metrics;
}

void printSummary(const json &metrics) {
    std::cout << "\nValidation replay summary\n";
    std::cout << "records: " << metrics.at("trace").at("record_count")
              << "\n\n";
    std::cout << std::left << std::setw(28) << "option"
              << std::right << std::setw(14) << "time(s)"
              << std::setw(12) << "count"
              << std::setw(12) << "valid%"
              << std::setw(12) << "p50(ms)"
              << std::setw(12) << "p90(ms)"
              << std::setw(12) << "p99(ms)" << "\n";
    for (const auto &variant : metrics.at("variants")) {
        const auto &latency = variant.at("latency_seconds");
        const double valid_ratio = variant.value("valid_ratio", 0.0);
        std::cout << std::left << std::setw(28)
                  << variant.at("name").get<std::string>()
                  << std::right << std::setw(14) << std::fixed
                  << std::setprecision(6)
                  << variant.at("total_validation_time_seconds").get<double>()
                  << std::setw(12) << variant.at("count").get<std::uint64_t>()
                  << std::setw(12) << std::setprecision(1)
                  << valid_ratio * 100.0
                  << std::setw(12) << std::setprecision(3)
                  << latency.at("p50").get<double>() * 1000.0
                  << std::setw(12)
                  << latency.at("p90").get<double>() * 1000.0
                  << std::setw(12)
                  << latency.at("p99").get<double>() * 1000.0 << "\n";
    }

    std::cout << "\nVAMP speedups\n";
    for (const auto &speedup : metrics.at("speedups")) {
        std::cout << "  " << speedup.at("variant").get<std::string>()
                  << ": " << std::setprecision(2)
                  << speedup.at("vs_fcl").get<double>() << "x vs FCL, "
                  << speedup.at("vs_sphere").get<double>()
                  << "x vs Sphere\n";
    }
}

void printUsage(const char *argv0) {
    std::cout
        << "Usage: " << argv0
        << " [--mode trace|sample|replay|trace-replay] [options]\n"
        << "  --num-robots <N>       Panda cage robot count (default: 4)\n"
        << "  --task-index <i>       Built-in task index (default: 0)\n"
        << "  --seeds <csv>          Planning seeds (default: 0,1,2)\n"
        << "  --iterations <n>       Composite RRT-C iteration cap (default: 1000)\n"
        << "  --time-limit <sec>     Planner time limit for tracing (default: 1000)\n"
        << "  --rrt-range <d>        Explicit RRTConnect extension range; 0 auto\n"
        << "  --composite-rrt-range <d> Alias for --rrt-range\n"
        << "  --composite-rrt-use-makespan-metric\n"
        << "  --trace-backend <b>    sphere, fcl, or vamp for corpus generation\n"
        << "                         (default: sphere)\n"
        << "  --trace-output <path>  Trace JSON path\n"
        << "  --trace-input <path>   Trace JSON path(s) for replay; comma-separated\n"
        << "  --metrics-json <path>  Replay metrics JSON path\n"
        << "  --variants <csv>      Replay variant names; default runs all\n"
        << "  --fcl-urdf <path>      Resource-relative mesh URDF used only by FCL\n"
        << "                         (default: panda/panda.urdf)\n"
        << "  --resolution <n>       Timesteps per second (default: 128)\n"
        << "  --empty-environment    Clear all scenario obstacles\n"
        << "  --motion-only          Replay only composite motion records\n"
        << "  --exhaustive-motion-validation\n"
        << "                         Check all motion samples before returning\n"
        << "  --sample-motion-count <n>\n"
        << "                         Motions generated by sample mode (default: 1000)\n"
        << "  --sample-vertex-batch-size <n>\n"
        << "                         Valid endpoints per sampling batch (default: 200)\n"
        << "  --sample-min-timesteps <n>\n"
        << "                         Minimum sampled motion timesteps (default: 100)\n"
        << "  --sample-max-timesteps <n>\n"
        << "                         Radius cap in timesteps (default: 700)\n"
        << "  --sample-max-batches <n>\n"
        << "                         Sampling safety cap (default: 1000)\n"
        << "  --sample-max-state-attempts-per-batch <n>\n"
        << "                         Endpoint rejection-sampling safety cap\n"
        << "  --sample-uniform-first-batch-attempt-cap <n>\n"
        << "                         Switch first batch to incremental sampling"
           " after this many uniform attempts (default: 10000)\n"
        << "  --sample-incremental-first-batch-attempt-cap <n>\n"
        << "                         Switch incremental sampling to anchored"
           " fallback after this many attempts (default: 10000)\n"
        << "  --sample-progress-interval <n>\n"
        << "                         Log sampling success rates every n attempts"
           " (default: 1000, 0 disables)\n"
        << "  --timing-clock <wall|cpu>\n"
        << "                         Measure replay validation calls with this clock\n"
        << "  --classify-scopes      Add an untimed sphere pass for collision-scope"
           " counts\n"
        << "  --progress-interval <n> Log every n replayed motions (default: 50,"
           " 0 disables)\n"
        << "  --verbose\n";
}

AppOptions parseArgs(int argc, char **argv) {
    AppOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--mode") {
            options.mode = requireValue(i, argc, argv, arg);
        } else if (arg == "--num-robots") {
            options.num_robots = std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--task-index") {
            options.task_index = std::stoi(requireValue(i, argc, argv, arg));
        } else if (arg == "--seeds") {
            options.seeds = parseSeeds(requireValue(i, argc, argv, arg));
        } else if (arg == "--iterations") {
            options.iterations = static_cast<unsigned>(
                std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--time-limit") {
            options.time_limit = std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--rrt-range" ||
                   arg == "--composite-rrt-range") {
            options.composite_rrt_range =
                std::stod(requireValue(i, argc, argv, arg));
        } else if (arg == "--composite-rrt-use-makespan-metric") {
            options.composite_rrt_use_makespan_metric = true;
        } else if (arg == "--trace-backend") {
            options.trace_backend =
                common::parseCollisionBackend(requireValue(i, argc, argv, arg));
        } else if (arg == "--trace-output") {
            options.trace_output = requireValue(i, argc, argv, arg);
        } else if (arg == "--trace-input") {
            options.trace_input = requireValue(i, argc, argv, arg);
        } else if (arg == "--metrics-json") {
            options.metrics_json = requireValue(i, argc, argv, arg);
        } else if (arg == "--variants") {
            options.replay_variants =
                parseCsvStrings(requireValue(i, argc, argv, arg));
        } else if (arg == "--fcl-urdf") {
            options.fcl_urdf_rel = requireValue(i, argc, argv, arg);
        } else if (arg == "--resolution") {
            options.resolution = static_cast<std::size_t>(
                std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--empty-environment") {
            options.empty_environment = true;
        } else if (arg == "--motion-only") {
            options.motion_only = true;
        } else if (arg == "--exhaustive-motion-validation") {
            options.exhaustive_motion_validation = true;
        } else if (arg == "--sample-motion-count") {
            options.sample_motion_count = static_cast<std::size_t>(
                std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--sample-vertex-batch-size") {
            options.sample_vertex_batch_size = static_cast<std::size_t>(
                std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--sample-min-timesteps") {
            options.sample_min_timesteps = static_cast<std::size_t>(
                std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--sample-max-timesteps") {
            options.sample_max_timesteps = static_cast<std::size_t>(
                std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--sample-max-batches") {
            options.sample_max_batches = static_cast<std::size_t>(
                std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--sample-max-state-attempts-per-batch") {
            options.sample_max_state_attempts_per_batch =
                static_cast<std::size_t>(
                    std::stoull(requireValue(i, argc, argv, arg)));
        } else if (
            arg == "--sample-uniform-first-batch-attempt-cap") {
            options.sample_uniform_first_batch_attempt_cap =
                static_cast<std::size_t>(
                    std::stoull(requireValue(i, argc, argv, arg)));
        } else if (
            arg == "--sample-incremental-first-batch-attempt-cap") {
            options.sample_incremental_first_batch_attempt_cap =
                static_cast<std::size_t>(
                    std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--sample-progress-interval") {
            options.sample_progress_interval =
                static_cast<std::size_t>(
                    std::stoull(requireValue(i, argc, argv, arg)));
        } else if (arg == "--timing-clock") {
            options.timing_clock =
                parseTimingClock(requireValue(i, argc, argv, arg));
        } else if (arg == "--cpu-time") {
            options.timing_clock = TimingClock::ProcessCpu;
        } else if (arg == "--classify-scopes") {
            options.classify_scopes = true;
        } else if (arg == "--progress-interval") {
            options.progress_interval = static_cast<std::size_t>(
                std::stoul(requireValue(i, argc, argv, arg)));
        } else if (arg == "--verbose") {
            options.verbose = true;
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (options.mode != "trace" && options.mode != "sample" &&
        options.mode != "replay" &&
        options.mode != "trace-replay") {
        throw std::runtime_error(
            "--mode must be trace, sample, replay, or trace-replay");
    }
    if (options.iterations == 0)
        throw std::runtime_error("--iterations must be positive");
    if (options.resolution == 0)
        throw std::runtime_error("--resolution must be positive");
    if (options.time_limit <= 0.0)
        throw std::runtime_error("--time-limit must be positive");
    if (options.composite_rrt_range < 0.0)
        throw std::runtime_error("--rrt-range must be non-negative");
    if (options.fcl_urdf_rel.empty())
        throw std::runtime_error("--fcl-urdf must not be empty");
    if (options.sample_motion_count == 0)
        throw std::runtime_error("--sample-motion-count must be positive");
    if (options.sample_vertex_batch_size == 0)
        throw std::runtime_error(
            "--sample-vertex-batch-size must be positive");
    if (options.sample_min_timesteps < 2 ||
        options.sample_min_timesteps >= options.sample_max_timesteps) {
        throw std::runtime_error(
            "sample timestep range must satisfy 2 <= min < max");
    }
    if (options.sample_max_batches == 0)
        throw std::runtime_error("--sample-max-batches must be positive");
    if (options.sample_max_state_attempts_per_batch == 0) {
        throw std::runtime_error(
            "--sample-max-state-attempts-per-batch must be positive");
    }
    if (options.sample_uniform_first_batch_attempt_cap == 0) {
        throw std::runtime_error(
            "--sample-uniform-first-batch-attempt-cap must be positive");
    }
    if (options.sample_incremental_first_batch_attempt_cap == 0) {
        throw std::runtime_error(
            "--sample-incremental-first-batch-attempt-cap must be positive");
    }
    return options;
}

} // namespace

int main(int argc, char **argv) {
    try {
        setExecutablePath(argv[0]);
        const AppOptions options = parseArgs(argc, argv);
        const PandaCageScenario scenario =
            loadPandaCageScenario(options.num_robots, options.task_index);

        json trace_doc;
        if (options.mode == "sample") {
            trace_doc = runSampleTrace(options, scenario);
            std::cout << "trace_json: " << options.trace_output << "\n";
        } else if (options.mode == "trace" ||
                   options.mode == "trace-replay") {
            trace_doc = runTrace(options, scenario);
            std::cout << "trace_json: " << options.trace_output << "\n";
        }

        if (options.mode == "replay" || options.mode == "trace-replay") {
            const std::string trace_path =
                options.mode == "trace-replay" ? options.trace_output
                                                : options.trace_input;
            if (trace_path.empty())
                throw std::runtime_error("--trace-input is required for replay");
            TraceCorpus corpus = loadTraceCorpora(trace_path);
            json metrics = runReplay(corpus, options, scenario);
            std::cout << "metrics_json: " << options.metrics_json << "\n";
            printSummary(metrics);
        }
        return 0;
    } catch (const std::exception &ex) {
        std::cerr << "validation_timing: " << ex.what() << "\n";
        return 1;
    }
}
