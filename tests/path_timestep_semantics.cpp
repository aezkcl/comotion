/**
 * Path timestep contract: index k = physical time k/resolution seconds.
 * UST-RRT* τ (seconds per layer, default 1) must map via τ * resolution, not
 * used directly as k. Pure-time waits need τ, not distance-only timing.
 */
#include "comotion/planning/Path.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

int main() {
    // Wrong legacy mapping: treat τ-like small integers as timestep indices k.
    comotion::Path mistaken;
    mistaken.push_back({0, 0, 0});
    mistaken.push_back({2, 0, 0});
    mistaken.waypoint_timesteps_ = {0, 1};
    comotion::Path wrong = mistaken;
    wrong.interpolate_to_timesteps(128, 2.0);

    // Manuscript-aligned: τ in seconds → k = llround(τ * resolution).
    comotion::Path from_tau = mistaken;
    std::vector<double> tau_sec = {0.0, 1.0};
    from_tau.set_waypoint_timesteps_from_tau(tau_sec, 128, 1.0);
    from_tau.interpolate_to_timesteps(128, 2.0);

    if (wrong.size() > 50) {
        std::cerr << "path_timestep_semantics: mistaken k=τ path should be "
                     "short, got "
                  << wrong.size() << "\n";
        return 1;
    }

    if (from_tau.size() < 100) {
        std::cerr << "path_timestep_semantics: τ→k mapping should span ~1s "
                     "at 128 Hz, got size "
                  << from_tau.size() << "\n";
        return 1;
    }

    if (from_tau.size() <= wrong.size() * 2) {
        std::cerr << "path_timestep_semantics: τ-based path should be much "
                     "longer than mistaken mapping\n";
        return 1;
    }

    // Wait in place: three identical configs, τ advances 0,1,2 s.
    comotion::Path wait;
    wait.push_back({5, 0, 0});
    wait.push_back({5, 0, 0});
    wait.push_back({5, 0, 0});
    std::vector<double> tau_wait = {0.0, 1.0, 2.0};
    wait.set_waypoint_timesteps_from_tau(tau_wait, 128, 1.0);
    if (wait.arrival_timestep() < 200) {
        std::cerr << "path_timestep_semantics: wait should advance k to ~256, "
                     "got "
                  << wait.arrival_timestep() << "\n";
        return 1;
    }

    comotion::Path wait_dist = wait;
    wait_dist.clearWaypointTimesteps();
    wait_dist.computeTimestepsFromDistance(128, 2.0);
    if (wait_dist.arrival_timestep() > 10) {
        std::cerr << "path_timestep_semantics: distance-only timing should not "
                     "encode pure waits (expected ~0 cumul), got last k "
                  << wait_dist.arrival_timestep() << "\n";
        return 1;
    }

    // equalizePaths: padding must extend waypoint_timesteps_ (hold at goal in time).
    comotion::Path short_p;
    short_p.push_back({0, 0, 0});
    short_p.push_back({1, 0, 0});
    short_p.waypoint_timesteps_ = {0, 1};
    comotion::Path long_p;
    for (size_t k = 0; k <= 10; ++k) {
        long_p.push_back({0, static_cast<double>(k), 0});
        long_p.waypoint_timesteps_.push_back(k);
    }
    std::vector<comotion::Path> pair{short_p, long_p};
    comotion::equalizePaths(pair);
    if (!pair[0].has_timesteps() || !pair[1].has_timesteps()) {
        std::cerr << "path_timestep_semantics: equalizePaths should preserve "
                     "has_timesteps on padded paths\n";
        return 1;
    }
    if (pair[0].size() != pair[1].size() || pair[0].size() != 11) {
        std::cerr << "path_timestep_semantics: equalizePaths length mismatch\n";
        return 1;
    }
    if (pair[0].arrival_timestep() != 10) {
        std::cerr << "path_timestep_semantics: equalizePaths last timestep want "
                     "10 got "
                  << pair[0].arrival_timestep() << "\n";
        return 1;
    }

    // CompositeRRT-style segment times: raw cumulative round can yield duplicate
    // timestep labels; Path enforces strict increase so interpolate_to_timesteps
    // maps global k=0 to path[0] (avoids CompositeRRT start mismatch).
    {
        comotion::Path plateau;
        plateau.push_back({0.0, 0.0, 0.0});
        plateau.push_back({0.00005, 0.0, 0.0});
        plateau.push_back({0.0001, 0.0, 0.0});
        plateau.push_back({1.0, 0.0, 0.0});
        const std::vector<double> seg_times{1.0 / 300.0, 1.0 / 300.0, 0.02};
        plateau.setTimestepsFromSegmentTimes(seg_times, 60);
        if (!plateau.has_timesteps()) {
            std::cerr << "path_timestep_semantics: plateau path needs 4 timesteps\n";
            return 1;
        }
        if (plateau.timestep_at(0) != 0) {
            std::cerr << "path_timestep_semantics: plateau first timestep must be 0\n";
            return 1;
        }
        for (size_t i = 1; i < plateau.size(); ++i) {
            if (plateau.timestep_at(i) <= plateau.timestep_at(i - 1)) {
                std::cerr << "path_timestep_semantics: want strictly increasing "
                             "waypoint_timesteps_ after setTimestepsFromSegmentTimes\n";
                return 1;
            }
        }
        comotion::Path sampled = plateau;
        sampled.interpolate_to_timesteps(60, 1.0);
        if (sampled.empty() || !sampled.has_timesteps()) {
            std::cerr << "path_timestep_semantics: plateau sampled path empty\n";
            return 1;
        }
        double max_start_diff = 0.0;
        for (size_t d = 0; d < sampled.front().size(); ++d)
            max_start_diff = std::max(max_start_diff,
                                      std::abs(sampled.front()[d] - plateau.front()[d]));
        if (max_start_diff > 1e-12) {
            std::cerr << "path_timestep_semantics: sampled start should match path[0], "
                         "max abs diff "
                      << max_start_diff << "\n";
            return 1;
        }
    }

    {
        comotion::Path synced;
        synced.push_back({0.0, 0.0, 0.0});
        synced.push_back({1.0, -1.0, 0.5});
        synced.push_back({0.25, 2.0, -0.5});
        synced.push_back({-3.0, 1.5, 0.25});
        synced.waypoint_timesteps_ = {0, 1, 2, 3};
        const comotion::Path original = synced;
        synced.interpolate_to_timesteps(60, 1.0);
        if (!synced.has_implicit_dense_timesteps() ||
            synced.has_explicit_timesteps()) {
            std::cerr << "path_timestep_semantics: already synchronized path "
                         "should store dense timesteps implicitly\n";
            return 1;
        }
        if (synced.size() != original.size()) {
            std::cerr << "path_timestep_semantics: already synchronized path "
                         "length changed\n";
            return 1;
        }
        double max_diff = 0.0;
        for (size_t i = 0; i < synced.size(); ++i) {
            for (size_t d = 0; d < synced[i].size(); ++d) {
                max_diff = std::max(
                    max_diff, std::abs(synced[i][d] - original[i][d]));
            }
        }
        for (size_t i = 0; i < synced.size(); ++i) {
            if (synced.timestep_at(i) != i) {
                std::cerr << "path_timestep_semantics: implicit dense timestep "
                             "lookup changed at index "
                          << i << "\n";
                return 1;
            }
        }
        if (max_diff > 1e-12) {
            std::cerr << "path_timestep_semantics: already synchronized path "
                         "configs changed, max abs diff "
                      << max_diff << "\n";
            return 1;
        }
    }

    std::cout << "path_timestep_semantics: OK (mistaken.size=" << wrong.size()
              << " tau_mapped.size=" << from_tau.size() << ")\n";
    return 0;
}
