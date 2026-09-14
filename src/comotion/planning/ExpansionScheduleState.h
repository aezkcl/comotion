#pragma once

#include <cstddef>
#include <cstdint>

namespace comotion {

struct ExpansionScheduleState {
    bool initial_valid_window_established = false;
    bool last_expansion_used_initial_valid_schedule = false;
    std::size_t initial_valid_expansion_index = 0;
    std::size_t main_expansion_index = 0;
    bool initial_search_geometry_initialized = false;
    std::int64_t initial_search_center_twice = 0;
    std::int64_t initial_search_half_width_twice = 0;
    std::int64_t main_window_center_twice = 0;
    std::int64_t main_base_half_width_twice = 0;
};

} // namespace comotion
