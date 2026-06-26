#pragma once

#include "comotion/collision/ValidationTypes.h"

#include <vamp/vector.hh>

#include <algorithm>
#include <array>
#include <cstddef>
#include <vector>

namespace comotion {
namespace detail {

inline constexpr std::size_t kVampPackingWidth = vamp::FloatVectorWidth;

struct VampPackingLayout {
    std::array<std::size_t, kVampPackingWidth> timesteps{};
    std::size_t lanes = 0;
};

inline std::vector<VampPackingLayout>
makeVampPackingLayouts(std::size_t begin, std::size_t end,
                       VampBatchPacking packing) {
    std::vector<VampPackingLayout> packs;
    if (begin >= end)
        return packs;

    const std::size_t window = end - begin;
    if (packing == VampBatchPacking::Linear) {
        for (std::size_t pack_begin = begin; pack_begin < end;
             pack_begin += kVampPackingWidth) {
            VampPackingLayout pack;
            for (std::size_t timestep = pack_begin;
                 timestep < end && pack.lanes < kVampPackingWidth; ++timestep) {
                pack.timesteps[pack.lanes++] = timestep;
            }
            packs.push_back(pack);
        }
        return packs;
    }

    const std::size_t stride = std::max<std::size_t>(
        1, (window + kVampPackingWidth - 1) / kVampPackingWidth);
    for (std::size_t offset = 0; offset < stride; ++offset) {
        VampPackingLayout pack;
        for (std::size_t lane = 0; lane < kVampPackingWidth; ++lane) {
            const std::size_t timestep = begin + offset + lane * stride;
            if (timestep >= end)
                break;
            pack.timesteps[pack.lanes++] = timestep;
        }
        if (pack.lanes > 0)
            packs.push_back(pack);
    }

    return packs;
}

} // namespace detail
} // namespace comotion
