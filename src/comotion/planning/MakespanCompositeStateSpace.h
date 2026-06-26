#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace comotion {

// Composite state space whose distance is the longest individual robot motion.
// AORRTC/AOXRRTConnect use SpaceInformation::distance() for both path length and
// AOX cost-space reasoning, so this is the CoMotion makespan adaptation point.
class MakespanCompositeStateSpace
    : public ompl::base::RealVectorStateSpace {
public:
    explicit MakespanCompositeStateSpace(const std::vector<unsigned int> &block_dims)
        : ompl::base::RealVectorStateSpace(totalDimension(block_dims)),
          block_dims_(block_dims) {
        if (block_dims_.empty())
            throw std::invalid_argument(
                "MakespanCompositeStateSpace requires at least one block");
        for (const unsigned int dim : block_dims_) {
            if (dim == 0)
                throw std::invalid_argument(
                    "MakespanCompositeStateSpace block dimension must be positive");
        }
        setName("MakespanComposite" + getName());
    }

    double distance(const ompl::base::State *state1,
                    const ompl::base::State *state2) const override {
        const double *s1 =
            state1->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        const double *s2 =
            state2->as<ompl::base::RealVectorStateSpace::StateType>()->values;

        unsigned int offset = 0;
        double max_block_distance = 0.0;
        for (const unsigned int block_dim : block_dims_) {
            double block_sq = 0.0;
            for (unsigned int i = 0; i < block_dim; ++i) {
                if (!std::isfinite(s1[offset + i]) ||
                    !std::isfinite(s2[offset + i]))
                    return std::numeric_limits<double>::infinity();
                const double diff = s1[offset + i] - s2[offset + i];
                block_sq += diff * diff;
            }
            max_block_distance =
                std::max(max_block_distance, std::sqrt(block_sq));
            offset += block_dim;
        }
        return max_block_distance;
    }

    double getMaximumExtent() const override {
        unsigned int offset = 0;
        double max_block_extent = 0.0;
        for (const unsigned int block_dim : block_dims_) {
            double block_sq = 0.0;
            for (unsigned int i = 0; i < block_dim; ++i) {
                const double span =
                    bounds_.high[offset + i] - bounds_.low[offset + i];
                if (!std::isfinite(span))
                    return std::numeric_limits<double>::infinity();
                block_sq += span * span;
            }
            max_block_extent =
                std::max(max_block_extent, std::sqrt(block_sq));
            offset += block_dim;
        }
        return max_block_extent;
    }

    bool satisfiesBounds(const ompl::base::State *state) const override {
        const double *values =
            state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (unsigned int i = 0; i < getDimension(); ++i) {
            if (!std::isfinite(values[i]))
                return false;
        }
        return ompl::base::RealVectorStateSpace::satisfiesBounds(state);
    }

    const std::vector<unsigned int> &blockDimensions() const {
        return block_dims_;
    }

private:
    static unsigned int totalDimension(const std::vector<unsigned int> &dims) {
        unsigned int total = 0;
        for (const unsigned int dim : dims)
            total += dim;
        return total;
    }

    std::vector<unsigned int> block_dims_;
};

} // namespace comotion
