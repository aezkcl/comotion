#pragma once

#include "comotion/collision/ValidationTypes.h"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace comotion {

enum class ValidationTraceCallType {
    CompositeState,
    CompositeMotion,
};

inline const char *validationTraceCallTypeName(
    ValidationTraceCallType type) {
    switch (type) {
    case ValidationTraceCallType::CompositeState:
        return "composite_state";
    case ValidationTraceCallType::CompositeMotion:
        return "composite_motion";
    }
    return "unknown";
}

inline ValidationTraceCallType parseValidationTraceCallType(
    const std::string &name) {
    if (name == "composite_state")
        return ValidationTraceCallType::CompositeState;
    if (name == "composite_motion")
        return ValidationTraceCallType::CompositeMotion;
    throw std::runtime_error("Unknown validation trace call type: " + name);
}

struct ValidationTraceRecord {
    ValidationTraceCallType type = ValidationTraceCallType::CompositeState;
    std::vector<std::vector<double>> configs;
    std::vector<std::vector<double>> from;
    std::vector<std::vector<double>> to;
    CompositePathValidationOptions options;
    bool result = false;
    std::uint64_t elapsed_nanoseconds = 0;
};

class ValidationTraceRecorder {
public:
    void clear() { records_.clear(); }
    const std::vector<ValidationTraceRecord> &records() const {
        return records_;
    }

    void recordCompositeState(std::vector<std::vector<double>> configs,
                              bool result,
                              std::uint64_t elapsed_nanoseconds) {
        ValidationTraceRecord record;
        record.type = ValidationTraceCallType::CompositeState;
        record.configs = std::move(configs);
        record.result = result;
        record.elapsed_nanoseconds = elapsed_nanoseconds;
        records_.push_back(std::move(record));
    }

    void recordCompositeMotion(std::vector<std::vector<double>> from,
                               std::vector<std::vector<double>> to,
                               CompositePathValidationOptions options,
                               bool result,
                               std::uint64_t elapsed_nanoseconds) {
        ValidationTraceRecord record;
        record.type = ValidationTraceCallType::CompositeMotion;
        record.from = std::move(from);
        record.to = std::move(to);
        record.options = std::move(options);
        record.result = result;
        record.elapsed_nanoseconds = elapsed_nanoseconds;
        records_.push_back(std::move(record));
    }

private:
    std::vector<ValidationTraceRecord> records_;
};

} // namespace comotion
