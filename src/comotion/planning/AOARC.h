#pragma once

#include "comotion/planning/ARC.h"
#include "comotion/planning/AORRTCUtils.h"

namespace comotion {

/// Stable planner API.
///
/// Anytime-optimization wrapper around ARC that repeatedly tightens the current
/// makespan bound.
class AOARC : public ARC {
public:
    ompl::base::PlannerStatus solve(double timeLimit) override;
    std::string name() const override { return "AOARC"; }

protected:
    void configureArcAttempt(ARC &planner) const;

private:
    static nlohmann::json
    solutionEventsJson(const std::vector<aorrtc::SolutionEvent> &events);
};

} // namespace comotion
