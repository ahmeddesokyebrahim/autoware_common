#pragma once

#include "lanelet2_routing/Forward.h"

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

namespace lanelet
{
namespace routing
{
/** @brief A basic distance-based routing cost module for no-drivable lane.
 *  Uses the 2D length and a fixed lane change cost to evaluate relations.
 *  If there is a no-drivable lane, it will assign the max cost transition from a lane to this
 * no-drivable lane */
class RoutingCostNoDrivableLane : public RoutingCost
{
public:
  //! Distance cost of a lane change [m] for no-drivable lane.
  //! If a lane change requires less than minLaneChangeLength, no lane change will
  //! be possible here. Instead, relation between the involved lanelets will be "adjacent".
  explicit RoutingCostNoDrivableLane(double laneChangeCost, double minLaneChangeLength = 0.)
  : laneChangeCost_{laneChangeCost}, minChangeLength_{minLaneChangeLength}
  {
    if (laneChangeCost_ < 0.0) {
      throw InvalidInputError(
        "Lane change cost must be positive, but it is " + std::to_string(laneChangeCost_));
    }
  }
  inline double getCostSucceeding(
    const traffic_rules::TrafficRules & /*trafficRules*/, const ConstLaneletOrArea & from,
    const ConstLaneletOrArea & to) const override
  {
    const std::string no_drivable_lane = to.lanelet()->attributeOr("no_drivable_lane", "no");
    if (no_drivable_lane == "yes") {
      return std::numeric_limits<double>::max();
    } else {
      auto getLength = [this](auto & lltOrArea) -> double { return this->length(lltOrArea); };
      return (from.applyVisitor(getLength) + to.applyVisitor(getLength)) / 2;
    }
  }
  inline double getCostLaneChange(
    const traffic_rules::TrafficRules & /*trafficRules*/, const ConstLanelets & from,
    const ConstLanelets & /*to*/) const noexcept override
  {
    if (minChangeLength_ <= 0.) {
      return laneChangeCost_;
    }
    auto totalLength = std::accumulate(
      from.begin(), from.end(), 0.,
      [this](double acc, auto & llt) { return acc + this->length(llt); });
    return totalLength >= minChangeLength_ ? laneChangeCost_
                                           : std::numeric_limits<double>::infinity();
  }

private:
  static double length(const ConstLanelet & ll) noexcept;
  static double length(const ConstArea & ar) noexcept;
  const double laneChangeCost_, minChangeLength_;  // NOLINT
};
}  // namespace routing
}  // namespace lanelet