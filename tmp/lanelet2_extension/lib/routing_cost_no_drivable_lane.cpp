#include "lanelet2_extension/routing_cost/routing_cost_no_drivable_lane.hpp"

#include <boost/geometry/algorithms/perimeter.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>
#include <lanelet2_core/utility/Units.h>

namespace lanelet
{
namespace routing
{

double RoutingCostNoDrivableLane::length(const ConstLanelet & ll) noexcept
{
  return geometry::approximatedLength2d(ll);
}

double RoutingCostNoDrivableLane::length(const ConstArea & ar) noexcept
{
  return double(boost::geometry::perimeter(utils::to2D(ar.outerBoundPolygon())));
}
}  // namespace routing
}  // namespace lanelet