
#include "rbt_planning/utils.hpp"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <stdexcept>

namespace rbt_planning
{
namespace utils
{

bool fromOMPL(const ompl::geometric::PathGeometric& path, int n_steps, std::size_t dof,
              trajopt::TrajArray& out)
{
  if (n_steps < 2)
  {
    throw std::invalid_argument("fromOMPL: n_steps must be >= 2");
  }
  if (dof == 0)
  {
    throw std::invalid_argument("fromOMPL: dof must be > 0");
  }

  // Interpolating modifies the path, so we work on a copy.
  ompl::geometric::PathGeometric path_copy = path;
  path_copy.interpolate(static_cast<unsigned int>(n_steps));

  const auto& states = path_copy.getStates();
  if (states.size() != static_cast<std::size_t>(n_steps))
  {
    throw std::runtime_error("fromOMPL: OMPL path interpolation did not match n_steps");
  }

  trajopt::TrajArray seed(n_steps, static_cast<Eigen::Index>(dof));
  for (int i = 0; i < n_steps; ++i)
  {
    const auto* rv = states[static_cast<std::size_t>(i)]
                         ->as<ompl::base::RealVectorStateSpace::StateType>();
    for (std::size_t j = 0; j < dof; ++j)
    {
      seed(i, static_cast<Eigen::Index>(j)) = rv->values[j];
    }
  }
  out = seed;
  return true;
}
}  // namespace utils
}  // namespace rbt_planning
