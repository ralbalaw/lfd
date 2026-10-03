#pragma once

#include <trajopt/problem_description.hpp>
#include <trajopt/utils.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>

namespace ompl::geometric
{
class PathGeometric;
}  // namespace ompl::geometric

namespace rbt_planning
{
namespace utils
{
/**
 * @brief Converts an OMPL geometric solution path into a TrajOpt seed trajectory.
 * @param[in] path OMPL solution path.
 * @param[in] n_steps Number of timesteps in the output trajectory (rows), must be >= 2.
 * @param[in] dof Joint dimension (columns), must be > 0.
 * @param[out] out Output trajectory [rad], shape = (`n_steps`, `dof`).
 * @return True on success.
 * @throws std::invalid_argument If inputs are invalid.
 * @throws std::runtime_error If OMPL path interpolation fails.
 */
bool fromOMPL(const ompl::geometric::PathGeometric& path, int n_steps, std::size_t dof,
              trajopt::TrajArray& out);

}
}  // namespace rbt_planning
