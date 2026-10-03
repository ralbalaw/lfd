#pragma once

#include <chrono>
#include <concepts>
#include <functional>

#include "rbt_irl/types.hpp"

namespace rbt_irl
{

/// @brief Callable that samples joint paths for weighted reward terms.
template <typename Sampler>
concept PathSampler = requires(Sampler& sampler, const JointEndpointsSet& endpoints,
                               const RewardTerms& rewards) {
  { std::invoke(sampler, endpoints, rewards) } -> std::same_as<JointPaths>;
};

/// @brief Callable invoked after a MaxEnt IRL iteration.
template <typename Callback>
concept IterationCallback =
    requires(Callback& callback, const WeightVec& theta, const WeightVec& theta_prev,
             std::chrono::milliseconds elapsed_time, int iteration) {
      {
        std::invoke(callback, theta, theta_prev, elapsed_time, iteration)
      } -> std::same_as<void>;
    };

}  // namespace rbt_irl
