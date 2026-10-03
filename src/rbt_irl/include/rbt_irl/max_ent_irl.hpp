#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <stop_token>
#include <utility>

#include "rbt_irl/concepts.hpp"
#include "rbt_irl/types.hpp"
#include "rbt_irl/utils.hpp"

namespace rbt_irl
{

/**
 * @brief Configuration for `maxEntIRL()`.
 *
 * @details `learning_rate` and `convergence_tol` must be positive, and
 * `max_iters` must be greater than zero.
 */
struct MaxEntIRLOptions
{
  double learning_rate{ 1.0 };
  int max_iters{ 50 };
  double convergence_tol{ 1e-4 };
};

namespace detail
{

inline void validateConfiguration(const Features& features,
                                  const MaxEntIRLOptions& options)
{
  if (features.empty())
  {
    throw std::invalid_argument("maxEntIRL: features list is empty.");
  }
  if (!(options.learning_rate > 0.0))
  {
    throw std::invalid_argument("maxEntIRL: learning_rate must be > 0.");
  }
  if (options.max_iters <= 0)
  {
    throw std::invalid_argument("maxEntIRL: max_iters must be > 0.");
  }
  if (!(options.convergence_tol > 0.0))
  {
    throw std::invalid_argument("maxEntIRL: convergence_tol must be > 0.");
  }
}

inline void validateDemos(const JointPaths& demos)
{
  if (demos.empty())
  {
    throw std::invalid_argument("maxEntIRL: demonstrations set is empty.");
  }
}

inline WeightVec initializeTheta(const WeightVec& theta0, std::size_t feature_dim)
{
  const Eigen::Index dimension = static_cast<Eigen::Index>(feature_dim);
  if (theta0.size() == 0)
  {
    return WeightVec::Ones(dimension);
  }
  if (theta0.size() != dimension)
  {
    throw std::invalid_argument("maxEntIRL: theta0 has wrong dimension.");
  }
  return theta0;
}

inline FeatureVec computeFeatureCounts(const JointPath& path, const Features& features)
{
  FeatureVec values = FeatureVec::Zero(static_cast<Eigen::Index>(features.size()));
  for (std::size_t i = 0; i < features.size(); ++i)
  {
    values[static_cast<Eigen::Index>(i)] = features[i]->evaluate(path.positions());
  }
  return values;
}

inline FeatureVec computeFeatureMean(const JointPaths& paths, const Features& features)
{
  FeatureVec sum = FeatureVec::Zero(static_cast<Eigen::Index>(features.size()));
  for (const JointPath& path : paths)
  {
    sum += computeFeatureCounts(path, features);
  }
  return sum / static_cast<double>(paths.size());
}

inline WeightVec estimateGradient(const FeatureVec& demo_mean,
                                  const FeatureVec& expected_mean)
{
  return demo_mean - expected_mean;
}

inline bool hasConverged(const WeightVec& theta_prev, const WeightVec& theta,
                         double convergence_tol)
{
  return (theta - theta_prev).norm() < convergence_tol;
}

template <typename Sampler>
  requires(PathSampler<Sampler>)
JointPaths sampleUnderTheta(const JointEndpointsSet& demo_endpoints,
                            const Features& features, const WeightVec& theta,
                            Sampler& sampler)
{
  RewardTerms rewards;
  rewards.reserve(features.size());

  for (std::size_t i = 0; i < features.size(); ++i)
  {
    rewards.push_back({ features[i], theta[static_cast<Eigen::Index>(i)] });
  }

  return std::invoke(sampler, demo_endpoints, rewards);
}

}  // namespace detail

/**
 * @brief Learns MaxEnt IRL weights from demonstration paths.
 * @tparam Sampler Path sampler satisfying `PathSampler`.
 * @tparam Callback Iteration callback satisfying `IterationCallback`.
 * @param[in] demo_paths Demonstration paths, length >= 1.
 * @param[in] features Features used to compute path feature values.
 * @param[in] sampler Path sampler.
 * @param[in] iteration_callback Callback invoked after each completed iteration.
 * @param[in] options Optimizer configuration.
 * @param[in] theta0 Initial weights; empty initializes all weights to one.
 * @param[in] stop_token Stop token used to terminate optimization early.
 * @return Learned weights, size = `features.size()`.
 * @throws std::invalid_argument If inputs or options are invalid.
 */
template <typename Sampler, typename Callback>
  requires(PathSampler<Sampler> && IterationCallback<Callback>)
WeightVec maxEntIRL(const JointPaths& demo_paths, const Features& features,
                    Sampler&& sampler, Callback&& iteration_callback,
                    const MaxEntIRLOptions& options = {}, const WeightVec& theta0 = {},
                    const std::stop_token& stop_token = {})
{
  detail::validateConfiguration(features, options);
  detail::validateDemos(demo_paths);
  WeightVec theta = detail::initializeTheta(theta0, features.size());

  const JointEndpointsSet demo_endpoints = utils::extractJointEndpoints(demo_paths);
  const FeatureVec demo_mean = detail::computeFeatureMean(demo_paths, features);
  const auto start_time = std::chrono::steady_clock::now();

  for (int iteration = 0; iteration < options.max_iters; ++iteration)
  {
    if (stop_token.stop_requested())
    {
      break;
    }

    const WeightVec theta_prev = theta;
    const JointPaths sampled_paths =
        detail::sampleUnderTheta(demo_endpoints, features, theta, sampler);
    const FeatureVec expected_mean = detail::computeFeatureMean(sampled_paths, features);
    const WeightVec gradient = detail::estimateGradient(demo_mean, expected_mean);

    theta -= options.learning_rate * gradient;

    const auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time);
    std::invoke(iteration_callback, theta, theta_prev, elapsed_time, iteration);

    if (detail::hasConverged(theta_prev, theta, options.convergence_tol))
    {
      break;
    }

    if (stop_token.stop_requested())
    {
      break;
    }
  }

  return theta;
}

/**
 * @brief Learns MaxEnt IRL weights without an iteration callback.
 * @tparam Sampler Path sampler satisfying `PathSampler`.
 * @param[in] demo_paths Demonstration paths, length >= 1.
 * @param[in] features Features used to compute path feature values.
 * @param[in] sampler Path sampler.
 * @param[in] options Optimizer configuration.
 * @param[in] theta0 Initial weights; empty initializes all weights to one.
 * @param[in] stop_token Stop token used to terminate optimization early.
 * @return Learned weights, size = `features.size()`.
 * @throws std::invalid_argument If inputs or options are invalid.
 */
template <typename Sampler>
  requires(PathSampler<Sampler>)
WeightVec maxEntIRL(const JointPaths& demo_paths, const Features& features,
                    Sampler&& sampler, const MaxEntIRLOptions& options = {},
                    const WeightVec& theta0 = {}, const std::stop_token& stop_token = {})
{
  auto iteration_callback = [](const WeightVec&, const WeightVec&,
                               std::chrono::milliseconds, int) {};
  return maxEntIRL(demo_paths, features, std::forward<Sampler>(sampler),
                   iteration_callback, options, theta0, stop_token);
}

}  // namespace rbt_irl
