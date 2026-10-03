#pragma once

#include <vector>

#include "rbt_types/feature.hpp"

namespace rbt_types
{

/// @brief Weighted feature contributing to an objective.
struct ObjectiveTerm
{
  FeaturePtr feature;
  double weight{ 1.0 };
};

/// @brief Semantic alias for an objective term interpreted as a reward.
using RewardTerm = ObjectiveTerm;

/// @brief Semantic alias for an objective term interpreted as a cost.
using CostTerm = ObjectiveTerm;

/// @brief Collection of weighted objective terms.
using ObjectiveTerms = std::vector<ObjectiveTerm>;

/// @brief Collection of weighted reward terms.
using RewardTerms = ObjectiveTerms;

/// @brief Collection of weighted cost terms.
using CostTerms = ObjectiveTerms;

/**
 * @brief Negates every objective weight in place.
 * @param[in,out] terms Objective terms whose weights are negated.
 */
inline void negateWeightsInPlace(ObjectiveTerms& terms)
{
  for (auto& term : terms)
  {
    term.weight = -term.weight;
  }
}

/**
 * @brief Converts reward terms to cost terms by negating their weights.
 * @param[in] reward_terms Reward terms to convert.
 * @return Cost terms sharing the same features with negated weights.
 */
[[nodiscard]] inline CostTerms rewardToCost(const RewardTerms& reward_terms)
{
  CostTerms cost_terms = reward_terms;
  negateWeightsInPlace(cost_terms);
  return cost_terms;
}

/**
 * @brief Converts cost terms to reward terms by negating their weights.
 * @param[in] cost_terms Cost terms to convert.
 * @return Reward terms sharing the same features with negated weights.
 */
[[nodiscard]] inline RewardTerms costToReward(const CostTerms& cost_terms)
{
  RewardTerms reward_terms = cost_terms;
  negateWeightsInPlace(reward_terms);
  return reward_terms;
}

/**
 * @brief Evaluates a weighted objective over a joint path.
 * @param[in] terms Weighted features with non-null feature pointers.
 * @param[in] path Joint positions [rad], shape = (num_waypoints, dof)
 * RowMajor; the view is not stored.
 * @return Weighted sum of all feature values.
 */
inline double evaluateObjective(const ObjectiveTerms& terms, const JointPathView& path)
{
  double total = 0.0;
  for (const auto& term : terms)
  {
    total += term.weight * term.feature->evaluate(path);
  }
  return total;
}

}  // namespace rbt_types
