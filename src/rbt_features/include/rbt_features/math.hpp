#pragma once

#include <cmath>

namespace rbt_features
{
namespace math
{
inline double sigmoid(double x, double slope, double midpoint)
{ return 1.0 / (1.0 + std::exp(-slope * (x - midpoint))); }

inline double normalizedSigmoid(double x, double slope, double midpoint)
{
  const double s = sigmoid(x, slope, midpoint);
  const double s0 = sigmoid(0.0, slope, midpoint);
  return (s - s0) / (1.0 - s0);
}

inline double gaussian(double x, double sigma)
{
  const double z = x / sigma;
  return std::exp(-0.5 * z * z);
}

inline double gaussian(double x, double mean, double sigma)
{
  const double z = (x - mean) / sigma;
  return std::exp(-0.5 * z * z);
}
}  // namespace math
}  // namespace rbt_features