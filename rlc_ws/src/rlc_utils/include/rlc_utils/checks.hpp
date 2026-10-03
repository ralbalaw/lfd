#pragma once

/**
 * @file checks.hpp
 * @brief Small helpers for validating container sizes and Eigen shapes.
 */

#include <Eigen/Core>

#include <cstddef>

namespace rlc_utils::checks
{

/**
 * @brief Returns `v.size()` as `std::size_t`.
 * @param[in] v Container-like object with a `size()` method.
 * @return Size cast to `std::size_t`.
 */
template <typename T>
[[nodiscard]] inline std::size_t size_of(const T& v) noexcept
{
  return static_cast<std::size_t>(v.size());
}

/**
 * @brief Checks whether `v.size()` equals `n`.
 * @param[in] v Container-like object with a `size()` method.
 * @param[in] n Expected size.
 * @return True if `size_of(v) == n`.
 */
template <typename T>
[[nodiscard]] inline bool size_is(const T& v, std::size_t n) noexcept
{
  return size_of(v) == n;
}

/**
 * @brief Checks whether all containers have size `n`.
 * @param[in] n Expected size.
 * @param[in] vs Containers.
 * @return True if every element of `vs...` satisfies `size_is(v, n)`.
 */
template <typename... Ts>
[[nodiscard]] inline bool all_size_is(std::size_t n, const Ts&... vs) noexcept
{
  return (... && size_is(vs, n));
}

/**
 * @brief Checks whether all containers have the same size.
 * @param[in] first First container.
 * @param[in] rest Remaining containers.
 * @return True if all containers in `rest...` have size `first.size()`.
 */
template <typename First, typename... Rest>
[[nodiscard]] inline bool all_same_size(const First& first, const Rest&... rest) noexcept
{
  const auto n = size_of(first);
  return (... && size_is(rest, n));
}

/**
 * @brief Checks whether a container is empty or has size `n`.
 * @param[in] c Container-like object with `empty()` and `size()`.
 * @param[in] n Expected size if non-empty.
 * @return True if `c.empty()` or `size_is(c, n)`.
 */
template <typename Container>
[[nodiscard]] inline bool empty_or_size_is(const Container& c, std::size_t n) noexcept
{
  return c.empty() || size_is(c, n);
}

/**
 * @brief Checks whether all containers are empty or have size `n`.
 * @param[in] n Expected size if non-empty.
 * @param[in] vs Containers.
 * @return True if every element of `vs...` satisfies `empty_or_size_is(v, n)`.
 */
template <typename... Ts>
[[nodiscard]] inline bool all_empty_or_size_is(std::size_t n, const Ts&... vs) noexcept
{
  return (... && empty_or_size_is(vs, n));
}

/**
 * @brief Checks whether an Eigen matrix has the given shape.
 * @param[in] m Eigen matrix/expression.
 * @param[in] rows Expected row count.
 * @param[in] cols Expected column count.
 * @return True if `shape = (rows, cols)`.
 */
template <typename Derived>
[[nodiscard]] inline bool shape_is(const Eigen::MatrixBase<Derived>& m, Eigen::Index rows,
                                   Eigen::Index cols) noexcept
{
  return (m.rows() == rows) && (m.cols() == cols);
}

}  // namespace rlc_utils::checks
