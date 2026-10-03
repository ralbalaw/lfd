#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

namespace rlc_executive
{

/**
 * @brief Returns a retained pointer-like value after validating it is non-null.
 * @tparam Ptr Pointer-like type with boolean null testing.
 * @param[in] ptr Retained pointer-like value; must be non-null.
 * @param[in] label Name used in the thrown diagnostic.
 * @return The original pointer-like value.
 * @throws std::invalid_argument If `ptr` is null.
 */
template <class Ptr>
Ptr requireNonNull(Ptr ptr, std::string_view label)
{
  if (!ptr)
  {
    throw std::invalid_argument(std::string(label) + " must not be null");
  }
  return ptr;
}

}  // namespace rlc_executive