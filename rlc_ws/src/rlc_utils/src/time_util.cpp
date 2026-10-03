#include "rlc_utils/time_util.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace rlc_utils::time_util
{
namespace
{

template <typename IntT>
constexpr IntT Clamp(IntT v, IntT lo, IntT hi) noexcept
{
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

}  // namespace

double from_ros_time(const builtin_interfaces::msg::Time& t) noexcept
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

double from_ros_duration(const builtin_interfaces::msg::Duration& d) noexcept
{
  return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
}

builtin_interfaces::msg::Time to_ros_time(double sec) noexcept
{
  if (!std::isfinite(sec) || sec < 0.0)
    sec = 0.0;

  auto sec_i64 = static_cast<std::int64_t>(std::floor(sec));
  double frac = sec - static_cast<double>(sec_i64);
  auto nsec_u64 = static_cast<std::uint64_t>(std::llround(frac * 1e9));

  if (nsec_u64 >= 1000000000ULL)
  {
    sec_i64 += 1;
    nsec_u64 -= 1000000000ULL;
  }

  sec_i64 = Clamp<std::int64_t>(
      sec_i64, 0, static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()));

  builtin_interfaces::msg::Time out;
  out.sec = static_cast<std::int32_t>(sec_i64);
  out.nanosec =
      static_cast<std::uint32_t>(std::min<std::uint64_t>(nsec_u64, 999999999ULL));
  return out;
}

builtin_interfaces::msg::Duration to_ros_duration(double sec) noexcept
{
  if (!std::isfinite(sec))
    sec = 0.0;
  if (sec < 0.0)
    sec = 0.0;

  auto sec_i64 = static_cast<std::int64_t>(std::floor(sec));
  double frac = sec - static_cast<double>(sec_i64);
  auto nsec_u64 = static_cast<std::uint64_t>(std::llround(frac * 1e9));

  if (nsec_u64 >= 1000000000ULL)
  {
    sec_i64 += 1;
    nsec_u64 -= 1000000000ULL;
  }

  sec_i64 = Clamp<std::int64_t>(
      sec_i64, static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::min()),
      static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()));

  builtin_interfaces::msg::Duration out;
  out.sec = static_cast<std::int32_t>(sec_i64);
  out.nanosec =
      static_cast<std::uint32_t>(std::min<std::uint64_t>(nsec_u64, 999999999ULL));
  return out;
}

}  // namespace rlc_utils::time_util
