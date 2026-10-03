#pragma once

#include <string_view>

namespace rlc_frontend::frontend
{

/**
 * @brief Resolves one joystick axis token or raw index to a concrete SDL axis index.
 * @param[in] value Raw YAML scalar for one joystick axis mapping.
 * @param[in] context Human-readable field name used in error messages.
 * @return Non-negative SDL axis index.
 * @throws std::invalid_argument If `value` is empty, negative, or not a supported SDL axis token.
 */
int resolveJoyAxisIndex(std::string_view value, std::string_view context);

/**
 * @brief Resolves one joystick button token or raw index to a concrete SDL button index.
 * @param[in] value Raw YAML scalar for one joystick button mapping.
 * @param[in] context Human-readable field name used in error messages.
 * @return Non-negative SDL button index.
 * @throws std::invalid_argument If `value` is empty, negative, or not a supported SDL
 * button token.
 */
int resolveJoyButtonIndex(std::string_view value, std::string_view context);

}  // namespace rlc_frontend::frontend
