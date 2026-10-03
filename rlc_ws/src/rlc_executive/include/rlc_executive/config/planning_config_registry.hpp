#pragma once

#include <string>

#include "rlc_executive/config/planning_config_map.hpp"

namespace rlc_executive
{

/**
 * @brief Owns named planning configs and resolves the configured default config.
 *
 * @details
 * The registry is valid by construction: it must contain at least one config, and
 * the default config name must be non-empty and present in the map.
 */
class PlanningConfigRegistry final
{
public:
  /// @brief Empty registries are invalid; construct with parsed config data.
  PlanningConfigRegistry() = delete;

  /**
   * @brief Creates a planning-config registry from parsed config data.
   * @param[in] default_config Default config name; must be non-empty and present in
   * `configs`.
   * @param[in] configs Named planning configs owned by this registry.
   * @throws std::invalid_argument If `configs` is empty, `default_config` is empty,
   * or `default_config` is not present in `configs`.
   */
  PlanningConfigRegistry(std::string default_config, PlanningConfigMap configs);

  /**
   * @brief Finds a planning config by name.
   * @param[in] name Planning config name.
   * @return Pointer to the config owned by this registry, or nullptr if not found.
   */
  const PlanningConfig* find(const std::string& name) const noexcept;

  /**
   * @brief Returns the configured default planning config.
   * @return Reference to the default config owned by this registry.
   */
  const PlanningConfig& defaultConfig() const noexcept;

  /**
   * @brief Returns the configured default planning config name.
   * @return Reference to the stored default config name.
   */
  const std::string& defaultConfigName() const noexcept;

  /**
   * @brief Reports whether a planning config exists.
   * @param[in] name Planning config name.
   * @return `true` if `name` exists in this registry, otherwise `false`.
   */
  bool contains(const std::string& name) const noexcept;

private:
  void validateDefaultConfig() const;

  std::string default_config_;
  PlanningConfigMap configs_;
};

}  // namespace rlc_executive
