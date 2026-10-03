#pragma once

#include <string>

#include <behaviortree_cpp/exceptions.h>
#include <behaviortree_cpp/tree_node.h>

namespace rlc_executive
{

namespace bt_utils
{

/**
 * @brief Standard diagnostic port names shared by BT nodes.
 *
 * @details
 * Use these port names consistently across nodes, and remap them in XML to avoid
 * blackboard collisions between different node instances.
 */
struct DiagnosticPortKeys
{
  static inline const std::string kError = "error";
  static inline const std::string kFeedback = "feedback";
  static inline const std::string kElapsedSec = "elapsed_sec";
};

/**
 * @brief Sanitize a string for use as part of an rclcpp logger name.
 *
 * @param[in] s Input name.
 * @return Sanitized name ("/" -> ".", other invalid chars -> "_").
 */
std::string sanitizeLoggerName(std::string s);

/**
 * @brief Returns a retained pointer-like value after validating it is non-null.
 * @tparam Ptr Pointer-like type with boolean null testing.
 * @param[in] ptr Retained pointer-like value; must be non-null.
 * @param[in] node BT node used to format the diagnostic.
 * @param[in] label Name used in the thrown diagnostic.
 * @return The original pointer-like value.
 * @throws BT::LogicError If `ptr` is null.
 */
template <class Ptr>
Ptr requireNonNull(Ptr ptr, const BT::TreeNode& node, const std::string& label)
{
  if (!ptr)
  {
    throw BT::LogicError(node.registrationName(), "[", node.fullPath(), "]: ", label,
                         " is null");
  }
  return ptr;
}

/**
 * @brief Sets an output port value and throws if it fails.
 *
 * @tparam T Type of the output value.
 * @param[in,out] node BT node setting the output.
 * @param[in] key Output port name.
 * @param[in] value Output value.
 * @throws BT::RuntimeError If setting the output port fails (e.g. port missing or not
 * remapped to a blackboard entry).
 */
template <class T>
void setOutput(BT::TreeNode& node, const std::string& key, const T& value)
{
  auto res = node.setOutput(key, value);

  if (!res)
  {
    throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                           "]: failed to set output port '", key, "': ", res.error());
  }
}

/**
 * @brief Writes all standard diagnostic outputs.
 *
 * @param[in,out] node BT node writing diagnostic outputs.
 * @param[in] error Error message; empty to clear.
 * @param[in] feedback Feedback message; empty to clear.
 * @param[in] elapsed_sec Elapsed time [s].
 * @throws BT::RuntimeError If any diagnostic output port can not be written.
 */
void setDiagnostics(BT::TreeNode& node, const std::string& error,
                    const std::string& feedback, double elapsed_sec);

/**
 * @brief Writes standard diagnostic outputs for a running node.
 *
 * @param[in,out] node BT node writing diagnostic outputs.
 * @param[in] feedback Feedback message.
 * @param[in] elapsed_sec Elapsed time [s].
 * @throws BT::RuntimeError If any diagnostic output port can not be written.
 */
void setRunningDiagnostics(BT::TreeNode& node, const std::string& feedback,
                           double elapsed_sec);

/**
 * @brief Writes standard diagnostic outputs for a successful completion.
 *
 * @param[in,out] node BT node writing diagnostic outputs.
 * @param[in] elapsed_sec Elapsed time [s].
 * @throws BT::RuntimeError If any diagnostic output port can not be written.
 */
void setSuccessDiagnostics(BT::TreeNode& node, double elapsed_sec);

/**
 * @brief Writes standard diagnostic outputs for a failure completion.
 *
 * @param[in,out] node BT node writing diagnostic outputs.
 * @param[in] error Error message.
 * @param[in] elapsed_sec Elapsed time [s].
 * @throws BT::RuntimeError If any diagnostic output port can not be written.
 */
void setFailureDiagnostics(BT::TreeNode& node, const std::string& error,
                           double elapsed_sec);

/**
 * @brief Reads a required input port.
 *
 * @details
 * Wraps `BT::TreeNode::getInput(key, out)` and throws if the port is missing or
 * conversion fails.
 *
 * @tparam T Type of the input port.
 * @param[in] node BT node reading the input.
 * @param[in] key Input port name.
 * @return Input value.
 * @throws BT::RuntimeError If the input port is missing or can not be converted to `T`.
 */
template <class T>
T requireInput(const BT::TreeNode& node, const std::string& key)
{
  T out{};
  auto res = node.getInput(key, out);
  if (!res)
  {
    throw BT::RuntimeError(node.registrationName(), "[", node.fullPath(),
                           "]: failed to read input port '", key, "': ", res.error());
  }
  return out;
}

}  // namespace bt_utils
}  // namespace rlc_executive
