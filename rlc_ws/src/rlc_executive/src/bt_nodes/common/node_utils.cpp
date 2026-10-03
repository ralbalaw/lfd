#include "rlc_executive/bt_nodes/common/node_utils.hpp"

namespace rlc_executive
{
namespace bt_utils
{

std::string sanitizeLoggerName(std::string s)
{
  for (char& c : s)
  {
    if (c == '/')
    {
      c = '.';
      continue;
    }

    const bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
                    (c >= 'a' && c <= 'z') || (c == '.') || (c == '_');
    if (!ok)
    {
      c = '_';
    }
  }
  return s;
}

void setDiagnostics(BT::TreeNode& node, const std::string& error,
                    const std::string& feedback, double elapsed_sec)
{
  setOutput(node, DiagnosticPortKeys::kError, error);
  setOutput(node, DiagnosticPortKeys::kFeedback, feedback);
  setOutput(node, DiagnosticPortKeys::kElapsedSec, elapsed_sec);
}

void setRunningDiagnostics(BT::TreeNode& node, const std::string& feedback,
                           double elapsed_sec)
{
  setDiagnostics(node, std::string(), feedback, elapsed_sec);
}

void setFailureDiagnostics(BT::TreeNode& node, const std::string& error,
                           double elapsed_sec)
{
  setDiagnostics(node, error, std::string(), elapsed_sec);
}

void setSuccessDiagnostics(BT::TreeNode& node, double elapsed_sec)
{
  setDiagnostics(node, std::string(), std::string(), elapsed_sec);
}

}  // namespace bt_utils
}  // namespace rlc_executive
