#include "rlc_frontend/frontend/joy_mapping.hpp"

#include <cctype>
#include <stdexcept>
#include <string>

namespace rlc_frontend::frontend
{
namespace
{

std::string normalizeToken(std::string_view value)
{
  std::string normalized;
  normalized.reserve(value.size());

  for (const unsigned char c : value)
  {
    if (std::isspace(c) != 0)
    {
      continue;
    }

    if (c == '-')
    {
      normalized.push_back('_');
      continue;
    }

    normalized.push_back(static_cast<char>(std::toupper(c)));
  }

  return normalized;
}

int parseRawIndex(const std::string& value, std::string_view context)
{
  if (value.empty())
  {
    throw std::invalid_argument(std::string(context) + " must not be empty");
  }

  try
  {
    std::size_t consumed = 0;
    const int index = std::stoi(value, &consumed);
    if (consumed == value.size())
    {
      if (index < 0)
      {
        throw std::invalid_argument(std::string(context) + " must be >= 0");
      }
      return index;
    }
  }
  catch (const std::invalid_argument&)
  {
  }
  catch (const std::out_of_range&)
  {
    throw std::invalid_argument(std::string(context) + " is out of range");
  }

  return -1;
}

int lookupAxisToken(const std::string& token)
{
  if (token == "LEFTX")
  {
    return 0;
  }
  if (token == "LEFTY")
  {
    return 1;
  }
  if (token == "RIGHTX")
  {
    return 2;
  }
  if (token == "RIGHTY")
  {
    return 3;
  }
  if (token == "TRIGGERLEFT")
  {
    return 4;
  }
  if (token == "TRIGGERRIGHT")
  {
    return 5;
  }

  return -1;
}

int lookupButtonToken(const std::string& token)
{
  if (token == "A")
  {
    return 0;
  }
  if (token == "B")
  {
    return 1;
  }
  if (token == "X")
  {
    return 2;
  }
  if (token == "Y")
  {
    return 3;
  }
  if (token == "BACK" || token == "SELECT")
  {
    return 4;
  }
  if (token == "GUIDE")
  {
    return 5;
  }
  if (token == "START")
  {
    return 6;
  }
  if (token == "LEFTSTICK")
  {
    return 7;
  }
  if (token == "RIGHTSTICK")
  {
    return 8;
  }
  if (token == "LEFTSHOULDER")
  {
    return 9;
  }
  if (token == "RIGHTSHOULDER")
  {
    return 10;
  }
  if (token == "DPAD_UP")
  {
    return 11;
  }
  if (token == "DPAD_DOWN")
  {
    return 12;
  }
  if (token == "DPAD_LEFT")
  {
    return 13;
  }
  if (token == "DPAD_RIGHT")
  {
    return 14;
  }

  return -1;
}

}  // namespace

int resolveJoyAxisIndex(std::string_view value, std::string_view context)
{
  const std::string raw(value);
  if (const int index = parseRawIndex(raw, context); index >= 0)
  {
    return index;
  }

  const int index = lookupAxisToken(normalizeToken(value));
  if (index >= 0)
  {
    return index;
  }

  throw std::invalid_argument(std::string(context) + ": unsupported axis mapping '" +
                              raw + "'");
}

int resolveJoyButtonIndex(std::string_view value, std::string_view context)
{
  const std::string raw(value);
  if (const int index = parseRawIndex(raw, context); index >= 0)
  {
    return index;
  }

  const int index = lookupButtonToken(normalizeToken(value));
  if (index >= 0)
  {
    return index;
  }

  throw std::invalid_argument(std::string(context) + ": unsupported button mapping '" +
                              raw + "'");
}

}  // namespace rlc_frontend::frontend
