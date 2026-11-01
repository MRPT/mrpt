/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/imgui/config_paths.h>

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <system_error>

namespace mrpt::imgui
{
std::string sanitizeFileName(const std::string& s)
{
  std::string out;
  out.reserve(s.size());
  for (const char c : s)
  {
    const auto uc = static_cast<unsigned char>(c);
    const bool bad = uc < 32 || c == '/' || c == '\\' || c == ':' || c == '*' || c == '?' ||
                     c == '"' || c == '<' || c == '>' || c == '|';
    out.push_back(bad ? '_' : c);
  }
  const auto first = out.find_first_not_of(" .");
  if (first == std::string::npos)
  {
    return "app";
  }
  const auto last = out.find_last_not_of(" .");
  return out.substr(first, last - first + 1);
}

namespace
{
std::string envOrEmpty(const char* name)
{
  const char* v = std::getenv(name);
  return v != nullptr ? std::string(v) : std::string();
}
}  // namespace

std::string userConfigDirectory(const std::string& appName, bool create)
{
  namespace fs = std::filesystem;

  fs::path base;
#if defined(_WIN32)
  base = envOrEmpty("APPDATA");
#else
  if (const auto xdg = envOrEmpty("XDG_CONFIG_HOME"); !xdg.empty())
  {
    base = xdg;
  }
  else if (const auto home = envOrEmpty("HOME"); !home.empty())
  {
    base = fs::path(home) / ".config";
  }
#endif
  if (base.empty())
  {
    return {};
  }

  const fs::path dir = base / sanitizeFileName(appName);
  if (create)
  {
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec || !fs::is_directory(dir))
    {
      return {};
    }
  }
  std::string ret = dir.string();
  ret.push_back(static_cast<char>(fs::path::preferred_separator));
  return ret;
}

}  // namespace mrpt::imgui
