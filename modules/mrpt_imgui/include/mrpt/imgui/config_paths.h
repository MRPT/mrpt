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
#pragma once

#include <string>

namespace mrpt::imgui
{
/** Returns `s` with every character that is not safe in a file name across
 *  platforms (path separators, `:*?"<>|`, control characters) replaced with
 *  an underscore, and leading/trailing spaces and dots removed. Returns
 *  `"app"` if nothing is left.
 * \ingroup mrpt_imgui_grp
 */
[[nodiscard]] std::string sanitizeFileName(const std::string& s);

/** The per-user configuration directory of an application, with a trailing
 *  path separator: `$XDG_CONFIG_HOME/<app>/`, or `~/.config/<app>/` on
 *  Unix-like systems; `%APPDATA%\<app>\` on Windows.
 *
 *  \param appName The application name; sanitized with sanitizeFileName().
 *  \param create If true, the directory is created if it does not exist.
 *  \return The directory, or an empty string if no home or config directory
 *          could be determined (or created).
 * \ingroup mrpt_imgui_grp
 */
[[nodiscard]] std::string userConfigDirectory(const std::string& appName, bool create = true);

}  // namespace mrpt::imgui
