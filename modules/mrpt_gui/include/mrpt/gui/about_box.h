/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#pragma once

#include <string>

namespace mrpt::gui
{
/** Shows the standard MRPT GUI "About Box" (wxWidgets version) */
void show_mrpt_about_box_wxWidgets(
    void* parent_wx_window,
    const std::string& appName,
    const std::string& additionalInfo = std::string(),
    const bool showStandardInfo = true);
void show_mrpt_about_box_Qt(
    const std::string& appName,
    const std::string& additionalInfo = std::string(),
    const bool showStandardInfo = true);
}  // namespace mrpt::gui
