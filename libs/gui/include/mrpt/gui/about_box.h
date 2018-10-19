/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <string>

namespace mrpt::gui
{
/** Shows the standard MRPT GUI "About Box" (wxWidgets version) */
void show_mrpt_about_box_wxWidgets(
	void* parent_wx_window, const std::string& appName,
	const std::string& additionalInfo = std::string(),
	const bool showStandardInfo = true);
void show_mrpt_about_box_Qt(
	const std::string& appName,
	const std::string& additionalInfo = std::string(),
	const bool showStandardInfo = true);
}  // namespace mrpt::gui
