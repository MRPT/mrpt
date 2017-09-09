/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/gui/link_pragmas.h>
#include <string>

namespace mrpt
{
namespace gui
{
/** Shows the standard MRPT GUI "About Box" (wxWidgets version) */
void GUI_IMPEXP show_mrpt_about_box_wxWidgets(
	void* parent_wx_window, const std::string& appName,
	const std::string& additionalInfo = std::string(),
	const bool showStandardInfo = true);
void GUI_IMPEXP show_mrpt_about_box_Qt(
	const std::string& appName,
	const std::string& additionalInfo = std::string(),
	const bool showStandardInfo = true);
}
}
