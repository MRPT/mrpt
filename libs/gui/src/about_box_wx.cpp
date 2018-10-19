/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers

#include <mrpt/gui/about_box.h>
#include <mrpt/core/exceptions.h>

#if MRPT_HAS_WXWIDGETS
#include "CAboutBox_wx.h"
#endif

void mrpt::gui::show_mrpt_about_box_wxWidgets(
	void* parent_wx_window, const std::string& appName,
	const std::string& additionalInfo, const bool showStandardInfo)
{
#if MRPT_HAS_WXWIDGETS
	auto* parent = reinterpret_cast<wxWindow*>(parent_wx_window);
	CAboutBox dlg(parent, appName, additionalInfo, showStandardInfo);
	dlg.ShowModal();
#else
	THROW_EXCEPTION("MRPT compiled without wxWidgets support");
#endif
}
