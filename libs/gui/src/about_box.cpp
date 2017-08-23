/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers

#include <mrpt/gui/about_box.h>

#if MRPT_HAS_WXWIDGETS
#include "CAboutBox_wx.h"
#endif

#if MRPT_HAS_Qt5
#include "CAboutBoxQt.h"
#endif

void GUI_IMPEXP mrpt::gui::show_mrpt_about_box_wxWidgets(
	void* parent_wx_window, const std::string& appName,
	const std::string& additionalInfo, const bool showStandardInfo)
{
#if MRPT_HAS_WXWIDGETS
	wxWindow* parent = reinterpret_cast<wxWindow*>(parent_wx_window);
	CAboutBox dlg(parent, appName, additionalInfo, showStandardInfo);
	dlg.ShowModal();
#endif
}

void GUI_IMPEXP mrpt::gui::show_mrpt_about_box_Qt(const std::string& appName,
	const std::string& additionalInfo, const bool showStandardInfo)
{
#if MRPT_HAS_Qt5
	CAboutBoxQt dlg(appName, additionalInfo, showStandardInfo);
	dlg.exec();
#endif
}
