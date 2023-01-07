/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers
//
#include <mrpt/core/exceptions.h>
#include <mrpt/gui/about_box.h>

#if MRPT_HAS_Qt5
#include "CAboutBoxQt.h"
#endif

void mrpt::gui::show_mrpt_about_box_Qt(
	const std::string& appName, const std::string& additionalInfo,
	const bool showStandardInfo)
{
#if MRPT_HAS_Qt5
	CAboutBoxQt dlg(appName, additionalInfo, showStandardInfo);
	dlg.exec();
#else
	THROW_EXCEPTION("MRPT compiled without Qt support");
#endif
}
