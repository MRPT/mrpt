/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gui-precomp.h"  // Precompiled headers
//
#include <mrpt/gui/error_box.h>
#include <mrpt/system/string_utils.h>  // firstNLines()

#if MRPT_HAS_Qt5
#include <QErrorMessage>
#include <QString>
#else
#include <iostream>
#endif	// MRPT_HAS_Qt5
#if MRPT_HAS_WXWIDGETS
#include <mrpt/gui/WxUtils.h>
#endif

void mrpt::gui::tryCatch(
	const std::function<void()>& tryPart, const std::string& catchMessage)
{
	try
	{
		tryPart();
	}
	catch (const std::exception& e)
	{
		showErrorMessage(catchMessage + e.what());
	}
	catch (...)
	{
		showErrorMessage("Untyped exception!");
	}
}

void mrpt::gui::showErrorMessage(const std::string& str)
{
	const size_t maxLines = 7;
	const std::string sErr = mrpt::system::firstNLines(str, maxLines);

#if MRPT_HAS_Qt5
	QErrorMessage msg;
	msg.showMessage(QString::fromStdString(sErr));
	msg.exec();
#elif MRPT_HAS_WXWIDGETS
	wxMessageBox(sErr.c_str(), _("Exception"));
#else
	std::cerr << str << std::endl;
#endif	// MRPT_HAS_Qt5
}
