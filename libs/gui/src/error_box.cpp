/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */

#include "gui-precomp.h"  // Precompiled headers

#include <mrpt/gui/error_box.h>
#if MRPT_HAS_Qt5
#include <QErrorMessage>
#include <QString>
#else
#include <iostream>
#endif  // MRPT_HAS_Qt5
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
#if MRPT_HAS_Qt5
	QErrorMessage msg;
	msg.showMessage(QString::fromStdString(str));
	msg.exec();
#elif MRPT_HAS_WXWIDGETS
	wxMessageBox(str.c_str(), _("Exception"));
#else
	std::cerr << str << std::endl;
#endif  // MRPT_HAS_Qt5
}
