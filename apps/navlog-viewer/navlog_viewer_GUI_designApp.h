/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef NAVLOG_VIEWER_GUI_DESIGNAPP_H
#define NAVLOG_VIEWER_GUI_DESIGNAPP_H

#include <wx/app.h>

class navlog_viewer_GUI_designApp : public wxApp
{
   public:
	bool OnInit() override;
	int OnRun() override;
};

#endif  // NAVLOG_VIEWER_GUI_DESIGNAPP_H
