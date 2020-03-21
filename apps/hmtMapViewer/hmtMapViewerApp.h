/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef HMTMAPVIEWERAPP_H
#define HMTMAPVIEWERAPP_H

#include <wx/app.h>

class hmtMapViewerApp : public wxApp
{
   public:
	bool OnInit() override;
	int OnExit() override;
};

#endif  // HMTMAPVIEWERAPP_H
