/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef gridmapSimulAPP_H
#define gridmapSimulAPP_H

#include <wx/app.h>

class gridmapSimulApp : public wxApp
{
   public:
	bool OnInit() override;
	int OnExit() override;
};

#endif  // gridmapSimulAPP_H
