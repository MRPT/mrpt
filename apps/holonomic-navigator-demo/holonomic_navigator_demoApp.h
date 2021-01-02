/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef HOLONOMIC_NAVIGATOR_DEMOAPP_H
#define HOLONOMIC_NAVIGATOR_DEMOAPP_H

#include <wx/app.h>

class holonomic_navigator_demoApp : public wxApp
{
   public:
	bool OnInit() override;
};

#endif  // HOLONOMIC_NAVIGATOR_DEMOAPP_H
