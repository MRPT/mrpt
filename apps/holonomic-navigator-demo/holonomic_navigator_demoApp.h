/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
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
