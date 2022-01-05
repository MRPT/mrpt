/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef XRAWLOGVIEWERAPP_H
#define XRAWLOGVIEWERAPP_H

#include <wx/app.h>

class xRawLogViewerApp : public wxApp
{
   public:
	bool OnInit() override;
	int OnExit() override;
};

#endif	// XRAWLOGVIEWERAPP_H
