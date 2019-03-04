/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef _DSCENEVIEWERAPP_H
#define _DSCENEVIEWERAPP_H

#include <wx/app.h>

class _DSceneViewerApp : public wxApp
{
   public:
	bool OnInit() override;
	int OnExit() override;
};

#endif  // _DSCENEVIEWERAPP_H
