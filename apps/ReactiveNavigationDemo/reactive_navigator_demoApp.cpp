/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "reactive_navigator_demoApp.h"

//(*AppHeaders
#include <wx/image.h>

#include "reactive_navigator_demoMain.h"
//*)

IMPLEMENT_APP(reactive_navigator_demoApp)

bool reactive_navigator_demoApp::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a
	//  perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC, wxString(wxT("C")));

	//(*AppInitialize
	bool wxsOK = true;
	wxInitAllImageHandlers();
	if (wxsOK)
	{
		auto* Frame = new reactive_navigator_demoframe(nullptr);
		Frame->Show();
		SetTopWindow(Frame);
	}
	//*)
	return wxsOK;
}
