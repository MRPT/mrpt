/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "gridmapSimulApp.h"

//(*AppHeaders
#include <wx/image.h>

#include "gridmapSimulMain.h"
//*)
#include <wx/log.h>
#include <wx/stdpaths.h>

IMPLEMENT_APP(gridmapSimulApp)

bool gridmapSimulApp::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a
	//  perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC, wxString(wxT("C")));

	/*    // Process cmd line arguments (for the case of opening a file):
		if (argc>1)
			global_fileToOpen = wxString(wxApp::argv[1]).mb_str();
	*/

	//(*AppInitialize
	bool wxsOK = true;
	wxInitAllImageHandlers();
	if (wxsOK)
	{
		auto* Frame = new gridmapSimulFrame(nullptr);
		Frame->Show();
		SetTopWindow(Frame);
	}
	//*)
	return wxsOK;
}

int gridmapSimulApp::OnExit() { return 0; }
