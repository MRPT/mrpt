/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*
  App      : kinect-calibrate
  Web page : https://www.mrpt.org/Kinect_and_MRPT

  Usage    : Run and follow on-screen instructions
*/

#include "kinect_calibrate_guiApp.h"

//(*AppHeaders
#include <wx/image.h>
#include "kinect_calibrate_guiMain.h"
//*)

IMPLEMENT_APP(kinect_calibrate_guiApp)

bool kinect_calibrate_guiApp::OnInit()
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
		kinect_calibrate_guiDialog Dlg(nullptr);
		SetTopWindow(&Dlg);
		Dlg.ShowModal();
		wxsOK = false;
	}
	//*)
	return wxsOK;
}
