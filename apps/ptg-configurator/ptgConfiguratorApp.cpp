/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ptgConfiguratorApp.h"

//(*AppHeaders
#include "ptgConfiguratorMain.h"
#include <wx/image.h>
//*)

IMPLEMENT_APP(ptgConfiguratorApp)

bool ptgConfiguratorApp::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC,wxString(wxT("C")));

    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
    	ptgConfiguratorframe* Frame = new ptgConfiguratorframe(0);
    	Frame->Show();
    	SetTopWindow(Frame);
    }
    //*)
    return wxsOK;

}
