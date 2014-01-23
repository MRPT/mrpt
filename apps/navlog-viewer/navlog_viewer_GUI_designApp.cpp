/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include "navlog_viewer_GUI_designApp.h"

//(*AppHeaders
#include "navlog_viewer_GUI_designMain.h"
#include <wx/image.h>
//*)

#include <mrpt/gui/WxUtils.h>

IMPLEMENT_APP(navlog_viewer_GUI_designApp)

std::string global_fileToOpen;

bool navlog_viewer_GUI_designApp::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC,wxString(wxT("C")));

    // Process cmd line arguments (for the case of opening a file):
    if (argc>1)
        global_fileToOpen = wxString(wxApp::argv[1]).mb_str();

    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
	navlog_viewer_GUI_designDialog* Frame = new navlog_viewer_GUI_designDialog (NULL);
	Frame->Show();
	SetTopWindow(Frame);
    wxsOK = true;
    }
    //*)
    return wxsOK;

}


int navlog_viewer_GUI_designApp::OnRun()
{
	try
	{
		return wxApp::OnRun();
	}
	catch (std::exception &e)
	{
		::wxMessageBox(_U(e.what()),_("Exception:"));
		return -1;
	}
}
