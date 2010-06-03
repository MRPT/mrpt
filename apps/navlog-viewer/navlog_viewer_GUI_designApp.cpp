/***************************************************************
 * Name:      navlog_viewer_GUI_designApp.cpp
 * Purpose:   Code for Application Class
 * Author:    J.L. Blanco (jlblanco@ctima.uma.es)
 * Created:   2010-02-03
 * Copyright: J.L. Blanco (http://www.isa.uma.es/jlblanco)
 * License:
 **************************************************************/

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
