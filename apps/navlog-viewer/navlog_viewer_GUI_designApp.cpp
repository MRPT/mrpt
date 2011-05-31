/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
