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

#include "_DSceneViewerApp.h"

//(*AppHeaders
#include "_DSceneViewerMain.h"
#include <wx/image.h>
//*)
#include <wx/stdpaths.h>

IMPLEMENT_APP(_DSceneViewerApp)

// The file to open (from cmd line), or an empty string
std::string     global_fileToOpen;

#include <mrpt/opengl.h>
#include <mrpt/base.h>
#include <mrpt/gui/WxUtils.h>
using namespace mrpt;
using namespace mrpt::utils;

// The configuration file:
CConfigFile      *iniFile=NULL;



bool _DSceneViewerApp::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC,wxString(wxT("C")));


    // Process cmd line arguments (for the case of opening a file):
    if (argc>1)
            global_fileToOpen = wxString(wxApp::argv[1]).mb_str();

    // Create the INI file:
    wxStandardPaths stdPaths;
    wxString    dataDir = stdPaths.GetUserDataDir();
    std::string dataDirStr( dataDir.mb_str() );
    mrpt::system::createDirectory( dataDirStr ); // Create dir!
    std::string iniFileName( dataDirStr + std::string("/config.cfg") );
    iniFile = new CConfigFile(iniFileName);


    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
    _DSceneViewerFrame* Frame = new _DSceneViewerFrame(0);
    Frame->Show();
    SetTopWindow(Frame);
    }
    //*)
    return wxsOK;

}

int _DSceneViewerApp::OnExit()
{
    delete iniFile;
    iniFile = NULL;

    return 0;
}

