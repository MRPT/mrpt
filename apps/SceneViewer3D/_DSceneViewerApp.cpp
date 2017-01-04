/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
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

#include <mrpt/gui/WxUtils.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/filesystem.h>
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
    // wxStandardPaths stdPaths;
    // wxString    dataDir = stdPaths.GetUserDataDir();
	wxString    dataDir = wxStandardPaths::Get().GetUserDataDir();
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

