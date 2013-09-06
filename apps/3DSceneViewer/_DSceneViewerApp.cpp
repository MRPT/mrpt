/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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

