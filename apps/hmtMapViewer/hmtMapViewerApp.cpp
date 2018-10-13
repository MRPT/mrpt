/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "hmtMapViewerApp.h"

//(*AppHeaders
#include "hmtMapViewerMain.h"
#include <wx/image.h>
//*)
#include <wx/stdpaths.h>
#include <wx/log.h>

IMPLEMENT_APP(hmtMapViewerApp)

// The file to open (from cmd line), or an empty string
std::string global_fileToOpen;

#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/hmtslam/CHMTSLAM.h>
#include <mrpt/hmtslam/CRobotPosesGraph.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::config;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace std;

// The configuration file:
CConfigFile* iniFile = nullptr;

bool hmtMapViewerApp::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a
	//  perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC, wxString(wxT("C")));

	// Process cmd line arguments (for the case of opening a file):
	if (argc > 1) global_fileToOpen = wxString(wxApp::argv[1]).mb_str();

	// Create the INI file:
	wxString dataDir = wxStandardPaths::Get().GetUserDataDir();
	std::string dataDirStr(dataDir.mb_str());
	createDirectory(dataDirStr);  // Create dir!
	std::string iniFileName(dataDirStr + std::string("/config.cfg"));
	iniFile = new CConfigFile(iniFileName);

	//(*AppInitialize
	bool wxsOK = true;
	wxInitAllImageHandlers();
	if (wxsOK)
	{
		auto* Frame = new hmtMapViewerFrame(nullptr);
		Frame->Show();
		SetTopWindow(Frame);
	}
	//*)
	return wxsOK;
}

int hmtMapViewerApp::OnExit()
{
	delete iniFile;
	iniFile = nullptr;
	return 0;
}
