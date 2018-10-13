/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "xRawLogViewerApp.h"
#include <wx/stdpaths.h>

//(*AppHeaders
#include "xRawLogViewerMain.h"
#include <wx/image.h>
//*)
#include <wx/log.h>

#include <clocale>

IMPLEMENT_APP(xRawLogViewerApp)

// The file to open (from cmd line), or an empty string
std::string global_fileToOpen;

#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/filesystem.h>

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::config;
using namespace std;

// The configuration file:
CConfigFile* iniFile = nullptr;

bool xRawLogViewerApp::OnInit()
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
	mrpt::system::createDirectory(dataDirStr);  // Create dir!
	std::string iniFileName(dataDirStr + std::string("/config.cfg"));
	iniFile = new CConfigFile(iniFileName);

	// Set numeric locale to "POSIX" to enable a consistent file generation in
	// all platforms:
	// setlocale(LC_NUMERIC,"C");

	//(*AppInitialize
	bool wxsOK = true;
	wxInitAllImageHandlers();
	if (wxsOK)
	{
		auto* Frame = new xRawLogViewerFrame(nullptr);
		Frame->Show();
		SetTopWindow(Frame);
	}
	//*)
	return wxsOK;
}

int xRawLogViewerApp::OnExit()
{
	delete iniFile;
	iniFile = nullptr;

	return 0;
}
