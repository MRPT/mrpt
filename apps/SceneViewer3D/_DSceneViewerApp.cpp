/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "_DSceneViewerApp.h"

//(*AppHeaders
#include <wx/image.h>
#include "_DSceneViewerMain.h"
//*)
#include <wx/cmdline.h>
#include <wx/stdpaths.h>

IMPLEMENT_APP(_DSceneViewerApp)

// The file to open (from cmd line), or an empty string
std::string global_fileToOpen;

#include <mrpt/config/CConfigFile.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::config;

// The configuration file:
std::unique_ptr<CConfigFile> iniFile;

bool _DSceneViewerApp::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a
	//  perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC, wxString(wxT("C")));

	// Process cmd line arguments:
	static const wxCmdLineEntryDesc cmdLineDesc[] = {
		{wxCMD_LINE_OPTION, wxT_2("l"), wxT_2("load"), wxT_2("load a library"),
		 wxCMD_LINE_VAL_STRING, 0},
		{wxCMD_LINE_PARAM, nullptr, nullptr, wxT_2("Input File"),
		 wxCMD_LINE_VAL_STRING, wxCMD_LINE_PARAM_OPTIONAL},
		{wxCMD_LINE_NONE, nullptr, nullptr, nullptr, wxCMD_LINE_VAL_NONE, 0}};

	wxCmdLineParser parser(cmdLineDesc, argc, argv);
	parser.Parse(true);
	wxString libraryPath;
	if (parser.Found(wxT_2("l"), &libraryPath))
	{
		const std::string sLib = std::string(libraryPath.mb_str());
		std::cout << "Loading plugin libraries: " << sLib << "...\n";
		mrpt::system::loadPluginModules(sLib);
	}
	if (parser.GetParamCount() == 1)
		global_fileToOpen = parser.GetParam().mb_str();

	// Create the INI file:
	// wxStandardPaths stdPaths;
	// wxString    dataDir = stdPaths.GetUserDataDir();
	wxString dataDir = wxStandardPaths::Get().GetUserDataDir();
	std::string dataDirStr(dataDir.mb_str());
	mrpt::system::createDirectory(dataDirStr);  // Create dir!
	std::string iniFileName(dataDirStr + std::string("/config.cfg"));
	iniFile = std::make_unique<CConfigFile>(iniFileName);

	//(*AppInitialize
	bool wxsOK = true;
	wxInitAllImageHandlers();
	if (wxsOK)
	{
		auto* Frame = new _DSceneViewerFrame(nullptr);
		Frame->Show();
		SetTopWindow(Frame);
	}
	//*)
	return wxsOK;
}

int _DSceneViewerApp::OnExit() { return 0; }
