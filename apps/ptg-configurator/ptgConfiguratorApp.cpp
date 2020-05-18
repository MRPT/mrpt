/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "ptgConfiguratorApp.h"

//(*AppHeaders
#include <wx/image.h>
#include "ptgConfiguratorMain.h"
//*)
#include <mrpt/system/os.h>
#include <wx/cmdline.h>

IMPLEMENT_APP(ptgConfiguratorApp)

bool ptgConfiguratorApp::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a
	//  perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC, wxString(wxT("C")));
	static const wxCmdLineEntryDesc cmdLineDesc[] = {
#ifdef MRPT_OS_LINUX
		{wxCMD_LINE_OPTION, wxT_2("l"), wxT_2("load"), wxT_2("load a library"),
		 wxCMD_LINE_VAL_STRING, 0},
#endif
		{wxCMD_LINE_NONE, nullptr, nullptr, nullptr, wxCMD_LINE_VAL_NONE, 0}};

	wxCmdLineParser parser(cmdLineDesc, argc, argv);
	parser.Parse(true);
	wxString libraryPath;
	if (parser.Found(wxT_2("l"), &libraryPath))
	{
		const std::string sLib = std::string(libraryPath.mb_str());
		mrpt::system::loadPluginModules(sLib);
	}

	//(*AppInitialize
	bool wxsOK = true;
	wxInitAllImageHandlers();
	if (wxsOK)
	{
		auto* Frame = new ptgConfiguratorframe(nullptr);
		Frame->Show();
		SetTopWindow(Frame);
	}
	//*)
	return wxsOK;
}
