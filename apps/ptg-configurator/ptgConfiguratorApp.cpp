/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
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
		{wxCMD_LINE_OPTION, "l", "load", "load a library",
		 wxCMD_LINE_VAL_STRING, 0},
#endif
		{wxCMD_LINE_OPTION, "i", "ini", "INI file to load",
		 wxCMD_LINE_VAL_STRING, 0},
		{wxCMD_LINE_OPTION, "s", "ini-section",
		 "INI file section to load (default=\"PTG_PARAMS\")",
		 wxCMD_LINE_VAL_STRING, 0},
		{wxCMD_LINE_NONE, nullptr, nullptr, nullptr, wxCMD_LINE_VAL_NONE, 0}};

	wxCmdLineParser parser(cmdLineDesc, argc, argv);
	parser.Parse(true);
	wxString libraryPath;
	if (parser.Found("l", &libraryPath))
		mrpt::system::loadPluginModules(libraryPath.ToStdString());

	bool wxsOK = true;
	wxInitAllImageHandlers();
	if (wxsOK)
	{
		auto* Frame = new ptgConfiguratorframe(nullptr);
		Frame->Show();
		SetTopWindow(Frame);

		wxString iniFilePath;
		if (parser.Found("i", &iniFilePath))
		{
			Frame->getCfgBox()->LoadFile(iniFilePath);
			Frame->m_disableLoadDefaultParams = true;
		}

		wxString iniSection;
		if (parser.Found("s", &iniSection))
			Frame->m_cfgFileSection = iniSection.ToStdString();
	}

	return wxsOK;
}
