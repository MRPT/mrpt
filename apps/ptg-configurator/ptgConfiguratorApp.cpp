/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ptgConfiguratorApp.h"

//(*AppHeaders
#include "ptgConfiguratorMain.h"
#include <wx/image.h>
//*)
#include <wx/cmdline.h>
#ifdef MRPT_OS_LINUX
#include <dlfcn.h>
#endif

IMPLEMENT_APP(ptgConfiguratorApp)

bool ptgConfiguratorApp::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC,wxString(wxT("C")));
	static const wxCmdLineEntryDesc cmdLineDesc[] =
	{
#ifdef MRPT_OS_LINUX
		{wxCMD_LINE_OPTION, wxT_2("l"), wxT_2("load"), wxT_2("load a library"), wxCMD_LINE_VAL_STRING, 0},
#endif
		{wxCMD_LINE_NONE, nullptr, nullptr, nullptr, wxCMD_LINE_VAL_NONE, 0}
	};

	wxCmdLineParser parser(cmdLineDesc, argc, argv);
	parser.Parse(true);
#ifdef MRPT_OS_LINUX
        wxString libraryPath;
        if(parser.Found(wxT_2("l"), &libraryPath))
		dlopen(libraryPath.mb_str(), RTLD_LAZY);
#endif

	//(*AppInitialize
	bool wxsOK = true;
	wxInitAllImageHandlers();
	if ( wxsOK )
	{
		ptgConfiguratorframe* Frame = new ptgConfiguratorframe(0);
		Frame->Show();
		SetTopWindow(Frame);
	}
	//*)
	return wxsOK;

}
