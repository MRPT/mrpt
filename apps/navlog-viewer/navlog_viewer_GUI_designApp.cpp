/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include "navlog_viewer_GUI_designApp.h"

//(*AppHeaders
#include "navlog_viewer_GUI_designMain.h"
#include <wx/image.h>
//*)
#include <wx/cmdline.h>
#ifdef MRPT_OS_LINUX
#include <dlfcn.h>
#endif
#include <mrpt/gui/WxUtils.h>

IMPLEMENT_APP(navlog_viewer_GUI_designApp)

std::string global_fileToOpen;

bool navlog_viewer_GUI_designApp::OnInit()
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
		{wxCMD_LINE_PARAM, nullptr, nullptr, wxT_2("Input File"), wxCMD_LINE_VAL_STRING, wxCMD_LINE_PARAM_OPTIONAL},
		{wxCMD_LINE_NONE, nullptr, nullptr, nullptr, wxCMD_LINE_VAL_NONE, 0}
	};

	wxCmdLineParser parser(cmdLineDesc, argc, argv);
	parser.Parse(true);
#ifdef MRPT_OS_LINUX
        wxString libraryPath;
        if(parser.Found(wxT_2("l"), &libraryPath))
		dlopen(libraryPath.mb_str(), RTLD_LAZY);
#endif
	if(parser.GetParamCount() == 1)
		global_fileToOpen = parser.GetParam().mb_str();


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
