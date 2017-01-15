/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slamdemoApp.h"

//(*AppHeaders
#include "slamdemoMain.h"
#include <wx/image.h>
//*)
#include <wx/msgdlg.h>

IMPLEMENT_APP(slamdemoApp)

#include <mrpt/otherlibs/tclap/CmdLine.h>
#include <mrpt/system/os.h>
#include <mrpt/system/filesystem.h>

using namespace std;


bool slamdemoApp::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC,wxString(wxT("C")));

    wxInitAllImageHandlers();

	// Process command line args?
	bool gotoGUI = true;

	win = new slamdemoFrame(NULL);

	if (argc>1)
		gotoGUI = doCommandLineProcess();

	if (gotoGUI)
	{
		// Show normal GUI:
		win->Show();
		SetTopWindow(win);
		return true;
	}
	else
	{
		delete win;
		win=NULL;
		return false; // Exit now.
	}
}

// Auxiliary class to allow text output in Win32 (in Linux the console keeps working for GUIs).
#ifdef MRPT_OS_WINDOWS

std::ostringstream  out_cmdLine;

class CMyCmdLineOut : public TCLAP::StdOutput
{
public:
	CMyCmdLineOut() : StdOutput(out_cmdLine)
	{
	}

};

#endif


// Executes the program in "batch" mode, where the user passes some command-line args
bool slamdemoApp::doCommandLineProcess()
{
	// Declare the supported options.
	TCLAP::CmdLine cmd("2d-slam-demo", ' ', mrpt::system::MRPT_getVersion().c_str());

	TCLAP::ValueArg<std::string> arg_cfgFil("c","config","Config file to load",false,"","params.ini",cmd);
	TCLAP::SwitchArg arg_nogui("n","nogui","Don't stay in the GUI, exit after the experiment.",cmd, false);
	TCLAP::SwitchArg arg_norun("r","norun","Just load the config file, don't run it.",cmd, false);

#ifdef MRPT_OS_WINDOWS
	CMyCmdLineOut  out_buf;
	cmd.setOutput(&out_buf);
#endif

	vector<char*>  auxArgs(argc);
	for (int i=0;i<argc;i++)
	{
		wxString  s(argv[i]);
		auxArgs[i]=new char[s.size()+10];
		strcpy(auxArgs[i],s.ToUTF8().data());
	}

	// Parse arguments:
	bool res_parse = cmd.parse( argc, &auxArgs[0] );

	// Free aux mem:
	for (int i=0;i<argc;i++)
		delete[] auxArgs[i];

	if (!res_parse)
	{
#ifdef MRPT_OS_WINDOWS
		if (!out_cmdLine.str().empty())
			wxMessageBox(_U(out_cmdLine.str().c_str()),_("2d-slam-demo"));
#endif
		return false;
	}

	const std::string cfgFil = arg_cfgFil.getValue();

	if ( !mrpt::system::fileExists(cfgFil) )
	{
		cerr << "The indicated config file does not exist: " << cfgFil << endl;
#ifdef MRPT_OS_WINDOWS
		wxMessageBox(wxT("The indicated config file does not exist"),_("2d-slam-demo"));
#endif
		return false;
	}
	else
	{
		m_option_norun = arg_norun.getValue();

		if (!arg_nogui.getValue())
		{
			win->Show();
			SetTopWindow(win);
		}

		try
		{
			DoBatchExperiments(cfgFil);
		}
		catch (std::exception &e)
		{
			cerr << e.what() << endl;
#ifdef MRPT_OS_WINDOWS
			wxMessageBox(_U(e.what()),_("2d-slam-demo"));
#endif
		}
	}

	// return false to exit the program now and NOT proceed with the GUI.
	// return true to continue with the program:
	return !arg_nogui.getValue();
}
