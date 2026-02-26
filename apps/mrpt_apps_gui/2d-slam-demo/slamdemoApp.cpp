/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include "slamdemoApp.h"

//(*AppHeaders
#include <wx/image.h>

#include "slamdemoMain.h"
//*)
#include <wx/msgdlg.h>

IMPLEMENT_APP(slamdemoApp)

#include <mrpt/core/config.h>  // MRPT_OS_*()
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>

using namespace std;

bool slamdemoApp::OnInit()
{
  // Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
  //  if we want numbers to use "." in all countries. The App::OnInit() is a
  //  perfect place to undo
  //  the default wxWidgets settings. (JL @ Sep-2009)
  wxSetlocale(LC_NUMERIC, wxString(wxT("C")));

  wxInitAllImageHandlers();

  // Process command line args?
  bool gotoGUI = true;

  win = new slamdemoFrame(nullptr);

  if (argc > 1) gotoGUI = doCommandLineProcess();

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
    win = nullptr;
    return false;  // Exit now.
  }
}

// Executes the program in "batch" mode, where the user passes some command-line
// args
bool slamdemoApp::doCommandLineProcess()
{
  try
  {
    // Declare the supported options.
    CLI::App app("2d-slam-demo");
    app.set_version_flag("--version", mrpt::system::MRPT_getVersion());

    std::string cfgFil;
    app.add_option("-c,--config", cfgFil, "Config file to load");

    bool nogui = false;
    app.add_flag("-n,--nogui", nogui, "Don't stay in the GUI, exit after the experiment.");

    bool norun = false;
    app.add_flag("-r,--norun", norun, "Just load the config file, don't run it.");

    // Parse arguments:
    CLI11_PARSE(app, argc, argv);

    if (cfgFil.empty())
    {
      // No config file specified, continue with GUI
      return true;
    }

    if (!mrpt::system::fileExists(cfgFil))
    {
      cerr << "The indicated config file does not exist: " << cfgFil << endl;
#ifdef MRPT_OS_WINDOWS
      wxMessageBox(wxT("The indicated config file does not exist"), _("2d-slam-demo"));
#endif
      return false;
    }
    else
    {
      m_option_norun = norun;

      if (!nogui)
      {
        win->Show();
        SetTopWindow(win);
      }

      try
      {
        DoBatchExperiments(cfgFil);
      }
      catch (const std::exception& e)
      {
        cerr << mrpt::exception_to_str(e) << endl;
#ifdef MRPT_OS_WINDOWS
        wxMessageBox(mrpt::exception_to_str(e), _("2d-slam-demo"));
#endif
      }
    }

    // return false to exit the program now and NOT proceed with the GUI.
    // return true to continue with the program:
    return !nogui;
  }
  catch (const std::exception& e)
  {
    cerr << "Error parsing command line: " << mrpt::exception_to_str(e) << endl;
#ifdef MRPT_OS_WINDOWS
    wxMessageBox(
        wxString("Error parsing command line: ") + mrpt::exception_to_str(e), _("2d-slam-demo"));
#endif
    return false;
  }
}
