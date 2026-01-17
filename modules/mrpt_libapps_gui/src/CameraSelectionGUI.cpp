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

#include <mrpt/apps/CameraSelectionGUI.h>
#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/hwdrivers/CCameraSensor.h>

/* ------------------------------------------------------------------------
            prepareVideoSourceFromUserSelection
   ------------------------------------------------------------------------ */
CCameraSensor::Ptr mrpt::hwdrivers::prepareVideoSourceFromUserSelection()
{
#if MRPT_HAS_WXWIDGETS
  // Create the main wxThread, if it doesn't exist yet:
  if (!mrpt::gui::WxSubsystem::createOneInstanceMainThread())
  {
    std::cerr << "[mrpt::hwdrivers::prepareVideoSourceFromUserSelection] "
                 "Error initiating Wx subsystem."
              << "\n";
    return CCameraSensor::Ptr();  // Error!
  }

  std::promise<void> semDlg;
  std::promise<mrpt::gui::detail::TReturnAskUserOpenCamera> dlgSelection;

  // Create window:
  auto* REQ = new WxSubsystem::TRequestToWxMainThread[1];
  REQ->OPCODE = 700;
  REQ->sourceCameraSelectDialog = true;
  REQ->voidPtr = reinterpret_cast<void*>(&semDlg);
  REQ->voidPtr2 = reinterpret_cast<void*>(&dlgSelection);
  WxSubsystem::pushPendingWxRequest(REQ);

  // Wait for the window to realize and signal it's alive:
  if (!WxSubsystem::isConsoleApp())
  {
    std::this_thread::sleep_for(20ms);  // Force at least 1-2 timer ticks for processing the event:
    wxApp::GetInstance()->Yield(true);
  }

  // wait for window construction:
  int maxTimeout =
#ifdef _DEBUG
      30000;
#else
      6000;
#endif
  // If we have an "MRPT_WXSUBSYS_TIMEOUT_MS" environment variable, use that
  // timeout instead:
  const char* envVal = getenv("MRPT_WXSUBSYS_TIMEOUT_MS");
  if (envVal) maxTimeout = atoi(envVal);

  if (semDlg.get_future().wait_for(std::chrono::milliseconds(maxTimeout)) ==
      std::future_status::timeout)
  {
    cerr << "[prepareVideoSourceFromUserSelection] Timeout waiting window "
            "creation."
         << "\n";
    return CCameraSensor::Ptr();
  }

  // wait for user selection:
  auto future = dlgSelection.get_future();
  future.wait();
  const auto& ret = future.get();

  // If the user didn't accept the dialog, return now:
  if (!ret.accepted_by_user) return CCameraSensor::Ptr();

  mrpt::config::CConfigFileMemory selectedConfig(ret.selectedConfig);

  CCameraSensor::Ptr cam = std::make_shared<CCameraSensor>();
  cam->loadConfig(selectedConfig, "CONFIG");
  cam->initialize();  // This will raise an exception if necessary

  return cam;
#else
  THROW_EXCEPTION("MRPT compiled without wxWidgets");
#endif  // MRPT_HAS_WXWIDGETS
}

/* ------------------------------------------------------------------------
            prepareVideoSourceFromPanel
   ------------------------------------------------------------------------ */
CCameraSensor::Ptr mrpt::hwdrivers::prepareVideoSourceFromPanel(void* _panel)
{
#if MRPT_HAS_WXWIDGETS

  try
  {
    CConfigFileMemory cfg;
    writeConfigFromVideoSourcePanel(_panel, "CONFIG", &cfg);

    // Try to open the camera:
    CCameraSensor::Ptr video = std::make_shared<CCameraSensor>();
    video->loadConfig(cfg, "CONFIG");

    // This will raise an exception if necessary
    video->initialize();

    return video;
  }
  catch (const std::exception& e)
  {
    cerr << endl << e.what() << "\n";
    wxMessageBox(_("Couldn't open video source"), _("Error"));
    return CCameraSensor::Ptr();
  }
#else
  THROW_EXCEPTION("MRPT compiled without wxWidgets");
#endif  // MRPT_HAS_WXWIDGETS
}

/* ------------------------------------------------------------------------
            writeConfigFromVideoSourcePanel
   ------------------------------------------------------------------------ */
void mrpt::hwdrivers::writeConfigFromVideoSourcePanel(
    void* _panel, const std::string& sect, mrpt::config::CConfigFileBase* cfg)
{
  MRPT_START
#if MRPT_HAS_WXWIDGETS
  ASSERT_(_panel);
  auto* panel = reinterpret_cast<mrpt::gui::CPanelCameraSelection*>(_panel);
  ASSERTMSG_(panel, "panel must be of type mrpt::gui::CPanelCameraSelection *");
  panel->writeConfigFromVideoSourcePanel(sect, cfg);

#else
  THROW_EXCEPTION("MRPT compiled without wxWidgets");
#endif  // MRPT_HAS_WXWIDGETS
  MRPT_END
}

/* ------------------------------------------------------------------------
            readConfigIntoVideoSourcePanel
   ------------------------------------------------------------------------ */
void mrpt::hwdrivers::readConfigIntoVideoSourcePanel(
    void* _panel, const std::string& sect, const mrpt::config::CConfigFileBase* cfg)
{
  MRPT_START
#if MRPT_HAS_WXWIDGETS
  ASSERT_(_panel);
  auto* panel = reinterpret_cast<mrpt::gui::CPanelCameraSelection*>(_panel);
  ASSERTMSG_(panel, "panel must be of type mrpt::gui::CPanelCameraSelection *");

  panel->readConfigIntoVideoSourcePanel(sect, cfg);

#else
  THROW_EXCEPTION("MRPT compiled without wxWidgets");
#endif  // MRPT_HAS_WXWIDGETS
  MRPT_END
}