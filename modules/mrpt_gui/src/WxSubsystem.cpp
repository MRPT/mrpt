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

#include <mrpt/core/get_env.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/gui/WxUtils.h>
#include <mrpt/gui/config.h>
#include <mrpt/system/os.h>
#include <mrpt/system/thread_name.h>

#include <queue>

namespace
{
const auto WX_SUBSYSTEM_VERBOSE = mrpt::get_env<bool>("MRPT_WX_SUBSYSTEM_VERBOSE", false);
}

// ------------------------------------------------------------------------
// Defined: Try to wait for all windows & the thread to exit cleanly.
// Undefined: Just do a std::this_thread::sleep_for(ms) and quit crossing our
// fingers.
//
//  Problem with the "clean way" is: As of feb/2011, I get this error
//   at the end:
//  ** (MRPT:11711): CRITICAL **: giop_thread_request_push: assertion `tdata !=
//  NULL' failed
// ------------------------------------------------------------------------
//#define WXSHUTDOWN_DO_IT_CLEAN

#if MRPT_HAS_WXWIDGETS

using namespace mrpt;
using namespace mrpt::gui;
using namespace std;

// Convenience alias so switch cases stay readable.
using OpCode = WxSubsystem::OpCode;

namespace
{
bool isConsoleApp_value = true;
}

// ---------------------------------------------------------------------------
// Module-local singleton holding mutable global state.
// ---------------------------------------------------------------------------
class WxSubSystemGlobalData
{
 public:
  static WxSubSystemGlobalData& Instance()
  {
    static WxSubSystemGlobalData d;
    return d;
  }

  int windowCount{0};
  std::mutex cs_windowCount;

  std::queue<WxSubsystem::TRequestToWxMainThread*> listPendingWxRequests;
  std::mutex cs_listPendingWxRequests;

 private:
  WxSubSystemGlobalData() = default;
};

volatile WxSubsystem::CWXMainFrame* WxSubsystem::CWXMainFrame::oneInstance = nullptr;

bool WxSubsystem::isConsoleApp() { return isConsoleApp_value; }
WxSubsystem::CAuxWxSubsystemShutdowner WxSubsystem::global_wxsubsystem_shutdown;

// ---------------------------------------------------------------------------
// CAuxWxSubsystemShutdowner
// ---------------------------------------------------------------------------
WxSubsystem::CAuxWxSubsystemShutdowner::CAuxWxSubsystemShutdowner() = default;

WxSubsystem::CAuxWxSubsystemShutdowner::~CAuxWxSubsystemShutdowner()
{
  if (WxSubsystem::isConsoleApp())
  {
    if (WX_SUBSYSTEM_VERBOSE)
    {
      printf("[~CAuxWxSubsystemShutdowner] Sending SHUTDOWN request...\n");
    }

    try
    {
      auto* REQ = new WxSubsystem::TRequestToWxMainThread;
      REQ->OPCODE = OpCode::SHUTDOWN;
      WxSubsystem::pushPendingWxRequest(REQ);

      WxSubsystem::waitWxShutdownsIfNoWindows();
    }
    catch (const std::exception& e)
    {
      // Swallow: may be an out-of-memory condition at program exit.
      std::cerr << "[WxSubsystem::CAuxWxSubsystemShutdowner::~CAuxWxSubsystemShutdowner()] Error "
                   "sending shutdown request: "
                << e.what() << "\n";
    }
  }  // is console app.

  if (WX_SUBSYSTEM_VERBOSE)
  {
    printf("[~CAuxWxSubsystemShutdowner] Deleting static objects.\n");
  }
}

// ---------------------------------------------------------------------------
// Auxiliary dialog: ask user to open a camera
// ---------------------------------------------------------------------------
class CDialogAskUserForCamera : public wxDialog
{
 public:
  mrpt::gui::CPanelCameraSelection* panel{nullptr};

  static const wxWindowID ID_BTN_OK;
  static const wxWindowID ID_BTN_CANCEL;

  CDialogAskUserForCamera() :
      wxDialog(
          nullptr,
          wxID_ANY,
          wxT("Select image source"),
          wxDefaultPosition,
          wxDefaultSize,
          wxDEFAULT_DIALOG_STYLE,
          wxDialogNameStr),
      panel(new mrpt::gui::CPanelCameraSelection(this, wxID_ANY))
  {
    auto* f1 = new wxFlexGridSizer(2, 1, 0, 0);

    f1->Add(panel, 1, wxALL | wxALIGN_BOTTOM | wxALIGN_CENTER_HORIZONTAL, 5);

    auto* f2 = new wxFlexGridSizer(1, 2, 0, 0);
    auto* btnOk = new wxButton(this, ID_BTN_OK, wxT("Ok"), wxDefaultPosition, wxDefaultSize);
    auto* btnCancel =
        new wxButton(this, ID_BTN_CANCEL, wxT("Cancel"), wxDefaultPosition, wxDefaultSize);
    f1->Add(f2, 1, wxALL | wxALIGN_BOTTOM | wxALIGN_CENTER_HORIZONTAL, 5);
    f2->Add(btnOk, 1, wxALL | wxALIGN_BOTTOM | wxALIGN_CENTER_HORIZONTAL, 5);
    f2->Add(btnCancel, 1, wxALL | wxALIGN_BOTTOM | wxALIGN_CENTER_HORIZONTAL, 5);

    Bind(wxEVT_BUTTON, &CDialogAskUserForCamera::OnBtnOk, this, ID_BTN_OK);
    Bind(wxEVT_BUTTON, &CDialogAskUserForCamera::OnBtnCancel, this, ID_BTN_CANCEL);

    SetSizer(f1);
    Fit();

    // Allow default params to be accepted by pressing ENTER.
    btnOk->SetFocus();
  }

  ~CDialogAskUserForCamera() override = default;
  void OnBtnOk(wxCommandEvent& /*event*/) { EndModal(wxID_OK); }
  void OnBtnCancel(wxCommandEvent& /*event*/) { EndModal(wxID_CANCEL); }
};

const wxWindowID CDialogAskUserForCamera::ID_BTN_OK = wxNewId();
const wxWindowID CDialogAskUserForCamera::ID_BTN_CANCEL = wxNewId();

// ---------------------------------------------------------------------------
// CWXMainFrame
// ---------------------------------------------------------------------------
BEGIN_EVENT_TABLE(WxSubsystem::CWXMainFrame, wxFrame)
END_EVENT_TABLE()

const auto ID_TIMER_WX_PROCESS_REQUESTS = wxNewId();

WxSubsystem::CWXMainFrame::CWXMainFrame(wxWindow* parent, wxWindowID id)
{
  Create(parent, id, _("MRPT-dummy frame window"), wxDefaultPosition, wxSize(1, 1), 0, _T("id"));

  if (oneInstance != nullptr)
  {
    cerr << "[CWXMainFrame] More than one instance running!\n";
  }
  oneInstance = this;

  Bind(wxEVT_TIMER, &CWXMainFrame::OnTimerProcessRequests, this, ID_TIMER_WX_PROCESS_REQUESTS);
  m_theTimer = new wxTimer(this, ID_TIMER_WX_PROCESS_REQUESTS);
  m_theTimer->Start(10, true);  // One-shot
}

WxSubsystem::CWXMainFrame::~CWXMainFrame()
{
  if (WX_SUBSYSTEM_VERBOSE)
  {
    cout << "[CWXMainFrame] Destructor.\n";
  }
  delete m_theTimer;
  m_theTimer = nullptr;
  oneInstance = nullptr;

  // Purge all pending requests.
  TRequestToWxMainThread* msg;
  while (nullptr != (msg = popPendingWxRequest()))
  {
    delete msg;
  }
}

int WxSubsystem::CWXMainFrame::notifyWindowCreation()
{
  auto& wxd = WxSubSystemGlobalData::Instance();
  std::lock_guard<std::mutex> lock(wxd.cs_windowCount);
  return ++wxd.windowCount;
}

int WxSubsystem::CWXMainFrame::notifyWindowDestruction()
{
  auto& wxd = WxSubSystemGlobalData::Instance();

  int ret = 0;
  {
    std::lock_guard<std::mutex> lock(wxd.cs_windowCount);
    ret = --wxd.windowCount;
  }

  if (ret == 0)
  {
    // Last window closed; request shutdown of the wx subsystem.
    if (oneInstance != nullptr)
    {
#ifdef WXSHUTDOWN_DO_IT_CLEAN
      // Cast away volatile: safe here because we are in the wx thread.
      auto* me = const_cast<CWXMainFrame*>(static_cast<const CWXMainFrame*>(oneInstance));
      me->Close();
#endif

      if (WX_SUBSYSTEM_VERBOSE)
      {
        cout << "[CWXMainFrame::notifyWindowDestruction] numWindows=0. "
                "me->Close() called.\n";
      }
    }
  }

  return ret;
}

// ---------------------------------------------------------------------------
// Request queue helpers
// ---------------------------------------------------------------------------
WxSubsystem::TRequestToWxMainThread* WxSubsystem::popPendingWxRequest()
{
  auto& wxd = WxSubSystemGlobalData::Instance();
  std::lock_guard<std::mutex> locker(wxd.cs_listPendingWxRequests);

  if (wxd.listPendingWxRequests.empty())
  {
    return nullptr;
  }

  TRequestToWxMainThread* ret = wxd.listPendingWxRequests.front();
  wxd.listPendingWxRequests.pop();
  return ret;
}

void WxSubsystem::pushPendingWxRequest(WxSubsystem::TRequestToWxMainThread* data)
{
  if (WxSubsystem::CWXMainFrame::oneInstance == nullptr)
  {
    if (WX_SUBSYSTEM_VERBOSE)
    {
      cout << "[WxSubsystem::pushPendingWxRequest] IGNORING request since "
              "app seems already closed.\n";
    }
    delete data;
    return;
  }

  auto& wxd = WxSubSystemGlobalData::Instance();
  std::lock_guard<std::mutex> locker(wxd.cs_listPendingWxRequests);
  wxd.listPendingWxRequests.push(data);
}

// ---------------------------------------------------------------------------
// Timer callback: process all pending inter-thread requests
// ---------------------------------------------------------------------------
void WxSubsystem::CWXMainFrame::OnTimerProcessRequests(wxTimerEvent& /*event*/)
{
  bool app_closed = false;
  try
  {
    if (WX_SUBSYSTEM_VERBOSE)
    {
      cout << "[OnTimerProcessRequests] Entering\n";
    }

    TRequestToWxMainThread* msg = nullptr;
    while (nullptr != (msg = popPendingWxRequest()))
    {
      switch (msg->OPCODE)
      {
        // ----------------------------------------------------------------
        // CDisplayWindow (2D)
        // ----------------------------------------------------------------
        case OpCode::WIN2D_CREATE:
        {
          if (msg->source2D != nullptr)
          {
            auto* wnd =
                new CWindowDialog(msg->source2D, this, wxID_ANY, msg->str, wxSize(msg->x, msg->y));

            // Hand the new wxFrame pointer back to the waiting caller.
            auto** outPtr = static_cast<void**>(msg->voidPtr);
            *outPtr = static_cast<void*>(wnd);

            msg->source2D->notifySemThreadReady();
            wnd->Show();
          }
        }
        break;

        case OpCode::WIN2D_UPDATE_IMAGE:
        {
          if (msg->source2D != nullptr)
          {
            auto* wnd = static_cast<CWindowDialog*>(msg->voidPtr);
            if (wnd == nullptr)
            {
              break;
            }
            auto* img = static_cast<wxImage*>(msg->voidPtr2);
            if (img == nullptr)
            {
              break;
            }

            wnd->m_image->AssignImage(new wxBitmap(*img));

            if (wnd->m_image->GetSize().GetX() != img->GetWidth() &&
                wnd->m_image->GetSize().GetY() != img->GetHeight())
            {
              wnd->m_image->SetSize(img->GetWidth(), img->GetHeight());
              wnd->m_image->SetMinSize(wxSize(img->GetWidth(), img->GetHeight()));
              wnd->m_image->SetMaxSize(wxSize(img->GetWidth(), img->GetHeight()));
              wnd->Fit();
            }
            delete img;
            wnd->m_image->Refresh(false);  // false: don't erase bg (avoids flicker)
            wnd->m_image->Update();
          }
        }
        break;

        case OpCode::WIN2D_SET_POS:
        {
          if (msg->source2D != nullptr)
          {
            auto* wnd = static_cast<CWindowDialog*>(msg->source2D->getWxObject());
            if (wnd != nullptr)
            {
              wnd->SetSize(msg->x, msg->y, wxDefaultCoord, wxDefaultCoord);
            }
          }
        }
        break;

        case OpCode::WIN2D_SET_SIZE:
        {
          if (msg->source2D != nullptr)
          {
            auto* wnd = static_cast<CWindowDialog*>(msg->source2D->getWxObject());
            if (wnd != nullptr)
            {
              wnd->SetClientSize(msg->x, msg->y);
            }
          }
        }
        break;

        case OpCode::WIN2D_SET_TITLE:
        {
          if (msg->source2D != nullptr)
          {
            auto* wnd = static_cast<CWindowDialog*>(msg->source2D->getWxObject());
            if (wnd != nullptr)
            {
              wnd->SetTitle(msg->str.c_str());
            }
          }
        }
        break;

        case OpCode::WIN2D_DESTROY:
        {
          if (msg->source2D != nullptr)
          {
            auto* wnd = static_cast<CWindowDialog*>(msg->source2D->getWxObject());
            if (wnd != nullptr)
            {
              wnd->Close();
            }
          }
        }
        break;

        // ----------------------------------------------------------------
        // CDisplayWindow3D
        // ----------------------------------------------------------------
        case OpCode::WIN3D_CREATE:
        {
          if (msg->source3D != nullptr)
          {
            auto* wnd = new C3DWindowDialog(
                msg->source3D, this, wxID_ANY, msg->str, wxSize(msg->x, msg->y));

            auto** outPtr = static_cast<void**>(msg->voidPtr);
            *outPtr = static_cast<void*>(wnd);

            msg->source3D->notifySemThreadReady();
            wnd->Show();
          }
        }
        break;

        case OpCode::WIN3D_SET_POS:
        {
          if (msg->source3D != nullptr)
          {
            auto* wnd = static_cast<C3DWindowDialog*>(msg->source3D->getWxObject());
            if (wnd != nullptr)
            {
              wnd->SetSize(msg->x, msg->y, wxDefaultCoord, wxDefaultCoord);
            }
          }
        }
        break;

        case OpCode::WIN3D_SET_SIZE:
        {
          if (msg->source3D != nullptr)
          {
            auto* wnd = static_cast<C3DWindowDialog*>(msg->source3D->getWxObject());
            if (wnd != nullptr)
            {
              wnd->SetClientSize(msg->x, msg->y);
            }
          }
        }
        break;

        case OpCode::WIN3D_SET_TITLE:
        {
          if (msg->source3D != nullptr)
          {
            auto* wnd = static_cast<C3DWindowDialog*>(msg->source3D->getWxObject());
            if (wnd != nullptr)
            {
              wnd->SetTitle(msg->str.c_str());
            }
          }
        }
        break;

        case OpCode::WIN3D_FORCE_REPAINT:
        {
          if (msg->source3D != nullptr)
          {
            auto* wnd = static_cast<C3DWindowDialog*>(msg->source3D->getWxObject());
            if (wnd != nullptr)
            {
              wnd->Refresh(false);
            }
          }
        }
        break;

        case OpCode::WIN3D_DESTROY:
        {
          if (msg->source3D != nullptr)
          {
            auto* wnd = static_cast<C3DWindowDialog*>(msg->source3D->getWxObject());
            if (wnd != nullptr)
            {
              wnd->Close();
            }
          }
        }
        break;

        // ----------------------------------------------------------------
        // CDisplayWindowPlots
        // ----------------------------------------------------------------
        case OpCode::PLOTS_CREATE:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = new CWindowDialogPlots(
                msg->sourcePlots, this, wxID_ANY, msg->str, wxSize(msg->x, msg->y));

            auto** outPtr = static_cast<void**>(msg->voidPtr);
            *outPtr = static_cast<void*>(wnd);

            msg->sourcePlots->notifySemThreadReady();
            wnd->Show();
          }
        }
        break;

        case OpCode::PLOTS_SET_POS:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              wnd->SetSize(msg->x, msg->y, wxDefaultCoord, wxDefaultCoord);
            }
          }
        }
        break;

        case OpCode::PLOTS_SET_SIZE:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              wnd->SetClientSize(msg->x, msg->y);
            }
          }
        }
        break;

        case OpCode::PLOTS_SET_TITLE:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              wnd->SetTitle(msg->str.c_str());
            }
          }
        }
        break;

        case OpCode::PLOTS_SET_MOUSE_PANZOOM:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              wnd->m_plot->EnableMousePanZoom(msg->boolVal);
            }
          }
        }
        break;

        case OpCode::PLOTS_SET_ASPECT_RATIO:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              wnd->m_plot->LockAspect(msg->boolVal);
            }
          }
        }
        break;

        case OpCode::PLOTS_ZOOM_RECT:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              if (msg->vector_x.size() == 2 && msg->vector_y.size() == 2)
              {
                wnd->m_plot->Fit(
                    msg->vector_x[0], msg->vector_x[1], msg->vector_y[0], msg->vector_y[1]);
                wnd->m_plot->LockAspect(msg->boolVal);
              }
            }
          }
        }
        break;

        case OpCode::PLOTS_AXIS_FIT:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              wnd->m_plot->LockAspect(msg->boolVal);
              wnd->m_plot->Fit();
            }
          }
        }
        break;

        case OpCode::PLOTS_CLEAR:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              wnd->m_plot->DelAllLayers(true, true);
              wnd->m_plot->AddLayer(new mpScaleX());
              wnd->m_plot->AddLayer(new mpScaleY());
            }
          }
        }
        break;

        case OpCode::PLOTS_ADD_LINE:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              wnd->plot(msg->vector_x, msg->vector_y, msg->str, msg->plotName);
            }
          }
        }
        break;

        case OpCode::PLOTS_ADD_ELLIPSE:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              wnd->plotEllipse(msg->vector_x, msg->vector_y, msg->str, msg->plotName, msg->boolVal);
            }
          }
        }
        break;

        case OpCode::PLOTS_ADD_BITMAP:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              wnd->image(
                  msg->voidPtr2, msg->vector_x[0], msg->vector_x[1], msg->vector_x[2],
                  msg->vector_x[3], msg->plotName);
            }
          }
        }
        break;

        case OpCode::PLOTS_INSERT_SUBMENU:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              const auto MENUITEM_ID = wxNewId();
              wnd->m_ID2ID[MENUITEM_ID] = msg->x;

              wxMenu* popupMnu = wnd->m_plot->GetPopupMenu();
              if (wnd->m_firstSubmenu)
              {
                wnd->m_firstSubmenu = false;
                popupMnu->InsertSeparator(0);
              }
              auto* mnuTarget = new wxMenuItem(
                  popupMnu, MENUITEM_ID, msg->plotName.c_str(), wxEmptyString, wxITEM_NORMAL);
              popupMnu->Insert(0, mnuTarget);

              wnd->Bind(wxEVT_MENU, &CWindowDialogPlots::OnMenuSelected, wnd, MENUITEM_ID);
            }
          }
        }
        break;

        case OpCode::PLOTS_DESTROY:
        {
          if (msg->sourcePlots != nullptr)
          {
            auto* wnd = static_cast<CWindowDialogPlots*>(msg->sourcePlots->getWxObject());
            if (wnd != nullptr)
            {
              wnd->Close();
            }
          }
        }
        break;

        // ----------------------------------------------------------------
        // Camera-selection dialog
        // ----------------------------------------------------------------
        case OpCode::CAMERA_SELECT_DIALOG:
        {
          if (msg->sourceCameraSelectDialog)
          {
            // voidPtr and voidPtr2 carry std::promise pointers that were
            // created in the calling thread.  We receive them as void*
            // and use static_cast (they were stored via static_cast in
            // the sender, so the types match exactly — no UB).
            auto* readySem = static_cast<std::promise<void>*>(msg->voidPtr);
            auto* resultPromise =
                static_cast<std::promise<mrpt::gui::detail::TReturnAskUserOpenCamera>*>(
                    msg->voidPtr2);

            auto dlg = std::make_unique<CDialogAskUserForCamera>();

            // Signal that the dialog window is now up.
            readySem->set_value();

            const bool wasOk = (dlg->ShowModal() == wxID_OK);

            mrpt::gui::detail::TReturnAskUserOpenCamera ret;
            mrpt::config::CConfigFileMemory c;
            dlg->panel->writeConfigFromVideoSourcePanel("CONFIG", &c);
            ret.selectedConfig = c.getContent();
            ret.accepted_by_user = wasOk;

            resultPromise->set_value(std::move(ret));
            dlg->Close();
          }
        }
        break;

        // ----------------------------------------------------------------
        // Execute arbitrary callable in the GUI thread
        // ----------------------------------------------------------------
        case OpCode::RUN_USER_FUNCTION:
        {
          try
          {
            msg->userFunction();
          }
          catch (const std::exception& e)
          {
            std::cerr << "[WxSubsystem] Exception in userFunction():\n" << e.what() << "\n";
          }
        }
        break;

        // ----------------------------------------------------------------
        // Shutdown
        // ----------------------------------------------------------------
        case OpCode::SHUTDOWN:
        {
          if (WX_SUBSYSTEM_VERBOSE)
          {
            cout << "[WxSubsystem::SHUTDOWN] Initiating shutdown\n";
          }
          app_closed = true;
          if (WxSubsystem::CWXMainFrame::oneInstance != nullptr)
          {
            // Cast away volatile: safe here because we are executing in
            // the wx thread that owns this object.
            auto* frame = const_cast<CWXMainFrame*>(
                static_cast<volatile const CWXMainFrame*>(WxSubsystem::CWXMainFrame::oneInstance));
            ASSERT_(frame);
            frame->Close();
          }
          if (WX_SUBSYSTEM_VERBOSE)
          {
            cout << "[WxSubsystem::SHUTDOWN] Done\n";
          }
        }
        break;

      }  // end switch (msg->OPCODE)

      delete msg;
    }  // end while (pending requests)
  }
  catch (const std::exception& e)
  {
    // Swallow so the timer loop is never broken by an unexpected exception.
    std::cerr << "[WxSubsystem] Exception in main loop: " << e.what() << "\n";
  }

  if (!app_closed)
  {
    m_theTimer->Start(10, true);
  }  // Restart one-shot timer.
}

// ---------------------------------------------------------------------------
// MRPT default icon (XPM)
// ---------------------------------------------------------------------------
namespace
{
const char* mrpt_default_icon_xpm[] = {
    "32 32 2 1",
    " 	c None",
    ".	c #000000",
    "                                ",
    "                                ",
    "                                ",
    " .....     ..... .........      ",
    "  ....     ....   ...   ....    ",
    "  .....    ....   ...    ...    ",
    "  . ...   . ...   ...    ...    ",
    "  . ...   . ...   ...    ...    ",
    "  .  ... .  ...   ...   ...     ",
    "  .  ... .  ...   ........      ",
    "  .  .....  ...   ... ....      ",
    "  .   ...   ...   ...  ....     ",
    "  .   ...   ...   ...   ....    ",
    "  .   ..    ...   ...    ....   ",
    " ...   .   ..... .....    ..... ",
    "                                ",
    "                                ",
    "    ........     ...........    ",
    "     ...  ....   ..  ...  ..    ",
    "     ...   ...   .   ...   .    ",
    "     ...   ...       ...        ",
    "     ...   ...       ...        ",
    "     ...  ...        ...        ",
    "     .......         ...        ",
    "     ...             ...        ",
    "     ...             ...        ",
    "     ...             ...        ",
    "     ...             ...        ",
    "    .....           .....       ",
    "                                ",
    "                                ",
    "                                "};
}

wxBitmap WxSubsystem::getMRPTDefaultIcon()
{
#ifdef _WIN32
  const wxSize iconsSize(::GetSystemMetrics(SM_CXICON), ::GetSystemMetrics(SM_CYICON));
  return wxBitmap(wxBitmap(mrpt_default_icon_xpm).ConvertToImage().Scale(iconsSize.x, iconsSize.y));
#else
  return wxBitmap(mrpt_default_icon_xpm);
#endif
}

// ---------------------------------------------------------------------------
// The wx application object
// ---------------------------------------------------------------------------
class CDisplayWindow_WXAPP : public wxApp
{
 public:
  bool OnInit() override;
  int OnExit() override;
};

bool CDisplayWindow_WXAPP::OnInit()
{
  // Reset numeric locale to "C" so floating-point I/O uses '.' everywhere.
  wxSetlocale(LC_NUMERIC, wxString(wxT("C")));
  wxInitAllImageHandlers();

  auto* Frame = new WxSubsystem::CWXMainFrame(nullptr);
  Frame->Hide();

  // Signal the waiting main thread.
  WxSubsystem::GetWxMainThreadInstance().m_semWxMainThreadReady.set_value();

  return true;
}

int CDisplayWindow_WXAPP::OnExit()
{
  if (WX_SUBSYSTEM_VERBOSE)
  {
    cout << "[wxApp::OnExit] wxApplication OnExit called.\n";
  }

  std::lock_guard<std::mutex> lock(WxSubsystem::GetWxMainThreadInstance().m_csWxMainThreadId);

  wxApp::OnExit();
  CleanUp();
  return 0;
}

// ---------------------------------------------------------------------------
// waitWxShutdownsIfNoWindows
// ---------------------------------------------------------------------------
void WxSubsystem::waitWxShutdownsIfNoWindows()
{
#ifndef WXSHUTDOWN_DO_IT_CLEAN

  if (WX_SUBSYSTEM_VERBOSE)
  {
    cout << "[WxSubsystem::waitWxShutdownsIfNoWindows] Quick sleep and return.\n";
  }
  std::this_thread::sleep_for(100ms);

#else

  auto& wxd = WxSubSystemGlobalData::Instance();
  int nOpenWnds;
  {
    std::lock_guard<std::mutex> lock(wxd.cs_windowCount);
    nOpenWnds = wxd.windowCount;
  }

  if (!nOpenWnds && WxSubsystem::isConsoleApp())
  {
    if (WXSUBSYSTEM_VERBOSE)
    {
      cout << "[WxSubsystem::waitWxShutdownsIfNoWindows] Waiting for "
              "WxWidgets thread to shut down...\n";
    }

    const int maxTimeout =
#ifdef _DEBUG
        30000;
#else
        5000;
#endif
    if (GetWxMainThreadInstance().m_done.get_future().wait_for(
            std::chrono::milliseconds(maxTimeout)) == std::future_status::timeout)
    {
      cerr << "[WxSubsystem::waitWxShutdownsIfNoWindows] Timeout waiting "
              "for WxWidgets thread to shut down!\n";
    }
  }
#endif
}

// ---------------------------------------------------------------------------
// wxEntry helpers
// ---------------------------------------------------------------------------
extern CDisplayWindow_WXAPP& wxGetApp();

namespace
{
wxAppConsole* mrpt_wxCreateApp()
{
  wxAppConsole::CheckBuildOptions(WX_BUILD_OPTIONS_SIGNATURE, "your program");
  return new CDisplayWindow_WXAPP;
}

int mrpt_wxEntryReal()
{
  if (!wxInitialize())
  {
#if wxUSE_LOG
    delete wxLog::SetActiveTarget(nullptr);
#endif
    return -1;
  }

  try
  {
    if (!wxTheApp->CallOnInit())
    {
      return -1;
    }

    const int ret = wxTheApp->OnRun();

    {
      wxLogNull logNo;
      wxTheApp->OnExit();
      wxEntryCleanup();
    }

    return ret;
  }
  catch (...)
  {
    wxTheApp->OnUnhandledException();
    wxEntryCleanup();
    return -1;
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// wxMainThread — the dedicated wx message-loop thread for console apps
// ---------------------------------------------------------------------------
void WxSubsystem::wxMainThread()
{
  MRPT_START

  if (WX_SUBSYSTEM_VERBOSE)
  {
    cout << "[wxMainThread] Starting...\n";
  }

  wxAppConsole* app_gui = wxApp::GetInstance();
  if (app_gui == nullptr)
  {
    // Console application: spin up our own wx app instance.
    if (WX_SUBSYSTEM_VERBOSE)
    {
      cout << "[wxMainThread] Running as console app\n";
    }

    wxApp::SetInitializerFunction(static_cast<wxAppInitializerFunction>(mrpt_wxCreateApp));
    mrpt_wxEntryReal();

    if (WX_SUBSYSTEM_VERBOSE)
    {
      cout << "[wxMainThread] Finished\n";
    }

    GetWxMainThreadInstance().m_done.set_value();
  }
  else
  {
    // GUI application: a wxApp already exists; just create our hidden frame.
    if (WX_SUBSYSTEM_VERBOSE)
    {
      cout << "[wxMainThread] Running inside an existing wxApp\n";
    }

    wxWindow* topWin = static_cast<wxApp*>(app_gui)->GetTopWindow();
    auto* Frame = new WxSubsystem::CWXMainFrame(topWin);
    Frame->Hide();

    if (WX_SUBSYSTEM_VERBOSE)
    {
      cout << "[wxMainThread] Signaling semaphore.\n";
    }

    GetWxMainThreadInstance().m_semWxMainThreadReady.set_value();
  }

  MRPT_END
}

// ---------------------------------------------------------------------------
// GetWxMainThreadInstance
//
// NOTE: This intentionally leaks the TWxMainThreadData object so that it
// outlives any static destructors that might still use it.  This is a
// known pre-existing design choice in the original code.
// ---------------------------------------------------------------------------
WxSubsystem::TWxMainThreadData& WxSubsystem::GetWxMainThreadInstance()
{
  static TWxMainThreadData* dat = nullptr;
  static bool first_creat = true;
  if (!dat && first_creat)
  {
    first_creat = false;
    dat = new TWxMainThreadData;
  }
  return *dat;
}

// ---------------------------------------------------------------------------
// createOneInstanceMainThread
// ---------------------------------------------------------------------------
bool WxSubsystem::createOneInstanceMainThread()
{
  TWxMainThreadData& wxmtd = GetWxMainThreadInstance();
  std::lock_guard<std::mutex> lock(wxmtd.m_csWxMainThreadId);

  wxAppConsole* app_con = wxApp::GetInstance();
  if (app_con != nullptr && wxmtd.m_wxMainThreadId.get_id() == std::thread::id())
  {
    // A wxApp instance already exists (user GUI app); don't create our own.
    isConsoleApp_value = false;
    if (CWXMainFrame::oneInstance == nullptr)
    {
      wxWindow* topWin = dynamic_cast<wxApp*>(app_con)->GetTopWindow();
      ASSERT_(topWin);
      auto* Frame = new CWXMainFrame(topWin);
      Frame->Hide();
    }
  }
  else
  {
    isConsoleApp_value = true;
    if (wxmtd.m_wxMainThreadId.get_id() == std::thread::id())
    {
      if (WX_SUBSYSTEM_VERBOSE)
      {
        printf(
            "[WxSubsystem::createOneInstanceMainThread] Launching "
            "wxMainThread() thread...\n");
      }

      wxmtd.m_wxMainThreadId = std::thread(wxMainThread);
      mrpt::system::thread_name("wxMainThread", wxmtd.m_wxMainThreadId);

      int maxTimeout =
#ifdef _DEBUG
          30000;
#else
          5000;
#endif

      if (const char* envVal = getenv("MRPT_WXSUBSYS_TIMEOUT_MS"))
      {
        maxTimeout = atoi(envVal);
      }

      if (wxmtd.m_semWxMainThreadReady.get_future().wait_for(
              std::chrono::milliseconds(maxTimeout)) == std::future_status::timeout)
      {
        cerr << "[WxSubsystem::createOneInstanceMainThread] Timeout "
                "waiting for wxApplication to start up!\n";
        return false;
      }
    }
  }

  return true;
}

#endif  // MRPT_HAS_WXWIDGETS