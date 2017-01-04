/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "gui-precomp.h"   // Precompiled headers

#include <mrpt/config.h>

#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>

#include <mrpt/system/os.h>

#include <mrpt/gui/WxSubsystem.h>
#include <mrpt/gui/WxUtils.h>

//#define  WXSUBSYSTEM_VERBOSE

// ------------------------------------------------------------------------
// Defined: Try to wait for all windows & the thread to exit cleanly.
// Undefined: Just to a sleep() and quit crossing our fingers.
//
//  Problem with the "clean way" is: As of feb/2011, I get this error
//   at the end:
//  ** (MRPT:11711): CRITICAL **: giop_thread_request_push: assertion `tdata != NULL' failed
// ------------------------------------------------------------------------
//#define WXSHUTDOWN_DO_IT_CLEAN

#if MRPT_HAS_WXWIDGETS


using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::synch;
using namespace std;

synch::CCriticalSection  	WxSubsystem::CWXMainFrame::cs_windowCount;
int                       	WxSubsystem::CWXMainFrame::m_windowCount = 0;

std::queue<WxSubsystem::TRequestToWxMainThread*> * WxSubsystem::listPendingWxRequests = NULL;
synch::CCriticalSection             * WxSubsystem::cs_listPendingWxRequests = NULL;

volatile WxSubsystem::CWXMainFrame* WxSubsystem::CWXMainFrame::oneInstance = NULL;
volatile bool WxSubsystem::isConsoleApp = true;

WxSubsystem::CAuxWxSubsystemShutdowner  WxSubsystem::global_wxsubsystem_shutdown;


// Auxiliary class implementation:
WxSubsystem::CAuxWxSubsystemShutdowner::CAuxWxSubsystemShutdowner()
{
}
WxSubsystem::CAuxWxSubsystemShutdowner::~CAuxWxSubsystemShutdowner()
{
	if (WxSubsystem::isConsoleApp)
	{
#ifdef WXSUBSYSTEM_VERBOSE
		printf("[~CAuxWxSubsystemShutdowner] Sending 999...\n");
#endif
		// Shut down:
		try
		{
			WxSubsystem::TRequestToWxMainThread  *REQ = new WxSubsystem::TRequestToWxMainThread[1];
			REQ->OPCODE   = 999;
			WxSubsystem::pushPendingWxRequest( REQ );

			//mrpt::system::sleep(100); // JL: I found no better way of doing this, sorry :-(  See WxSubsystem::waitWxShutdownsIfNoWindows()
			WxSubsystem::waitWxShutdownsIfNoWindows();
		} catch (...) { } // Just in case we got an out-of-mem error.
	} // is console app.


#ifdef WXSUBSYSTEM_VERBOSE
	printf("[~CAuxWxSubsystemShutdowner] Deleting static objects.\n");
#endif
	// This is the final point where all dynamic memory must be deleted:
	delete &WxSubsystem::GetWxMainThreadInstance();
	delete WxSubsystem::listPendingWxRequests;
	delete WxSubsystem::cs_listPendingWxRequests;
}


// ---------------------------------------------------------------------------------------
// Auxiliary dialog class for the "ask user to open a camera":
// ---------------------------------------------------------------------------------------
class CDialogAskUserForCamera : public wxDialog
{
public:
	mrpt::gui::CPanelCameraSelection *panel;

	static const long ID_BTN_OK;
	static const long ID_BTN_CANCEL;

	CDialogAskUserForCamera() : wxDialog(NULL,wxID_ANY,wxT("Select image source"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE, wxDialogNameStr)
	{
		wxFlexGridSizer *f1 = new wxFlexGridSizer(2, 1, 0, 0);
		panel = new mrpt::gui::CPanelCameraSelection(this, wxID_ANY);
		f1->Add(panel, 1, wxALL|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 5);

		wxFlexGridSizer *f2 = new wxFlexGridSizer(1, 2, 0, 0);
		wxButton *btnOk = new wxButton(this,ID_BTN_OK,wxT("Ok"), wxDefaultPosition, wxDefaultSize);
		wxButton *btnCancel = new wxButton(this,ID_BTN_CANCEL,wxT("Cancel"), wxDefaultPosition, wxDefaultSize);
		f1->Add(f2, 1, wxALL|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 5);

		f2->Add(btnOk, 1, wxALL|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 5);
		f2->Add(btnCancel, 1, wxALL|wxALIGN_BOTTOM|wxALIGN_CENTER_HORIZONTAL, 5);

		Connect(ID_BTN_OK,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDialogAskUserForCamera::OnBtnOk);
		Connect(ID_BTN_CANCEL,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&CDialogAskUserForCamera::OnBtnCancel);

		SetSizer(f1);
		Fit();

		btnOk->SetFocus(); // So the default params can be accepted by just pressing ENTER.
	}

	virtual ~CDialogAskUserForCamera()
	{
	}

	void OnBtnOk(wxCommandEvent& event)
	{
		EndModal(wxID_OK);
	}

	void OnBtnCancel(wxCommandEvent& event)
	{
		EndModal(wxID_CANCEL);
	}
};

const long CDialogAskUserForCamera::ID_BTN_OK = wxNewId();
const long CDialogAskUserForCamera::ID_BTN_CANCEL = wxNewId();


// ---------------------------------------------------------------------------------------
// The wx dummy frame:
// ---------------------------------------------------------------------------------------
BEGIN_EVENT_TABLE(WxSubsystem::CWXMainFrame,wxFrame)

END_EVENT_TABLE()


const long ID_TIMER_WX_PROCESS_REQUESTS = wxNewId();


WxSubsystem::CWXMainFrame::CWXMainFrame(wxWindow* parent,wxWindowID id)
{
    Create(
        parent,
        id,
        _("MRPT-dummy frame window"),
        wxDefaultPosition,
        wxSize(1,1),
        0, //wxDEFAULT_FRAME_STYLE,
        _T("id"));

	if (oneInstance)
	{
		cerr << "[CWXMainFrame] More than one instance running!" << endl;
	}
	oneInstance = this;

    // ------------------------------------------------------------------------------------------
    // Create a timer so requests from the main application thread can be processed regularly:
    // ------------------------------------------------------------------------------------------
    Connect(ID_TIMER_WX_PROCESS_REQUESTS, wxEVT_TIMER ,(wxObjectEventFunction)&CWXMainFrame::OnTimerProcessRequests);
    m_theTimer =  new wxTimer(this, ID_TIMER_WX_PROCESS_REQUESTS );

    m_theTimer->Start( 10, true );  // One-shot
}

WxSubsystem::CWXMainFrame::~CWXMainFrame()
{
#ifdef WXSUBSYSTEM_VERBOSE
	cout << "[CWXMainFrame] Destructor." << endl;
#endif
	delete m_theTimer;
	oneInstance=NULL;

	// Purge all pending requests:
	TRequestToWxMainThread *msg;
	while (NULL!=(msg= popPendingWxRequest()))
		delete[] msg;

}

int WxSubsystem::CWXMainFrame::notifyWindowCreation()
{
	CCriticalSectionLocker	lock(&cs_windowCount);
    return ++m_windowCount;
}

int WxSubsystem::CWXMainFrame::notifyWindowDestruction()
{
	int ret;
	{
		CCriticalSectionLocker	lock(&cs_windowCount);
		ret = --m_windowCount;
	}

	if (ret==0)
	{
		// That was the last window... we should close the wx subsystem:
		if (oneInstance)
		{
#ifdef WXSHUTDOWN_DO_IT_CLEAN
			CWXMainFrame * me = (CWXMainFrame*)(oneInstance); // cast away the "volatile".
			me->Close();
#endif

#ifdef WXSUBSYSTEM_VERBOSE
	cout << "[CWXMainFrame::notifyWindowDestruction] numWindows=0. me->Close() called." << endl;
#endif
		}
	}

	return ret;
}

/** Thread-safe method to return the next pending request, or NULL if there is none (After usage, FREE the memory!)
  */
WxSubsystem::TRequestToWxMainThread * WxSubsystem::popPendingWxRequest()
{
	if (!cs_listPendingWxRequests)
	{
		cs_listPendingWxRequests= new CCriticalSection();
		listPendingWxRequests = new std::queue<TRequestToWxMainThread*>;
	}

	synch::CCriticalSectionLocker	locker( cs_listPendingWxRequests );

    // Is empty?
    if (listPendingWxRequests->empty())
        return NULL;

    TRequestToWxMainThread *ret=listPendingWxRequests->front();
    listPendingWxRequests->pop(); // Remove from the queue

    return ret;
}

/** Thread-safe method to insert a new pending request (The memory must be dinamically allocated with "new T[1]", will be freed by receiver.)
  */
void WxSubsystem::pushPendingWxRequest( WxSubsystem::TRequestToWxMainThread *data )
{
	if (!WxSubsystem::CWXMainFrame::oneInstance)
	{
	#ifdef WXSUBSYSTEM_VERBOSE
		cout << "[WxSubsystem::pushPendingWxRequest] IGNORING request since app seems already closed.\n";
	#endif
		delete[] data;
		return; // wx subsystem already closed, ignore.
	}

	if (!cs_listPendingWxRequests)
	{
		cs_listPendingWxRequests= new CCriticalSection();
		listPendingWxRequests = new std::queue<TRequestToWxMainThread*>;
	}

	CCriticalSectionLocker locker(cs_listPendingWxRequests);
    listPendingWxRequests->push( data );
}


/** This method processes the pending requests from the main MRPT application thread.
  *  The requests may be to create a new window, close another one, change title, etc...
  */
void WxSubsystem::CWXMainFrame::OnTimerProcessRequests(wxTimerEvent& event)
{
	bool app_closed = false;
    try
    {
        TRequestToWxMainThread *msg;

#ifdef WXSUBSYSTEM_VERBOSE
		cout << "[OnTimerProcessRequests] Entering" << endl;
#endif

        // For each pending request:
        while ( NULL != (msg = popPendingWxRequest() ) )
		{
            // Process it:
            switch (msg->OPCODE)
			{
                // CREATE NEW WINDOW
                case 200:
				if (msg->source2D)
                {
                    CWindowDialog  *wnd = new CWindowDialog(msg->source2D, this, (wxWindowID) -1, msg->str, wxSize(msg->x,msg->y)  );

                    // Set the "m_hwnd" member of the window:
                    * ((void**)msg->voidPtr) = (void*)wnd;

                    // Signal to the constructor (still waiting) that the window is now ready so it can continue:
                    msg->source2D->notifySemThreadReady();

                    wnd->Show();
                }
                break;
                // UPDATE IMAGE
                case 201:
                if (msg->source2D)
				{
                    CWindowDialog  *wnd = (CWindowDialog*) msg->voidPtr; //msg->source2D->getWxObject();
					if (!wnd) break;
                    wxImage       *img = (wxImage*) msg->voidPtr2;
					if (!img) break;

					wnd->m_image->AssignImage(new wxBitmap(*img)); // Memory will be freed by the object.

					if (wnd->m_image->GetSize().GetX() != img->GetWidth() &&
						wnd->m_image->GetSize().GetY() != img->GetHeight() )
					{
						wnd->m_image->SetSize( img->GetWidth(), img->GetHeight() );
						wnd->m_image->SetMinSize( wxSize( img->GetWidth(), img->GetHeight() ));
						wnd->m_image->SetMaxSize( wxSize( img->GetWidth(), img->GetHeight() ));
						wnd->Fit();
						//wnd->SetClientSize(img->GetWidth(), img->GetHeight());
					}
					delete img;
					wnd->m_image->Refresh(false); // false: Do NOT erase background: avoid flickering
                }
                break;
                // Set position
                case 202:
                if (msg->source2D)
				{
                    CWindowDialog  *wnd = (CWindowDialog*) msg->source2D->getWxObject();
                    if (wnd)
                        wnd->SetSize( msg->x, msg->y, wxDefaultCoord, wxDefaultCoord );
                }
                break;
                // Set size
                case 203:
                if (msg->source2D)
				{
                    CWindowDialog  *wnd = (CWindowDialog*) msg->source2D->getWxObject();
                    if (wnd)
                        wnd->SetClientSize( msg->x, msg->y );
                }
                break;
                // Set window's title:
                case 204:
                if (msg->source2D)
				{
                    CWindowDialog  *wnd = (CWindowDialog*) msg->source2D->getWxObject();
                    if (wnd)
						wnd->SetTitle( _U(msg->str.c_str()) );
                }
                break;
                // DESTROY EXISTING WINDOW:
                case 299:
                if (msg->source2D)
				{
                    CWindowDialog  *wnd = (CWindowDialog*) msg->source2D->getWxObject();
                    if (wnd)
                    {
                        //delete wnd;
                        wnd->Close();
                    }
                }
                break;

                // CREATE NEW WINDOW
                case 300:
				if (msg->source3D )
                {
                    C3DWindowDialog  *wnd = new C3DWindowDialog(msg->source3D, this, (wxWindowID) -1, msg->str, wxSize(msg->x,msg->y)  );

					// Set the "m_hwnd" member of the window:
                    * ((void**)msg->voidPtr) = (void*)wnd;

                    // Signal to the constructor (still waiting) that the window is now ready so it can continue:
                    msg->source3D->notifySemThreadReady();

                    wnd->Show();
                }
                break;
                // Set position
                case 302:
                if (msg->source3D )
				{
                    C3DWindowDialog  *wnd = (C3DWindowDialog*) msg->source3D->getWxObject();
                    if (wnd)
                        wnd->SetSize( msg->x, msg->y, wxDefaultCoord, wxDefaultCoord );
                }
                break;
                // Set size
                case 303:
                if (msg->source3D )
				{
                    C3DWindowDialog  *wnd = (C3DWindowDialog*) msg->source3D->getWxObject();
                    if (wnd)
                        wnd->SetClientSize( msg->x, msg->y );
                }
                break;
                // Set window's title:
                case 304:
                if (msg->source3D )
				{
                    C3DWindowDialog  *wnd = (C3DWindowDialog*) msg->source3D->getWxObject();
                    if (wnd)
						wnd->SetTitle( _U(msg->str.c_str()) );
                }
                break;
                // FORCE REPAINT
                case 350:
                if (msg->source3D )
				{
                    C3DWindowDialog  *wnd = (C3DWindowDialog*) msg->source3D->getWxObject();
                    if (wnd)
                    {
                        wnd->Refresh(false);
                    }
                }
                break;
                // Add a 2D text message: vector_x: [0]:x, [1]:y, [2,3,4]:R G B, "x": enum of desired font. "y": unique index, "str": String.
                case 360:
                if (msg->source3D )
				{
                    C3DWindowDialog  *wnd = (C3DWindowDialog*) msg->source3D->getWxObject();
                    if (wnd)
                    {
                        wnd->addTextMessage(
							msg->vector_x[0],msg->vector_x[1],
							msg->str,
							mrpt::utils::TColorf(msg->vector_x[2],msg->vector_x[3],msg->vector_x[4]),
							size_t(msg->y),
							mrpt::opengl::TOpenGLFont(msg->x) );
                    }
                }
                break;
                // Clear 2D text messages
                case 361:
                if (msg->source3D )
				{
                    C3DWindowDialog  *wnd = (C3DWindowDialog*) msg->source3D->getWxObject();
                    if (wnd)
                    {
                        wnd->clearTextMessages();
                    }
                }
                break;
                // Add a 2D text message: vector_x: [0]:x, [1]:y, [2,3,4]:R G B, "x": enum of desired font. "y": unique index, "str": String.
                case 362:
                if (msg->source3D )
				{
                    C3DWindowDialog  *wnd = (C3DWindowDialog*) msg->source3D->getWxObject();
                    if (wnd)
                    {
                        wnd->addTextMessage(
							msg->vector_x[0],msg->vector_x[1],
							msg->str,
							mrpt::utils::TColorf(msg->vector_x[2],msg->vector_x[3],msg->vector_x[4]),
							msg->plotName,
							msg->vector_x[5],
							mrpt::opengl::TOpenGLFontStyle(msg->x),
							size_t(msg->y),
							msg->vector_x[6],msg->vector_x[7],
							msg->vector_x[8]!=0,
							mrpt::utils::TColorf(msg->vector_x[9],msg->vector_x[10],msg->vector_x[11])
							);
                    }
                }
                break;

                // DESTROY EXISTING WINDOW:
                case 399:
                if (msg->source3D )
				{
                    C3DWindowDialog  *wnd = (C3DWindowDialog*) msg->source3D->getWxObject();
                    if (wnd)
                    {
                        //delete wnd;
                        wnd->Close();
                    }
                }
                break;

				// CREATE NEW WINDOW
                case 400:
				if (msg->sourcePlots )
                {
                    CWindowDialogPlots  *wnd = new CWindowDialogPlots(msg->sourcePlots, this, (wxWindowID) -1, msg->str, wxSize(msg->x,msg->y) );

                    // Set the "m_hwnd" member of the window:
                    * ((void**)msg->voidPtr) = (void*)wnd;

                    // Signal to the constructor (still waiting) that the window is now ready so it can continue:
                    msg->sourcePlots->notifySemThreadReady();

                    wnd->Show();
                }
                break;
                // Set position
                case 402:
				if (msg->sourcePlots )
                {
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd)
                        wnd->SetSize( msg->x, msg->y, wxDefaultCoord, wxDefaultCoord );
                }
                break;
                // Set size
                case 403:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd)
                        wnd->SetClientSize( msg->x, msg->y );
                }
                break;
                // Set window's title:
                case 404:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd)
						wnd->SetTitle( _U(msg->str.c_str()) );
                }
                break;
                // Mouse pan
                case 410:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd) wnd->m_plot->EnableMousePanZoom(msg->boolVal);
                }
                break;
                // Aspect ratio
                case 411:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd) wnd->m_plot->LockAspect(msg->boolVal);
                }
                break;

                // Zoom over a rectangle vectorx[0-1] & vectory[0-1]
                case 412:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd)
                    {
                    	if (msg->vector_x.size()==2 &&  msg->vector_y.size()==2)
                    	{
							wnd->m_plot->Fit( msg->vector_x[0],msg->vector_x[1],msg->vector_y[0],msg->vector_y[1] );
							wnd->m_plot->LockAspect(msg->boolVal);
                    	}
                    }
                }
                break;
                // Axis fit, with aspect ratio fix to boolVal.
                case 413:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd)
					{
						wnd->m_plot->LockAspect(msg->boolVal);
						wnd->m_plot->Fit();
					}
                }
                break;
                // Clear all objects:
                case 414:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd)
					{
						wnd->m_plot->DelAllLayers(true,true);
						wnd->m_plot->AddLayer( new mpScaleX() );
						wnd->m_plot->AddLayer( new mpScaleY() );
					}
                }
                break;

                // Create/modify 2D plot
                case 420:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd)
						wnd->plot( msg->vector_x, msg->vector_y, msg->str, msg->plotName );
                }
                break;

                // Create/modify 2D ellipse
                case 421:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd)
						wnd->plotEllipse( msg->vector_x, msg->vector_y, msg->str, msg->plotName, msg->boolVal );
                }
                break;

                // Create/modify bitmap image
                case 422:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd)
						wnd->image(
							msg->voidPtr2,
							msg->vector_x[0],
							msg->vector_x[1],
							msg->vector_x[2],
							msg->vector_x[3],
							msg->plotName );
                }
                break;

                // 440: Insert submenu in the popup menu. name=menu label, x=ID
                case 440:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd)
					{
						const long MENUITEM_ID = wxNewId();
						// Remember the association between this ID and the user ID:
						wnd->m_ID2ID[MENUITEM_ID] = msg->x;

						wxMenu *popupMnu = wnd->m_plot->GetPopupMenu();
						if (wnd->m_firstSubmenu)
						{
							wnd->m_firstSubmenu=false;
							popupMnu->InsertSeparator(0);
						}
						wxMenuItem *mnuTarget = new wxMenuItem(popupMnu, MENUITEM_ID, _U(msg->plotName.c_str()), wxEmptyString, wxITEM_NORMAL);
						popupMnu->Insert(0,mnuTarget);

						wnd->Connect( MENUITEM_ID, wxEVT_COMMAND_MENU_SELECTED, (wxObjectEventFunction) &CWindowDialogPlots::OnMenuSelected );

					}
				}
                break;

                // DESTROY EXISTING WINDOW:
                case 499:
                if (msg->sourcePlots )
				{
                	CWindowDialogPlots *wnd = (CWindowDialogPlots*) msg->sourcePlots->getWxObject();
                    if (wnd)
                    {
                        //delete wnd;
                        wnd->Close();
                    }
                }
                break;

                // CREATE NEW WINDOW
                case 700:
				if (msg->sourceCameraSelectDialog)
                {
                	mrpt::synch::CSemaphore *sem = reinterpret_cast<mrpt::synch::CSemaphore*>(msg->voidPtr);

					CDialogAskUserForCamera *dlg = new CDialogAskUserForCamera();
					// Signal that the window is ready:
					sem->release(1);

					// Show
					const bool wasOk = (dlg->ShowModal() == wxID_OK);

					// send selection to caller:
					mrpt::gui::detail::TReturnAskUserOpenCamera *ret = reinterpret_cast<mrpt::gui::detail::TReturnAskUserOpenCamera*>(msg->voidPtr2);

					// Parse selection as a config text block:
					dlg->panel->writeConfigFromVideoSourcePanel("CONFIG",&ret->selectedConfig);
					ret->accepted_by_user = wasOk;

					delete dlg;

					sem->release(1);

                } break;


				// wxSubsystem shutdown:
				case 999:
				{
#ifdef WXSUBSYSTEM_VERBOSE
					cout << "[WxSubsystem:999] Shutdown" << endl;
#endif
					app_closed = true; // Do NOT launch a timer again
					if (WxSubsystem::CWXMainFrame::oneInstance)
						((WxSubsystem::CWXMainFrame*)(WxSubsystem::CWXMainFrame::oneInstance))->Close();
#ifdef WXSUBSYSTEM_VERBOSE
					cout << "[WxSubsystem:999] Shutdown done" << endl;
#endif
				}
				break;

			} // end switch OPCODE

			// Free the memory:
			delete[] msg;
		} // end while
    }
    catch(...)
    {
    }

	if (!app_closed)
		m_theTimer->Start( 10, true );  // One-shot
}

// ---------------------------------------------------------------------------------------
// MRPT Icons
// ---------------------------------------------------------------------------------------
const char * mrpt_default_icon_xpm[] = {
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

wxBitmap WxSubsystem::getMRPTDefaultIcon()
{
	// To avoid an error in wx, always resize the icon to the expected size:
#ifdef MRPT_OS_WINDOWS
	const wxSize iconsSize(::GetSystemMetrics(SM_CXICON), ::GetSystemMetrics(SM_CYICON));
	return wxBitmap(wxBitmap(mrpt_default_icon_xpm).ConvertToImage().Scale(iconsSize.x,iconsSize.y));
#else
	return wxBitmap(mrpt_default_icon_xpm);
#endif
}

// ---------------------------------------------------------------------------------------
// The wx app:
// ---------------------------------------------------------------------------------------
class CDisplayWindow_WXAPP : public wxApp
{
    public:
        virtual bool OnInit();
		virtual int OnExit();
};

bool CDisplayWindow_WXAPP::OnInit()
{
	// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
	//  if we want numbers to use "." in all countries. The App::OnInit() is a perfect place to undo
	//  the default wxWidgets settings. (JL @ Sep-2009)
	wxSetlocale(LC_NUMERIC, wxString(wxT("C")) );

	wxInitAllImageHandlers();

	//cout << "[wxApp::OnInit] wxApplication OnInit called." << endl;

	// Create a dummy frame:
	WxSubsystem::CWXMainFrame* Frame = new WxSubsystem::CWXMainFrame(0);
	Frame->Hide();

	// We are ready!!
	//cout << "[wxMainThread] Signaling semaphore." << endl;
	WxSubsystem::GetWxMainThreadInstance().m_semWxMainThreadReady.release();

	return true;
}

// This will be called when all the windows / frames are closed.
int CDisplayWindow_WXAPP::OnExit()
{
#ifdef WXSUBSYSTEM_VERBOSE
	cout << "[wxApp::OnExit] wxApplication OnExit called." << endl;
#endif

	CCriticalSectionLocker	lock(& WxSubsystem::GetWxMainThreadInstance().m_csWxMainThreadId );

	wxApp::OnExit();
	CleanUp();
	//WxSubsystem::GetWxMainThreadInstance().m_wxMainThreadId.clear(); // Moved to wxMainThread()
	return 0;
}


/** This method must be called in the destructor of the user class FROM THE MAIN THREAD, in order to wait for the shutdown of the wx thread if this was the last open window.
  */
void WxSubsystem::waitWxShutdownsIfNoWindows()
{
#ifndef WXSHUTDOWN_DO_IT_CLEAN

	#ifdef WXSUBSYSTEM_VERBOSE
		cout << "[WxSubsystem::waitWxShutdownsIfNoWindows] Doing a quick sleep() and returning.\n";
	#endif
	mrpt::system::sleep(100);
	return;
#else
	// Just let know a global object that, at its destruction, it must  ....
	// Any open windows?
	int nOpenWnds;
	{
		CCriticalSectionLocker	locker(&CWXMainFrame::cs_windowCount);
		nOpenWnds = CWXMainFrame::m_windowCount;
	}

    if (!nOpenWnds && WxSubsystem::isConsoleApp)
    {
#ifdef WXSUBSYSTEM_VERBOSE
		cout << "[WxSubsystem::waitWxShutdownsIfNoWindows] Waiting for WxWidgets thread to shutdown...\n";
#endif

    	// Then we must be shutting down in the wx thread (we are in the main MRPT application thread)...
    	// Wait until wx is safely shut down:
    	bool done=false;
		int maxTimeout =
#ifdef _DEBUG
			3000;
#else
			500;
#endif
    	while (!done && --maxTimeout >0)
    	{
    		system::sleep(10);
			WxSubsystem::GetWxMainThreadInstance().m_csWxMainThreadId.enter();
			done = WxSubsystem::GetWxMainThreadInstance().m_wxMainThreadId.isClear();
			WxSubsystem::GetWxMainThreadInstance().m_csWxMainThreadId.leave();
    	}

    	if (maxTimeout<=0)
    	{
    		cerr << "[WxSubsystem::waitWxShutdownsIfNoWindows] Timeout waiting for WxWidgets thread to shutdown!" << endl;
    	}
    }
#endif
}

wxAppConsole *mrpt_wxCreateApp()
{
	wxAppConsole::CheckBuildOptions(WX_BUILD_OPTIONS_SIGNATURE, "your program");
	return new CDisplayWindow_WXAPP;
}

//DECLARE_APP(CDisplayWindow_WXAPP)
extern CDisplayWindow_WXAPP& wxGetApp();


// Aux. funcs used in WxSubsystem::wxMainThread
// --------------------------------------------------
int mrpt_wxEntryReal(int argc, char **argv)
{
	// library initialization
	if ( !wxEntryStart(argc, argv) )
	{
#if wxUSE_LOG
		// flush any log messages explaining why we failed
		delete wxLog::SetActiveTarget(NULL);
#endif
		return -1;
	}

	// if wxEntryStart succeeded, we must call wxEntryCleanup even if the code
	// below returns or throws
	try
	{
		// app initialization
		if ( ! wxTheApp->CallOnInit() )
			return -1;	// don't call OnExit() if OnInit() failed

		// app execution
		int ret = wxTheApp->OnRun();

		{
			wxLogNull logNo;  // Skip any warning in this scope.

			wxTheApp->OnExit();  // This replaces the above callOnExit class
			wxEntryCleanup();
		}

		return ret;
	}
	catch(...)
	{
		wxTheApp->OnUnhandledException();
		wxEntryCleanup();
		return -1;
	}

}

/*---------------------------------------------------------------
					wxMainThread
 This will be the "MAIN" of wxWidgets: It starts an application
   object and does not end until all the windows are closed.
 Only for console apps, not for user GUI apps already with wx.
 ---------------------------------------------------------------*/
void WxSubsystem::wxMainThread()
{
	MRPT_START

	// Prepare wxWidgets:
	int argc=1;
	static const char *dummy_prog_name = "./MRPT";
	char *argv[2] = { const_cast<char*>(dummy_prog_name), NULL };

#ifdef WXSUBSYSTEM_VERBOSE
    cout << "[wxMainThread] Starting..." << endl;
#endif

	// Are we in a console or wxGUI application????
	wxAppConsole *app_gui = wxApp::GetInstance();
	if (!app_gui)
	{
		// We are NOT in a wx application (it's a console program)
		// ---------------------------------------------------------
#ifdef WXSUBSYSTEM_VERBOSE
		cout << "[wxMainThread] I am in a console app" << endl;
#endif
		//  Start a new wx application object:

		// JLBC OCT2008: wxWidgets little hack to enable console/gui mixed applications:
		wxApp::SetInitializerFunction( (wxAppInitializerFunction) mrpt_wxCreateApp );
		mrpt_wxEntryReal(argc,argv);

#ifdef WXSUBSYSTEM_VERBOSE
	    cout << "[wxMainThread] Finished" << endl;
#endif

		// Now this thread is ready. The main thread is free to end now:
		WxSubsystem::GetWxMainThreadInstance().m_wxMainThreadId.clear();
	}
	else
	{
		// We are ALREADY in a wx application:
		// ---------------------------------------------------------
#ifdef WXSUBSYSTEM_VERBOSE
		cout << "[wxMainThread] I am in a GUI app" << endl;
#endif
		wxWindow *topWin = static_cast<wxApp*>(app_gui)->GetTopWindow();

		WxSubsystem::CWXMainFrame* Frame = new WxSubsystem::CWXMainFrame( topWin );
		Frame->Hide();

		// We are ready!!
#ifdef WXSUBSYSTEM_VERBOSE
		cout << "[wxMainThread] Signaling semaphore." << endl;
#endif
		WxSubsystem::GetWxMainThreadInstance().m_semWxMainThreadReady.release();

	}

	MRPT_END
}

WxSubsystem::TWxMainThreadData& WxSubsystem::GetWxMainThreadInstance()
{
	//static TWxMainThreadData dat;
	// Create as dynamic memory, since it'll be deleted in CAuxWxSubsystemShutdowner:
	static TWxMainThreadData *dat=NULL;
	static bool first_creat = true;
	if (!dat && first_creat) { first_creat=false; dat = new TWxMainThreadData; }
	return *dat;
}

WxSubsystem::TWxMainThreadData::TWxMainThreadData() :
	m_wxMainThreadId(),
	m_semWxMainThreadReady(0,1),
	m_csWxMainThreadId("csWxMainThreadId")
{
}


/*---------------------------------------------------------------
					createOneInstanceMainThread
 ---------------------------------------------------------------*/
bool WxSubsystem::createOneInstanceMainThread()
{
	WxSubsystem::TWxMainThreadData & wxmtd = WxSubsystem::GetWxMainThreadInstance();
	CCriticalSectionLocker	lock(&wxmtd.m_csWxMainThreadId);

	wxAppConsole *app_con = wxApp::GetInstance();
	if (app_con && wxmtd.m_wxMainThreadId.isClear())
	{
		// We are NOT in a console application: There is already a wxApp instance running and it's not us.
		WxSubsystem::isConsoleApp = false;
		//cout << "[createOneInstanceMainThread] Mode: User GUI." << endl;
		if (!WxSubsystem::CWXMainFrame::oneInstance)
		{
			// Create our main hidden frame:
			wxWindow *topWin = static_cast<wxApp*>(app_con)->GetTopWindow();

			WxSubsystem::CWXMainFrame* Frame = new WxSubsystem::CWXMainFrame( topWin );
			//Frame->Show();
			//SetTopWindow(Frame);
			Frame->Hide();
		}
	}
	else
	{
		//cout << "[createOneInstanceMainThread] Mode: Console." << endl;
		WxSubsystem::isConsoleApp = true;
		if (wxmtd.m_wxMainThreadId.isClear())
		{
#ifdef WXSUBSYSTEM_VERBOSE
		printf("[WxSubsystem::createOneInstanceMainThread] Launching wxMainThread() thread...\n");
#endif
			// Create a thread for message processing there:
			wxmtd.m_wxMainThreadId = system::createThread( wxMainThread );

			int maxTimeout =
#ifdef _DEBUG
			30000;
#else
			5000;
#endif

			// If we have an "MRPT_WXSUBSYS_TIMEOUT_MS" environment variable, use that timeout instead:
			const char *envVal = getenv("MRPT_WXSUBSYS_TIMEOUT_MS");
			if (envVal)
				maxTimeout = atoi(envVal);

			if(! wxmtd.m_semWxMainThreadReady.waitForSignal(maxTimeout) )  // A few secs should be enough...
			{
				cerr << "[WxSubsystem::createOneInstanceMainThread] Timeout waiting wxApplication to start up!" << endl;
				return false;
			}
		}
	}

	return true; // OK
}


#endif // MRPT_HAS_WXWIDGETS
