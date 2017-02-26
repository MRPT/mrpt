/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "gui-precomp.h"   // Precompiled headers

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/system/os.h>
#include <mrpt/gui/WxSubsystem.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_VIRTUAL_MRPT_OBJECT(CBaseGUIWindow, CObject,mrpt::gui)


/*---------------------------------------------------------------
					Ctor
 ---------------------------------------------------------------*/
CBaseGUIWindow::CBaseGUIWindow(void* winobj_voidptr, int CMD_CREATE_WIN, int CMD_DESTROY_WIN, const std::string &initial_caption )
	: m_CMD_CREATE_WIN(CMD_CREATE_WIN),
	  m_CMD_DESTROY_WIN(CMD_DESTROY_WIN),
	  m_winobj_voidptr(winobj_voidptr),
	  m_semThreadReady(0,1),
	  m_semWindowDestroyed(0,1),
	  m_caption(initial_caption),
	  m_hwnd(NULL),
  	  m_keyPushed(false),
	  m_keyPushedCode(0),
	  m_keyPushedModifier(MRPTKMOD_NONE)
{
}

/*---------------------------------------------------------------
					Create the wx Window
 ---------------------------------------------------------------*/
void CBaseGUIWindow::createWxWindow(unsigned int initialWidth, unsigned int initialHeight)
{
	MRPT_UNUSED_PARAM(initialWidth); MRPT_UNUSED_PARAM(initialHeight);
	MRPT_START
#if MRPT_HAS_WXWIDGETS
	// Create the main wxThread:
	// -------------------------------
	if (!WxSubsystem::createOneInstanceMainThread() )
        return; // Error!

    // Create window:
    WxSubsystem::TRequestToWxMainThread  *REQ = new WxSubsystem::TRequestToWxMainThread[1];
    REQ->source2D = static_cast<gui::CDisplayWindow*>(m_winobj_voidptr);
    REQ->source3D = static_cast<gui::CDisplayWindow3D*>(m_winobj_voidptr);
    REQ->sourcePlots = static_cast<gui::CDisplayWindowPlots*>(m_winobj_voidptr);
    REQ->str      = m_caption;
    REQ->OPCODE   = m_CMD_CREATE_WIN;
	REQ->voidPtr  = m_hwnd.getPtrToPtr();
	REQ->x 		  = initialWidth ;
	REQ->y 		  = initialHeight ;

    WxSubsystem::pushPendingWxRequest( REQ );

    // Wait for the window to realize and signal it's alive:
    if (!WxSubsystem::isConsoleApp)
    {
    	mrpt::system::sleep(20);	// Force at least 1-2 timer ticks for processing the event:
    	wxApp::GetInstance()->Yield(true);
    }
	int maxTimeout =
#ifdef _DEBUG
		30000;
#else
		6000;
#endif
	// If we have an "MRPT_WXSUBSYS_TIMEOUT_MS" environment variable, use that timeout instead:
	const char *envVal = getenv("MRPT_WXSUBSYS_TIMEOUT_MS");
	if (envVal) maxTimeout = atoi(envVal);


	if(!m_semThreadReady.waitForSignal(maxTimeout))  // 2 secs should be enough...
	{
		cerr << "[CBaseGUIWindow::ctor] Timeout waiting window creation." << endl;
	}
#else
	THROW_EXCEPTION("MRPT compiled without wxWidgets!")
#endif
	MRPT_END
}

/*---------------------------------------------------------------
					Dtor
 ---------------------------------------------------------------*/
CBaseGUIWindow::~CBaseGUIWindow()
{
}

/*---------------------------------------------------------------
				destroyWxWindow
 ---------------------------------------------------------------*/
void CBaseGUIWindow::destroyWxWindow()
{
	MRPT_START
#if MRPT_HAS_WXWIDGETS
    // Send close request:
    if (m_hwnd.get())
    {
		WxSubsystem::TRequestToWxMainThread  *REQ = new WxSubsystem::TRequestToWxMainThread[1];
		REQ->OPCODE   = m_CMD_DESTROY_WIN;
		REQ->source2D = static_cast<gui::CDisplayWindow*>(m_winobj_voidptr);
		REQ->source3D = static_cast<gui::CDisplayWindow3D*>(m_winobj_voidptr);
		REQ->sourcePlots = static_cast<gui::CDisplayWindowPlots*>(m_winobj_voidptr);

		WxSubsystem::pushPendingWxRequest( REQ );

		// Wait until the thread ends:
		if (!WxSubsystem::isConsoleApp)
		{
			mrpt::system::sleep(20);	// Force at least 1-2 timer ticks for processing the event:
			wxApp::GetInstance()->Yield(true);
		}
		const int maxTimeout =
	#ifdef _DEBUG
			30000;
	#else
			6000;
	#endif
		if(!m_semWindowDestroyed.waitForSignal(maxTimeout))  // 2 secs should be enough...
		{
			cerr << "[CBaseGUIWindow::dtor] Timeout waiting window destruction." << endl;
		}
    }
	WxSubsystem::waitWxShutdownsIfNoWindows();
#endif
	MRPT_END
}

/*---------------------------------------------------------------
					notifyChildWindowDestruction
 ---------------------------------------------------------------*/
void CBaseGUIWindow::notifyChildWindowDestruction()
{
    //cout << "[CBaseGUIWindow::notifyChildWindowDestruction] Called." << endl;
    m_hwnd = NULL;
}

/*---------------------------------------------------------------
					waitForKey
 ---------------------------------------------------------------*/
int  CBaseGUIWindow::waitForKey(bool ignoreControlKeys,mrptKeyModifier *out_pushModifier)
{
	int k = 0;
	if (out_pushModifier) *out_pushModifier = MRPTKMOD_NONE;
	m_keyPushed=false;

	for (;;)
	{
		if (os::kbhit())
		{
			k=os::getch();
			return k;
		}
		if (m_keyPushed)
		{
			k=m_keyPushedCode;
			m_keyPushed=false;
			if (m_keyPushedCode<256 || !ignoreControlKeys)
			{
				if (out_pushModifier) *out_pushModifier = m_keyPushedModifier;
				return k;
			}
			// Ignore and keep waiting
		}
		mrpt::system::sleep(10);
		// Are we still alive?
		if (!isOpen())
			return 0;
	}
}

/*---------------------------------------------------------------
					getPushedKey
 ---------------------------------------------------------------*/
int  CBaseGUIWindow::getPushedKey(mrptKeyModifier *out_pushModifier)
{
	int k = 0;
	if (out_pushModifier) *out_pushModifier = MRPTKMOD_NONE;

	for (;;)
	{
		if (m_keyPushed)
		{
			k=m_keyPushedCode;
			m_keyPushed=false;
			if (out_pushModifier) *out_pushModifier = m_keyPushedModifier;
			return k;
		}
		mrpt::system::sleep(10);
		// Are we still alive?
		if (!isOpen())
			return 0;
	}
}

/*---------------------------------------------------------------
					isOpen
 ---------------------------------------------------------------*/
bool CBaseGUIWindow::isOpen()
{
    return m_hwnd!=NULL;
}

/*---------------------------------------------------------------
					notifySemThreadReady
 ---------------------------------------------------------------*/
void CBaseGUIWindow::notifySemThreadReady()
{
	m_semThreadReady.release();
}
