/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/gui.h>  // precompiled header

#include <mrpt/gui/CBaseGUIWindow.h>
#include <mrpt/system/os.h>
#include <mrpt/gui/WxSubsystem.h>

#include <mrpt/utils/CStartUpClassesRegister.h>

using namespace mrpt;
using namespace mrpt::gui;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CBaseGUIWindow, CObject,mrpt::gui)

extern CStartUpClassesRegister  mrpt_gui_class_reg;
const int dumm = mrpt_gui_class_reg.do_nothing(); // Avoid compiler removing this class in static linking


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
	const int maxTimeout =
#ifdef _DEBUG
		30000;
#else
		6000;
#endif
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
