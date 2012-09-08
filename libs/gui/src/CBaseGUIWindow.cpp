/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
