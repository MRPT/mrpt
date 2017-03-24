/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CDLGLOG_H
#define CDLGLOG_H

//(*Headers(CDlgLog)
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/button.h>
#include <wx/textctrl.h>
#include <wx/timer.h>
//*)

#include <mrpt/gui/CMyRedirector.h>


class CDlgLog: public wxDialog
{
	public:

		CDlgLog(wxWindow* parent,wxWindowID id=wxID_ANY,const wxPoint& pos=wxDefaultPosition,const wxSize& size=wxDefaultSize);
		virtual ~CDlgLog();

		//(*Declarations(CDlgLog)
		wxButton* btnClear;
		wxTimer timDumpLog;
		wxTextCtrl* edLog;
		wxButton* btnSave;
		//*)

	protected:

		//(*Identifiers(CDlgLog)
		static const long ID_BUTTON1;
		static const long ID_BUTTON2;
		static const long ID_TEXTCTRL2;
		static const long ID_TIMER1;
		//*)


		CMyRedirector  *m_redirector;

	private:

		//(*Handlers(CDlgLog)
		void OnClose(wxCloseEvent& event);
		void OntimDumpLogTrigger(wxTimerEvent& event);
		//*)

		DECLARE_EVENT_TABLE()
};

#endif
