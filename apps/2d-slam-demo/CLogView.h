/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef CLOGVIEW_H
#define CLOGVIEW_H

//(*Headers(CLogView)
#include <wx/button.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/textctrl.h>
//*)

class CLogView : public wxDialog
{
   public:
	CLogView(
		wxWindow* parent, wxWindowID id = wxID_ANY,
		const wxPoint& pos = wxDefaultPosition,
		const wxSize& size = wxDefaultSize);
	~CLogView() override;

	//(*Declarations(CLogView)
	wxButton* btnOk;
	wxTextCtrl* edLog;
	//*)

   protected:
	//(*Identifiers(CLogView)
	static const long ID_TEXTCTRL1;
	static const long ID_BUTTON1;
	//*)

   private:
	//(*Handlers(CLogView)
	void OnbtnOkClick(wxCommandEvent& event);
	//*)

	DECLARE_EVENT_TABLE()
};

#endif
